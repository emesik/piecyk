#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <hd44780_low.h>
#include <hd44780fw.h>

/* The sensor shows some kind of variarion, jumping even a full centigrade every
 * few seconds. For the heating we need very slow responsiveness, so the samples
 * will be gathered in a table and averaged.
 */
#define		SAMPLE_BUFFER_SIZE	240		// 240B makes almost 1/4 of the RAM
#define		INVALID_TEMP		-128	// mordor
int8_t *samples;	// one byte per sample, will allocate later
unsigned int samples_idx = 0;
uint8_t buffer_filled = 0;
#ifdef DEBUG
volatile int last_adcval;
#endif

#define		MIN_VALID_TEMP	0
#define		MAX_VALID_TEMP	50
uint8_t temp_min = 15, temp_max = 25;

// The LCD configs
struct hd44780fw_conf lcd_conf;
struct hd44780_l_conf lcd_low_conf;
volatile uint8_t display_needs_refresh = 0;

/*
 *	The LCD desired structure during normal operation
 *	   0123456789abcdef
 *
 *	0  Temp   Min   Max
 *	1   20C  *20C  *20C
 *
 *	         ^-----^-- indicates the value is being edited
 *
 */
#define		HEADER_IDX		0x00
#define		GATHER_IDX		0x10
#define		CUR_VAL_IDX		0x11
#define		MIN_EDIT_IDX	0x16
#define		MIN_VAL_IDX		0x17
#define		MAX_EDIT_IDX	0x1c
#define		MAX_VAL_IDX		0x1d

#define		HEADER			"Temp   Min   Max"
#define		VALUES			"   C     C     C"
#define		INTRO0			"piecyk v0.1"
#define		INTRO1			"zaraz grzejemy"

inline void init_display()
{
	// pins connected to PORTB
	DDRB |= (1 << 1) | (1 << 2) | (1 << 6) | (1 << 7);
	lcd_low_conf.rs_i = 2;
	lcd_low_conf.rw_i = 1;
	lcd_low_conf.en_i = 6;
	lcd_low_conf.db4_i = 7;
	lcd_low_conf.rs_port = lcd_low_conf.rw_port =
		lcd_low_conf.en_port = lcd_low_conf.db4_port = &PORTB;

	// pins connected to PORTD
	DDRD |= (1 << 5) | (1 << 6) | (1 << 7);
	lcd_low_conf.db5_i = 5;
	lcd_low_conf.db6_i = 6;
	lcd_low_conf.db7_i = 7;
	lcd_low_conf.db5_port = lcd_low_conf.db6_port =
		lcd_low_conf.db7_port = &PORTD;

	lcd_low_conf.line1_base_addr = 0x00;
	lcd_low_conf.line2_base_addr = 0x40;
	lcd_low_conf.dl = HD44780_L_FS_DL_4BIT;
	lcd_conf.low_conf = &lcd_low_conf;
	lcd_conf.total_chars = 32;
	lcd_conf.font = HD44780_L_FS_F_58;
	lcd_conf.lines = HD44780_L_FS_N_DUAL;

	hd44780fw_init(&lcd_conf);
	hd44780fw_write(&lcd_conf, INTRO0, 0, HD44780FW_WR_CLEAR_BEFORE);
	hd44780fw_write(&lcd_conf, INTRO1, 0x10, HD44780FW_WR_NO_CLEAR_BEFORE);

	display_needs_refresh = 1;
}

inline void init_analog_temp()
{
	/*
	 * Based on https://sites.google.com/site/qeewiki/books/avr-guide/analog-input
	 *
	 * Initialize ADMUX register with new value
	 * REFS0 = 1, REFS1 = 1:	use Aref
	 * ADLAR = 0:				10-bit resolution
	 * MUX = 5:					ADC5 (PC5) is the input
	 */
	ADMUX = (1 << REFS0) | (1 << REFS1) | (1 << MUX2) | (1 << MUX0);
	// enable ADC, set 128 prescaler for 8MHz mode
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// set other pins as output and set high state
	DDRB = ~(1 << PC5);
	PORTC |= ~(1 << PC5);
	// create buffer for samples and empty it
	samples = malloc(SAMPLE_BUFFER_SIZE * sizeof(int8_t));
	for (unsigned int i = 0; i < SAMPLE_BUFFER_SIZE; i++) *(samples + i) = INVALID_TEMP;
}

inline void read_temp()
{
	int adcval;
	// start conversion
	ADCSRA |= (1 << ADSC);
	// spinlock until finished
	while (ADCSRA & (1 << ADSC));

	/*
	 * The docs say reading the higher byte causes update. This should not be the case in
	 * per-request conversion, but we keep the convention of reading lower byte first.
	 *
	 */
	adcval = ADCL;
	adcval = (ADCH << 8) + adcval;

	/*
	 * LM35 returns linear function of temperature in Celsius:
	 * Vout = 0V + (t * 0.01V)
	 * As we have 2.56V as Aref and 1024 values of 10-bit resolution,
	 * The only thing we need is to divide the result by 4 with rounding.
	 *
	 */
	*(samples + samples_idx++) = (int8_t)((adcval >> 2) + ((adcval & 2) >> 1));
#ifdef DEBUG
	last_adcval = adcval;
#endif

	// cycle buffer
	if (samples_idx >= SAMPLE_BUFFER_SIZE) samples_idx = 0;
}

int get_avg_temp()
{
	unsigned int counted = SAMPLE_BUFFER_SIZE;
	long sum = 0;

	buffer_filled = 1;
	for (unsigned int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
		if (*(samples + i) == INVALID_TEMP) {
			counted--;
			buffer_filled = 0;
			continue;
		}
		sum += *(samples + i);
	}
	return sum / counted;
}

#define		KEY_EDIT	(1 << PD2)
#define		KEY_DOWN	(1 << PD3)
#define		KEY_UP		(1 << PD4)
#define		KEYS_ALL	(KEY_EDIT | KEY_DOWN | KEY_UP)

#define		EDIT_NONE		0
#define		EDIT_MIN		1
#define		EDIT_MAX		2
#define		EDIT_INVALID	3
volatile uint8_t edit_mode = EDIT_NONE;
#ifdef DEBUG
volatile uint8_t kbd_state_old = 0, kbd_state;
#endif

ISR(TIMER0_OVF_vect)
{
#ifndef DEBUG
	static uint8_t kbd_state_old = 0, kbd_state;
#endif
	static uint8_t needs_reaction = 0, reacted_state = 0;

	// Switches lower the pin state, so we use negative value
	kbd_state = ~PIND & KEYS_ALL;

	/*
	 * Debouncing, as in:
	 * http://mikrokontrolery.blogspot.com/2011/03/epp-eliminacja-drgan-stykow-omicronns.html
	 *
	 */
	if ((kbd_state == kbd_state_old) && (kbd_state != reacted_state)) {
		needs_reaction = 1;
		reacted_state = kbd_state;
	}

	kbd_state_old = kbd_state;

	// React to the keypad state
	if (!needs_reaction) return;
	if (kbd_state & KEY_EDIT) {
		// this is not atomic, so we use a temporary var
		uint8_t _edit_mode = edit_mode;
		_edit_mode++;
		if (_edit_mode == EDIT_INVALID) _edit_mode = EDIT_NONE;
		edit_mode = _edit_mode;
	} else if (kbd_state & KEY_DOWN) {
		if ((edit_mode == EDIT_MIN) && (temp_min > MIN_VALID_TEMP)) temp_min--;
		if ((edit_mode == EDIT_MAX) && (temp_max > MIN_VALID_TEMP) &&
				(temp_max > temp_min)) temp_max--;
	} else if (kbd_state & KEY_UP) {
		if ((edit_mode == EDIT_MIN) && (temp_min < MAX_VALID_TEMP) &&
				(temp_min < temp_max)) temp_min++;
		if ((edit_mode == EDIT_MAX) && (temp_max < MAX_VALID_TEMP)) temp_max++;
	}

	needs_reaction = 0;
	display_needs_refresh = 1;
}

inline void refresh_display()
{
	char *buf = malloc(4);
	// Write the template strings to the display
	hd44780fw_write(&lcd_conf, HEADER, 0, HD44780FW_WR_NO_CLEAR_BEFORE);
	hd44780fw_write(&lcd_conf, VALUES, 0x10, HD44780FW_WR_NO_CLEAR_BEFORE);
	// Update values
	if (buffer_filled)
		hd44780fw_write(&lcd_conf, " ", GATHER_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
	else
		hd44780fw_write(&lcd_conf, "=", GATHER_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
	hd44780fw_write(&lcd_conf, itoa(get_avg_temp(), buf, 10),
			CUR_VAL_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
	hd44780fw_write(&lcd_conf, itoa(temp_min, buf, 10),
			MIN_VAL_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
	hd44780fw_write(&lcd_conf, itoa(temp_max, buf, 10),
			MAX_VAL_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
#ifdef DEBUG
	// Write the samples counter to the display, just after "Temp"
	hd44780fw_write(&lcd_conf, itoa(samples_idx, buf, 10),
			4, HD44780FW_WR_NO_CLEAR_BEFORE);
	// Write the last sample to the display, just after current temp
	hd44780fw_write(&lcd_conf, itoa(last_adcval, buf, 10),
			0x14, HD44780FW_WR_NO_CLEAR_BEFORE);
#endif
	if (edit_mode == EDIT_MIN)
		hd44780fw_write(&lcd_conf, "*", MIN_EDIT_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
	if (edit_mode == EDIT_MAX)
		hd44780fw_write(&lcd_conf, "*", MAX_EDIT_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
#ifdef DEBUG
	hd44780fw_write(&lcd_conf, itoa(kbd_state, buf, 10),
			0xb, HD44780FW_WR_NO_CLEAR_BEFORE);
#endif
	display_needs_refresh = 0;
	free(buf);
}

inline void init_keypad()
{
	DDRD &= ~KEYS_ALL;
	PORTD |= KEYS_ALL;

	// use internal clock, 256 prescaler
	TCCR0 |= (1 << CS02);

	// enable timer overflow interrupt
	TIMSK |= (1 << TOIE0);
}

int main()
{
	init_display();
	init_analog_temp();
	init_keypad();
	sei();
	for (unsigned int lcx = 0;; lcx++) {
		_delay_ms(50);
		if (!(lcx & 7)) {
			// read temp every 8 cycles
			read_temp();
		}

		// refresh when needed or every 64 cycles
		if (display_needs_refresh || (lcx & 0x3f)) {
			refresh_display();
		}
	}
	return 0;
}
