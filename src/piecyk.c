#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <hd44780_low.h>
#include <hd44780fw.h>

FUSES =
{
	.low = LFUSE_DEFAULT & FUSE_BODLEVEL & FUSE_BODEN,
	.high = HFUSE_DEFAULT & FUSE_EESAVE,
	/* WARNING: These settings cannot be written from a .hex file without external tools.
	 * Fusebits are redefined in Makefile and that setting is the working one.
	 */
};

/* The sensor shows some kind of variarion, jumping even a full centigrade every
 * few seconds. For the heating we need very slow responsiveness, so the samples
 * will be gathered in a table and averaged.
 */
#ifdef DEBUG
#define		SAMPLE_BUFFER_SIZE	10
#else
#define		SAMPLE_BUFFER_SIZE	120		// remember this chip has 1K only
#endif
#define		INVALID_TEMP		-(300 << 2)	// mordor
int16_t *samples;		// keep full 10-bit sample
unsigned int samples_idx = 0;
uint8_t buffer_filled = 0;

/* WARNING: all temperatures are measured in 0,5C units, 9 bit */
#define		MIN_VALID_TEMP	0
#define		MAX_VALID_TEMP	(50 << 1)
#define		DEFAULT_MIN		(17 << 1) + 1
#define		DEFAULT_MAX		(20 << 1) + 1
uint16_t volatile temp_min = DEFAULT_MIN, temp_max = DEFAULT_MAX;
uint8_t heating_state = 0;

// the permanent storage of programmed temps and last state
uint16_t EEMEM perm_temp_min = DEFAULT_MIN;
uint16_t EEMEM perm_temp_max = DEFAULT_MAX;
uint8_t EEMEM perm_heating_state = 1;
uint8_t need_store = 0;

inline void restore_parameters()
{
	cli();
	eeprom_busy_wait();
	temp_min = eeprom_read_word(&perm_temp_min);
	temp_max = eeprom_read_word(&perm_temp_max);
	heating_state = eeprom_read_byte(&perm_heating_state);
	sei();
}

void save_parameters()
{
	cli();
	eeprom_busy_wait();
	eeprom_write_word(&perm_temp_min, temp_min);
	eeprom_write_word(&perm_temp_max, temp_max);
	eeprom_write_byte(&perm_heating_state, heating_state);
	sei();
}

// The LCD configs
struct hd44780fw_conf lcd_conf;
struct hd44780_l_conf lcd_low_conf;
volatile uint8_t display_need_refresh = 0;

/*
 *	The LCD desired structure during normal operation
 *	   0123456789abcdef
 *
 *	0  Temp= *Max 21,0C
 *	1  20,5C *Min 20,0C
 *
 *	         ^-------- indicates the value is being edited
 *
 */
#define		DISPLAY_SIZE	0x20
#define		GATHER_IDX		0x04
#define		CUR_VAL_IDX		0x10
#define		MIN_EDIT_IDX	0x16
#define		MIN_VAL_IDX		0x1b
#define		MAX_EDIT_IDX	0x06
#define		MAX_VAL_IDX		0x0b

const char template[DISPLAY_SIZE] PROGMEM	= "Temp   Max     C    C  Min     C";
const char intro[DISPLAY_SIZE] PROGMEM		= "piecyk v0.3     zaraz grzejemy! ";

inline void init_display()
{
	char *display_buf = malloc(DISPLAY_SIZE + 1);

	// pins connected to PORTB
	DDRB |= 1 | (1 << 1) | (1 << 2) | (1 << 7);
	lcd_low_conf.rs_i = 2;
	lcd_low_conf.rw_i = 1;
	lcd_low_conf.en_i = 0;
	lcd_low_conf.db7_i = 7;
	lcd_low_conf.rs_port = lcd_low_conf.rw_port =
		lcd_low_conf.en_port = lcd_low_conf.db7_port = &PORTB;

	// pins connected to PORTD
	DDRD |= (1 << 5) | (1 << 6) | (1 << 7);
	lcd_low_conf.db4_i = 7;
	lcd_low_conf.db5_i = 6;
	lcd_low_conf.db6_i = 5;
	lcd_low_conf.db4_port = lcd_low_conf.db5_port =
		lcd_low_conf.db6_port = &PORTD;

	lcd_low_conf.line1_base_addr = 0x00;
	lcd_low_conf.line2_base_addr = 0x40;
	lcd_low_conf.dl = HD44780_L_FS_DL_4BIT;
	lcd_conf.low_conf = &lcd_low_conf;
	lcd_conf.total_chars = 32;
	lcd_conf.font = HD44780_L_FS_F_58;
	lcd_conf.lines = HD44780_L_FS_N_DUAL;

	hd44780fw_init(&lcd_conf);

	memcpy_P(display_buf, &intro, DISPLAY_SIZE);
	*(display_buf + DISPLAY_SIZE) = '\0';
	hd44780fw_write(&lcd_conf, display_buf, 0, HD44780FW_WR_CLEAR_BEFORE);

	display_need_refresh = 1;
	free(display_buf);
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
	samples = malloc(SAMPLE_BUFFER_SIZE * sizeof(int16_t));
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
	 * the maximum resolution is 0,25C.
	 *
	 * We keep full sample.
	 *
	 */
	*(samples + samples_idx++) = adcval;
	// cycle buffer
	if (samples_idx >= SAMPLE_BUFFER_SIZE) samples_idx = 0;
}

uint16_t get_avg_temp()
{
	unsigned int counted = SAMPLE_BUFFER_SIZE;
	uint32_t sum = 0;
	uint16_t avg;

	buffer_filled = 1;
	for (unsigned int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
		if (*(samples + i) == INVALID_TEMP) {
			counted--;
			buffer_filled = 0;
			continue;
		}
		sum += *(samples + i);
	}
	avg = sum / counted;
	/* The samples are 10-bit integers in 0,25C units.
	 * We use 9-bit integers in 0,5C units as they seem more user-friendly.
	 */
	return (avg >> 1) + (avg & 1);
}

#define		KEY_EDIT	(1 << PD4)
#define		KEY_DOWN	(1 << PD3)
#define		KEY_UP		(1 << PD2)
#define		KEYS_ALL	(KEY_EDIT | KEY_DOWN | KEY_UP)

#define		EDIT_NONE		0
#define		EDIT_MAX		1
#define		EDIT_MIN		2
#define		EDIT_INVALID	3
volatile uint8_t edit_mode = EDIT_NONE;

ISR(TIMER0_OVF_vect)
{
	static uint8_t kbd_state, kbd_state_old = 0;
	static uint8_t need_reaction = 0, reacted_state = 0;

	// Switches lower the pin state, so we use negative value
	kbd_state = ~PIND & KEYS_ALL;

	/*
	 * Debouncing, as in:
	 * http://mikrokontrolery.blogspot.com/2011/03/epp-eliminacja-drgan-stykow-omicronns.html
	 *
	 */
	if ((kbd_state == kbd_state_old) && (kbd_state != reacted_state)) {
		need_reaction = 1;
		reacted_state = kbd_state;
	}

	kbd_state_old = kbd_state;

	// React to the keypad state
	if (!need_reaction) return;
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

	need_store = 1;
	need_reaction = 0;
	display_need_refresh = 1;
}

void temptoa(uint16_t temp, char *rbuf)
{
	/* Converts 9-bit temp to string. Does not terminate the string with \0,
	 * allowing simple pasting into the template. */
	char *buf = malloc(4);
	size_t s = 0;

	itoa((temp >> 1), buf, 10);
	s = strlen(buf);
	memcpy(rbuf + (2-s), buf, s);
	*(rbuf + s) = ',';
	if (temp & 1) *(rbuf + s + 1) = '5';
	else *(rbuf + s + 1) = '0';
	free(buf);
}

void refresh_display()
{
	// allocate once and for all
	static char *display_buf = NULL, *buf = NULL;
	if (display_buf == NULL) display_buf = malloc(DISPLAY_SIZE + 1);
	if (buf == NULL) buf = malloc(4);

	memcpy_P(display_buf, &template, DISPLAY_SIZE);
	*(display_buf + DISPLAY_SIZE) = '\0';
	// Update values
	if (buffer_filled)
		*(display_buf + GATHER_IDX) =  ' ';
	else
		*(display_buf + GATHER_IDX) =  '=';
	temptoa(get_avg_temp(), display_buf + CUR_VAL_IDX);
	temptoa(temp_min, display_buf + MIN_VAL_IDX);
	temptoa(temp_max, display_buf + MAX_VAL_IDX);
	if (edit_mode == EDIT_MIN)
		*(display_buf + MIN_EDIT_IDX) = '*';
	else
		*(display_buf + MIN_EDIT_IDX) = ' ';
	if (edit_mode == EDIT_MAX)
		*(display_buf + MAX_EDIT_IDX) = '*';
	else
		*(display_buf + MAX_EDIT_IDX) = ' ';
	display_need_refresh = 0;
	// Write the string to the display
	hd44780fw_write(&lcd_conf, display_buf, 0, HD44780FW_WR_NO_CLEAR_BEFORE);
}

inline void init_keypad()
{
	DDRD &= ~KEYS_ALL;
	PORTD |= KEYS_ALL;	// pull-up

	// use internal clock, 256 prescaler
	TCCR0 |= (1 << CS02);

	// enable timer overflow interrupt
	TIMSK |= (1 << TOIE0);
}

inline void turn_heating(uint8_t on)
{
	// Pin 0 is control, pin 1 is diode.
	if (on) {
		PORTD &= ~(1 << PD0);
		PORTD |= (1 << PD1);
	} else {
		PORTD |= (1 << PD0);
		PORTD &= ~(1 << PD1);
	}
}

inline void init_heating()
{
	DDRD |= (1 << PD0) | (1 << PD1);
	turn_heating(heating_state);
}

inline void control_heating()
{
	uint16_t temp = get_avg_temp();
	if (!buffer_filled) return;	// no balanced data yet
	if ((temp < temp_min) || (temp > temp_max)) {
		if (temp < temp_min) heating_state = 1;
		else heating_state = 0;
		turn_heating(heating_state);
		need_store = 1;
	}
}

int main()
{
	init_display();
	restore_parameters();
	init_heating();
	init_analog_temp();
	init_keypad();
	sei();
	for (unsigned int lcx = 0;; lcx++) {
		_delay_ms(50);
		if (!(lcx & 7)) {
			// read temp and update heating state every 8 cycles
			read_temp();
			control_heating();
		}

		// refresh when needed or every 64 cycles
		if (display_need_refresh || (lcx & 0x3f)) {
			refresh_display();
		}

		if (need_store) save_parameters();
	}
	return 0;
}
