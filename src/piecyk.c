#include <stdlib.h>
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

// The LCD configs
struct hd44780fw_conf lcd_conf;
struct hd44780_l_conf lcd_low_conf;

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
	// set other pins as output
	DDRB = ~(1 << PC5);
	// create buffer for samples and empty it
	samples = malloc(SAMPLE_BUFFER_SIZE * sizeof(int8_t));
	for (unsigned int i = 0; i < SAMPLE_BUFFER_SIZE; i++) *(samples + i) = INVALID_TEMP;
}

inline void init_keypad()
{
	DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4));
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

	// cycle buffer
	if (samples_idx >= SAMPLE_BUFFER_SIZE) samples_idx = 0;
}

int get_avg_temp()
{
	unsigned int counted = SAMPLE_BUFFER_SIZE;
	long sum = 0;

	for (unsigned int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
		if (*(samples + i) == INVALID_TEMP) {
			counted--;
			continue;
		}
		sum += *(samples + i);
	}
	return sum / counted;
}

void display_temp()
{
	char *buf = NULL;
	buf = itoa(get_avg_temp(), malloc(4), 10);
	hd44780fw_write(&lcd_conf, buf, CUR_VAL_IDX, HD44780FW_WR_NO_CLEAR_BEFORE);
#ifdef DEBUG
	// Write the samples counter to the display, just after "Temp"
	buf = itoa(samples_idx, buf, 10);
	hd44780fw_write(&lcd_conf, buf, 4, HD44780FW_WR_NO_CLEAR_BEFORE);
#endif
	free(buf);
}

inline void refresh_display()
{
	// Write the template strings to the display
	hd44780fw_write(&lcd_conf, HEADER, 0, HD44780FW_WR_NO_CLEAR_BEFORE);
	hd44780fw_write(&lcd_conf, VALUES, 0x10, HD44780FW_WR_NO_CLEAR_BEFORE);
	// Update values
	display_temp();
}

int main()
{
	init_display();
	init_analog_temp();
	for (unsigned int lcx = 0;; lcx++) {
		read_temp();
		_delay_ms(500);
#ifdef DEBUG
		refresh_display();
#else
		// Refresh display every 10 samples
		if (lcx == 10) {
			refresh_display();
			lcx = 0;
		}
#endif
	}
	return 0;
}
