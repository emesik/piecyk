#include <hd44780_low.h>
#include <hd44780fw.h>

// The LCD configs
struct hd44780fw_conf lcd_conf;
struct hd44780_l_conf lcd_low_conf;

/*
	The LCD structure during normal operation
	   0123456789abcdef

	0  Temp   Min   Max
	1  -20C  *20C  *20C

	         ^-----^-- shows when the value is being edited

*/
#define		HEADER_IDX		0x00
#define		CUR_VAL_IDX		0x10
#define		MIN_EDIT_IDX	0x16
#define		MIN_VAL_IDX		0x17
#define		MAX_EDIT_IDX	0x1c
#define		MAX_VAL_IDX		0x1d

#define		HEADER			"Temp   Min   Max"
#define		INTRO0			"piecyk v0.1"
#define		INTRO1			"zaraz grzejemy"

void init_lcd() {
	DDRC = 0xff;

	// RS and RW are connected to PORTB
	lcd_low_conf.rs_i = 2;
	lcd_low_conf.rw_i = 0;
	lcd_low_conf.rs_port = lcd_low_conf.rw_port = &PORTB;

	// E and 4-bit data pins are connected to PORTD
	lcd_low_conf.en_i = 7;
	lcd_low_conf.db7_i = 6;
	lcd_low_conf.db6_i = 5;
	lcd_low_conf.db5_i = 4;
	lcd_low_conf.db4_i = 3;
	lcd_low_conf.en_port = lcd_low_conf.db7_port = lcd_low_conf.db6_port =
		lcd_low_conf.db5_port = lcd_low_conf.db4_port = &PORTD;

	lcd_low_conf.line1_base_addr = 0x00;
	lcd_low_conf.line2_base_addr = 0x40;
	lcd_low_conf.dl = HD44780_L_FS_DL_4BIT;
	lcd_conf.low_conf = &lcd_low_conf;
	lcd_conf.total_chars = 32;
	lcd_conf.font = HD44780_L_FS_F_58;
	lcd_conf.lines = HD44780_L_FS_N_DUAL;

	hd44780fw_init(&lcd_conf);
	hd44780fw_write(&lcd_conf, INTRO0, 0, 1);
	hd44780fw_write(&lcd_conf, INTRO0, 0x10, 0);
}

int main() {
	init_lcd();
	return 0;
}
