#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
//#include "io.h"

// Permission to copy is granted provided that this header remains intact. 
// This software is provided with no warranties.

////////////////////////////////////////////////////////////////////////////////

#ifndef TIMER_H
#define TIMER_H

volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1ms
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B 	= 0x0B;	// bit3 = 1: CTC mode (clear timer on compare)
					// bit2bit1bit0=011: prescaler /64
					// 00001011: 0x0B
					// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
					// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A 	= 125;	// Timer interrupt will be generated when TCNT1==OCR1A
					// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
					// So when TCNT1 register equals 125,
					// 1 ms has passed. Thus, we compare to 125.
					// AVR timer interrupt mask register

	TIMSK1 	= 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1 = 0;

	// TimerISR will be called every _avr_timer_cntcurr milliseconds
	_avr_timer_cntcurr = _avr_timer_M;

	//Enable global interrupts
	SREG |= 0x80;	// 0x80: 1000000
}

void TimerOff() {
	TCCR1B 	= 0x00; // bit3bit2bit1bit0=0000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect)
{
	// CPU automatically calls when TCNT0 == OCR0 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; 			// Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { 	// results in a more efficient compare
		TimerISR(); 				// Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

#endif //TIMER_H

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// Nokia5110.h start

#include <avr/pgmspace.h>
#include <stdint.h>

/*
 * LCD's port
 */
#define PORT_LCD PORTA
#define DDR_LCD DDRA

/*
 * LCD's pins
 */
#define LCD_SCE PA0
#define LCD_RST PA1
#define LCD_DC PA2
#define LCD_DIN PA3
#define LCD_CLK PA4

#define LCD_CONTRAST 0x40

/*
 * Must be called once before any other function, initializes display
 */
void nokia_lcd_init(void);

/*
 * Clear screen
 */
void nokia_lcd_clear(void);

/**
 * Power of display
 * @lcd: lcd nokia struct
 * @on: 1 - on; 0 - off;
 */
void nokia_lcd_power(uint8_t on);

/**
 * Set single pixel
 * @x: horizontal pozition
 * @y: vertical position
 * @value: show/hide pixel
 */
void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value);

/**
 * Draw single char with 1-6 scale
 * @code: char code
 * @scale: size of char
 */
void nokia_lcd_write_char(char code, uint8_t scale);

/**
 * Draw string. Example: writeString("abc",3);
 * @str: sending string
 * @scale: size of text
 */
void nokia_lcd_write_string(const char *str, uint8_t scale);

/**
 * Set cursor position
 * @x: horizontal position
 * @y: vertical position
 */
void nokia_lcd_set_cursor(uint8_t x, uint8_t y);

/*
 * Render screen to display
 */
void nokia_lcd_render(void);


// Nokia5110.h end
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// Nokia5110_chars.h start

//Nokia5110_chars.h start

#include <avr/pgmspace.h>

const uint8_t CHARSET[][5] PROGMEM = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00 }, // 20 space
	{ 0x00, 0x00, 0x5f, 0x00, 0x00 }, // 21 !
	{ 0x00, 0x07, 0x00, 0x07, 0x00 }, // 22 "
	{ 0x14, 0x7f, 0x14, 0x7f, 0x14 }, // 23 #
	{ 0x24, 0x2a, 0x7f, 0x2a, 0x12 }, // 24 $
	{ 0x23, 0x13, 0x08, 0x64, 0x62 }, // 25 %
	{ 0x36, 0x49, 0x55, 0x22, 0x50 }, // 26 &
	{ 0x00, 0x05, 0x03, 0x00, 0x00 }, // 27 '
	{ 0x00, 0x1c, 0x22, 0x41, 0x00 }, // 28 (
	{ 0x00, 0x41, 0x22, 0x1c, 0x00 }, // 29 )
	{ 0x14, 0x08, 0x3e, 0x08, 0x14 }, // 2a *
	{ 0x08, 0x08, 0x3e, 0x08, 0x08 }, // 2b +
	{ 0x00, 0x50, 0x30, 0x00, 0x00 }, // 2c ,
	{ 0x08, 0x08, 0x08, 0x08, 0x08 }, // 2d -
	{ 0x00, 0x60, 0x60, 0x00, 0x00 }, // 2e .
	{ 0x20, 0x10, 0x08, 0x04, 0x02 }, // 2f /
	{ 0x3e, 0x51, 0x49, 0x45, 0x3e }, // 30 0
	{ 0x00, 0x42, 0x7f, 0x40, 0x00 }, // 31 1
	{ 0x42, 0x61, 0x51, 0x49, 0x46 }, // 32 2
	{ 0x21, 0x41, 0x45, 0x4b, 0x31 }, // 33 3
	{ 0x18, 0x14, 0x12, 0x7f, 0x10 }, // 34 4
	{ 0x27, 0x45, 0x45, 0x45, 0x39 }, // 35 5
	{ 0x3c, 0x4a, 0x49, 0x49, 0x30 }, // 36 6
	{ 0x01, 0x71, 0x09, 0x05, 0x03 }, // 37 7
	{ 0x36, 0x49, 0x49, 0x49, 0x36 }, // 38 8
	{ 0x06, 0x49, 0x49, 0x29, 0x1e }, // 39 9
	{ 0x00, 0x36, 0x36, 0x00, 0x00 }, // 3a :
	{ 0x00, 0x56, 0x36, 0x00, 0x00 }, // 3b ;
	{ 0x08, 0x14, 0x22, 0x41, 0x00 }, // 3c <
	{ 0x14, 0x14, 0x14, 0x14, 0x14 }, // 3d =
	{ 0x00, 0x41, 0x22, 0x14, 0x08 }, // 3e >
	{ 0x02, 0x01, 0x51, 0x09, 0x06 }, // 3f ?
	{ 0x32, 0x49, 0x79, 0x41, 0x3e }, // 40 @
	{ 0x7e, 0x11, 0x11, 0x11, 0x7e }, // 41 A
	{ 0x7f, 0x49, 0x49, 0x49, 0x36 }, // 42 B
	{ 0x3e, 0x41, 0x41, 0x41, 0x22 }, // 43 C
	{ 0x7f, 0x41, 0x41, 0x22, 0x1c }, // 44 D
	{ 0x7f, 0x49, 0x49, 0x49, 0x41 }, // 45 E
	{ 0x7f, 0x09, 0x09, 0x09, 0x01 }, // 46 F
	{ 0x3e, 0x41, 0x49, 0x49, 0x7a }, // 47 G
	{ 0x7f, 0x08, 0x08, 0x08, 0x7f }, // 48 H
	{ 0x00, 0x41, 0x7f, 0x41, 0x00 }, // 49 I
	{ 0x20, 0x40, 0x41, 0x3f, 0x01 }, // 4a J
	{ 0x7f, 0x08, 0x14, 0x22, 0x41 }, // 4b K
	{ 0x7f, 0x40, 0x40, 0x40, 0x40 }, // 4c L
	{ 0x7f, 0x02, 0x0c, 0x02, 0x7f }, // 4d M
	{ 0x7f, 0x04, 0x08, 0x10, 0x7f }, // 4e N
	{ 0x3e, 0x41, 0x41, 0x41, 0x3e }, // 4f O
	{ 0x7f, 0x09, 0x09, 0x09, 0x06 }, // 50 P
	{ 0x3e, 0x41, 0x51, 0x21, 0x5e }, // 51 Q
	{ 0x7f, 0x09, 0x19, 0x29, 0x46 }, // 52 R
	{ 0x46, 0x49, 0x49, 0x49, 0x31 }, // 53 S
	{ 0x01, 0x01, 0x7f, 0x01, 0x01 }, // 54 T
	{ 0x3f, 0x40, 0x40, 0x40, 0x3f }, // 55 U
	{ 0x1f, 0x20, 0x40, 0x20, 0x1f }, // 56 V
	{ 0x3f, 0x40, 0x38, 0x40, 0x3f }, // 57 W
	{ 0x63, 0x14, 0x08, 0x14, 0x63 }, // 58 X
	{ 0x07, 0x08, 0x70, 0x08, 0x07 }, // 59 Y
	{ 0x61, 0x51, 0x49, 0x45, 0x43 }, // 5a Z
	{ 0x00, 0x7f, 0x41, 0x41, 0x00 }, // 5b [
	{ 0x02, 0x04, 0x08, 0x10, 0x20 }, // 5c backslash
	{ 0x00, 0x41, 0x41, 0x7f, 0x00 }, // 5d ]
	{ 0x04, 0x02, 0x01, 0x02, 0x04 }, // 5e ^
	{ 0x40, 0x40, 0x40, 0x40, 0x40 }, // 5f _
	{ 0x00, 0x01, 0x02, 0x04, 0x00 }, // 60 `
	{ 0x20, 0x54, 0x54, 0x54, 0x78 }, // 61 a
	{ 0x7f, 0x48, 0x44, 0x44, 0x38 }, // 62 b
	{ 0x38, 0x44, 0x44, 0x44, 0x20 }, // 63 c
	{ 0x38, 0x44, 0x44, 0x48, 0x7f }, // 64 d
	{ 0x38, 0x54, 0x54, 0x54, 0x18 }, // 65 e
	{ 0x08, 0x7e, 0x09, 0x01, 0x02 }, // 66 f
	{ 0x0c, 0x52, 0x52, 0x52, 0x3e }, // 67 g
	{ 0x7f, 0x08, 0x04, 0x04, 0x78 }, // 68 h
	{ 0x00, 0x44, 0x7d, 0x40, 0x00 }, // 69 i
	{ 0x20, 0x40, 0x44, 0x3d, 0x00 }, // 6a j
	{ 0x7f, 0x10, 0x28, 0x44, 0x00 }, // 6b k
	{ 0x00, 0x41, 0x7f, 0x40, 0x00 }, // 6c l
	{ 0x7c, 0x04, 0x18, 0x04, 0x78 }, // 6d m
	{ 0x7c, 0x08, 0x04, 0x04, 0x78 }, // 6e n
	{ 0x38, 0x44, 0x44, 0x44, 0x38 }, // 6f o
	{ 0x7c, 0x14, 0x14, 0x14, 0x08 }, // 70 p
	{ 0x08, 0x14, 0x14, 0x18, 0x7c }, // 71 q
	{ 0x7c, 0x08, 0x04, 0x04, 0x08 }, // 72 r
	{ 0x48, 0x54, 0x54, 0x54, 0x20 }, // 73 s
	{ 0x04, 0x3f, 0x44, 0x40, 0x20 }, // 74 t
	{ 0x3c, 0x40, 0x40, 0x20, 0x7c }, // 75 u
	{ 0x1c, 0x20, 0x40, 0x20, 0x1c }, // 76 v
	{ 0x3c, 0x40, 0x30, 0x40, 0x3c }, // 77 w
	{ 0x44, 0x28, 0x10, 0x28, 0x44 }, // 78 x
	{ 0x0c, 0x50, 0x50, 0x50, 0x3c }, // 79 y
	{ 0x44, 0x64, 0x54, 0x4c, 0x44 }, // 7a z
	{ 0x00, 0x08, 0x36, 0x41, 0x00 }, // 7b {
	{ 0x00, 0x00, 0x7f, 0x00, 0x00 }, // 7c |
	{ 0x00, 0x41, 0x36, 0x08, 0x00 }, // 7d }
	{ 0x10, 0x08, 0x08, 0x10, 0x08 }, // 7e ~
	{ 0x00, 0x00, 0x00, 0x00, 0x00 } // 7f
};
//Nokia5110_chars.h end
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// Nokia5110.c start

//#include "nokia5110.h"

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
//#include "nokia5110_chars.h"


static struct {
    /* screen byte massive */
    uint8_t screen[504];

    /* cursor position */
    uint8_t cursor_x;
    uint8_t cursor_y;

} nokia_lcd = {
    .cursor_x = 0,
    .cursor_y = 0
};

/**
 * Sending data to LCD
 * @bytes: data
 * @is_data: transfer mode: 1 - data; 0 - command;
 */
static void write(uint8_t bytes, uint8_t is_data)
{
	register uint8_t i;
	/* Enable controller */
	PORT_LCD &= ~(1 << LCD_SCE);

	/* We are sending data */
	if (is_data)
		PORT_LCD |= (1 << LCD_DC);
	/* We are sending commands */
	else
		PORT_LCD &= ~(1 << LCD_DC);

	/* Send bytes */
	for (i = 0; i < 8; i++) {
		/* Set data pin to byte state */
		if ((bytes >> (7-i)) & 0x01)
			PORT_LCD |= (1 << LCD_DIN);
		else
			PORT_LCD &= ~(1 << LCD_DIN);

		/* Blink clock */
		PORT_LCD |= (1 << LCD_CLK);
		PORT_LCD &= ~(1 << LCD_CLK);
	}

	/* Disable controller */
	PORT_LCD |= (1 << LCD_SCE);
}

static void write_cmd(uint8_t cmd)
{
	write(cmd, 0);
}

static void write_data(uint8_t data)
{
	write(data, 1);
}

/*
 * Public functions
 */

void nokia_lcd_init(void)
{
	register unsigned i;
	/* Set pins as output */
	DDR_LCD |= (1 << LCD_SCE);
	DDR_LCD |= (1 << LCD_RST);
	DDR_LCD |= (1 << LCD_DC);
	DDR_LCD |= (1 << LCD_DIN);
	DDR_LCD |= (1 << LCD_CLK);

	/* Reset display */
	PORT_LCD |= (1 << LCD_RST);
	PORT_LCD |= (1 << LCD_SCE);
	_delay_ms(10);
	PORT_LCD &= ~(1 << LCD_RST);
	_delay_ms(70);
	PORT_LCD |= (1 << LCD_RST);

	/*
	 * Initialize display
	 */
	/* Enable controller */
	PORT_LCD &= ~(1 << LCD_SCE);
	/* -LCD Extended Commands mode- */
	write_cmd(0x21);
	/* LCD bias mode 1:48 */
	write_cmd(0x13);
	/* Set temperature coefficient */
	write_cmd(0x06);
	/* Default VOP (3.06 + 66 * 0.06 = 7V) */
	write_cmd(0xC2);
	/* Standard Commands mode, powered down */
	write_cmd(0x20);
	/* LCD in normal mode */
	write_cmd(0x09);

	/* Clear LCD RAM */
	write_cmd(0x80);
	write_cmd(LCD_CONTRAST);
	for (i = 0; i < 504; i++)
		write_data(0x00);

	/* Activate LCD */
	write_cmd(0x08);
	write_cmd(0x0C);
}

void nokia_lcd_clear(void)
{
	register unsigned i;
	/* Set column and row to 0 */
	write_cmd(0x80);
	write_cmd(0x40);
	/*Cursor too */
	nokia_lcd.cursor_x = 0;
	nokia_lcd.cursor_y = 0;
	/* Clear everything (504 bytes = 84cols * 48 rows / 8 bits) */
	for(i = 0;i < 504; i++)
		nokia_lcd.screen[i] = 0x00;
}

void nokia_lcd_power(uint8_t on)
{
	write_cmd(on ? 0x20 : 0x24);
}

void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value)
{
	uint8_t *byte = &nokia_lcd.screen[y/8*84+x];
	if (value)
		*byte |= (1 << (y % 8));
	else
		*byte &= ~(1 << (y %8 ));
}

void nokia_lcd_write_char(char code, uint8_t scale)
{
	register uint8_t x, y;

	for (x = 0; x < 5*scale; x++)
		for (y = 0; y < 7*scale; y++)
			if (pgm_read_byte(&CHARSET[code-32][x/scale]) & (1 << y/scale))
				nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 1);
			else
				nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 0);

	nokia_lcd.cursor_x += 5*scale + 1;
	if (nokia_lcd.cursor_x >= 84) {
		nokia_lcd.cursor_x = 0;
		nokia_lcd.cursor_y += 7*scale + 1;
	}
	if (nokia_lcd.cursor_y >= 48) {
		nokia_lcd.cursor_x = 0;
		nokia_lcd.cursor_y = 0;
	}
}

void nokia_lcd_write_string(const char *str, uint8_t scale)
{
	while(*str)
		nokia_lcd_write_char(*str++, scale);
}

void nokia_lcd_set_cursor(uint8_t x, uint8_t y)
{
	nokia_lcd.cursor_x = x;
	nokia_lcd.cursor_y = y;
}

void nokia_lcd_render(void)
{
	register unsigned i;
	/* Set column and row to 0 */
	write_cmd(0x80);
	write_cmd(0x40);

	/* Write screen to display */
	for (i = 0; i < 504; i++)
		write_data(nokia_lcd.screen[i]);
}

//Nokia5110.c end
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
//LCD File start

void LCD_init();
void LCD_ClearScreen(void);
void LCD_WriteCommand (unsigned char Command);
void LCD_Cursor (unsigned char column);
void LCD_DisplayString(unsigned char column ,const unsigned char *string);
void delay_ms(int miliSec);

#define SET_BIT(p,i) ((p) |= (1 << (i)))
#define CLR_BIT(p,i) ((p) &= ~(1 << (i)))
#define GET_BIT(p,i) ((p) & (1 << (i)))

/*-------------------------------------------------------------------------*/

#define DATA_BUS PORTC		// port connected to pins 7-14 of LCD display
#define CONTROL_BUS PORTD	// port connected to pins 4 and 6 of LCD disp.
#define RS 6			// pin number of uC connected to pin 4 of LCD disp.
#define E 7			// pin number of uC connected to pin 6 of LCD disp.

/*-------------------------------------------------------------------------*/

void LCD_ClearScreen(void) {
	LCD_WriteCommand(0x01);
}

void LCD_init(void) {

	//wait for 100 ms.
	delay_ms(100);
	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x06);
	LCD_WriteCommand(0x0f);
	LCD_WriteCommand(0x01);
	delay_ms(10);
}

void LCD_WriteCommand (unsigned char Command) {
	CLR_BIT(CONTROL_BUS,RS);
	DATA_BUS = Command;
	SET_BIT(CONTROL_BUS,E);
	asm("nop");
	CLR_BIT(CONTROL_BUS,E);
	delay_ms(2); // ClearScreen requires 1.52ms to execute
}

void LCD_WriteData(unsigned char Data) {
	SET_BIT(CONTROL_BUS,RS);
	DATA_BUS = Data;
	SET_BIT(CONTROL_BUS,E);
	asm("nop");
	CLR_BIT(CONTROL_BUS,E);
	delay_ms(1);
}

void LCD_DisplayString( unsigned char column, const unsigned char* string) {
	//LCD_ClearScreen();
	unsigned char c = column;
	while(*string) {
		LCD_Cursor(c++);
		LCD_WriteData(*string++);
	}
}

void LCD_Cursor(unsigned char column) {
	if ( column < 17 ) { // 16x1 LCD: column < 9
		// 16x2 LCD: column < 17
		LCD_WriteCommand(0x80 + column - 1);
		} else {
		LCD_WriteCommand(0xB8 + column - 9);	// 16x1 LCD: column - 1
		// 16x2 LCD: column - 9
	}
}

void delay_ms(int miliSec) //for 8 Mhz crystal

{
	int i,j;
	for(i=0;i<miliSec;i++)
	for(j=0;j<775;j++)
	{
		asm("nop");
	}
}

void LCD_Custom_Char (unsigned char loc, unsigned char *msg){
	int i;
	LCD_WriteCommand (0x40 + (loc*8));	/* Command 0x40 for CGRAM */
	for(i = 0;i < 8; i++)	/* 8 cause 8 lines x 5 rows per character */
	LCD_WriteData(msg[i]);
	LCD_WriteCommand(0x80);
}

// LCD file end
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
// Main file start

unsigned char smiley[8] = {0b00000,0b01010,0b01010,0b00000,
	0b10000,0b10001,0b11111,0b00000
};
unsigned char frowny[8] = {0b10001,0b01010,0b00000,0b10001,
	0b00000,0b01110,0b10001,0b10001
};
unsigned char heart[8] = {0b00000,0b00000,0b01010,0b10101,
  0b10001,0b01010,0b00100,0b00000
};

unsigned char choose[13] = {'M','a','k','e',' ','a',' ','c','h','o','i','c','e'};
unsigned char title[9] = {'4',' ','C','o','r','n','e','r','s'};
unsigned char youWin[8] = {'Y','o','u',' ','W','i','n','!'};
unsigned char youLose[7] = {'W','r','o','n','g','!'};
unsigned char reset[14] = {'P','r','e','s','s',' ','t','o',' ','r','e','s','e','t'};
unsigned char selection = 0x00;
unsigned char ran_num = 0x00;

unsigned char button1 = 0x00;
unsigned char button2 = 0x00;
unsigned char button3 = 0x00;
unsigned char button4 = 0x00;
unsigned char resetButton = 0x00;

char ranArr[4]= {'0','0','0','0'};

enum SM_States{SM_Start, SM_On, SM_Reset} state;

void SM_Tick(int state) {
	
	button1 = ~PINB & 0x01;
	button2 = ~PINB & 0x02;
	button3 = ~PINB & 0x04;
	button4 = ~PINB & 0x08;
	resetButton = ~PINB & 0x10;
	
	switch(state) { //transitions
		case SM_Start:
			//itoa(ran_num,ranArr,10); //debug random number
			//LCD_Cursor(15);
			//LCD_WriteData(ranArr[0]);
			state = SM_On;
			break;
		case SM_On:
			if(button1){
				state = SM_Reset;
			}
			else if(button2){
				state = SM_Reset;
				
			}
			else if(button3){
				state = SM_Reset;
			}
			else if(button4){
				state = SM_Reset;
			}
			else{
				state = SM_Reset;
			}
			break;
		case SM_Reset:
			if(resetButton){
				state = SM_On;
			}
			else{
				state = SM_Reset;
			}
			break;
		default: break;
	}
	switch(state) { //actions
		
		button1 = ~PINB & 0x01;
		button2 = ~PINB & 0x02;
		button3 = ~PINB & 0x04;
		button4 = ~PINB & 0x08;
		resetButton = ~PINB & 0x10;
		
		case SM_Start:
			break;
		case SM_On:
			
			selection = 0;
			if((button1 && ran_num == 0) //guess correctly
				||(button2 && ran_num == 1)
				||(button3 && ran_num == 2)
				||(button4 && ran_num ==3)){

				LCD_ClearScreen();
				LCD_Cursor(1);
				for(int i = 0; i < 8; ++i){
					LCD_WriteData(youWin[i]);
				}
				LCD_WriteData(0); //display heart
				LCD_Cursor(0); //hide cursor
			}
			else if((button1 && ran_num != 0) //guess wrong
				||(button2 && ran_num != 1)
				||(button3 && ran_num != 2)
				||(button4 && ran_num !=3)){

				LCD_ClearScreen();
				LCD_Cursor(1);
				for(int i = 0; i < 6; ++i){
					LCD_WriteData(youLose[i]);
				}
				LCD_WriteData(4); //display frowny
				LCD_Cursor(0); //hide cursor
			}
			else if(resetButton){ //reset random number
				ran_num = rand() % 4;
			}
			else{
				//display LCD Menu
				LCD_Cursor(1);
				
				for(int i = 0; i < 9; ++i){
					LCD_WriteData(title[i]);
				}
				LCD_Cursor(17); //move cursor to 2nd row
				for(int i = 0; i < 13; i++){
					LCD_WriteData(choose[i]);
				}
				LCD_Cursor(0); //hide cursor
				//display corner numbers
				
				nokia_lcd_set_cursor(0,0);
				nokia_lcd_write_string("1", 2);
				nokia_lcd_set_cursor(65,0);
				nokia_lcd_write_string("2", 2);
				nokia_lcd_set_cursor(0,30);
				nokia_lcd_write_string("3", 2);
				nokia_lcd_set_cursor(65,30);
				nokia_lcd_write_string("4", 2);
				//display horizontal line
				nokia_lcd_set_cursor(0,19);
				nokia_lcd_write_string("--------------",1);
				//display vertical line
				for(int i = 0; i < 45; ++i){
					nokia_lcd_set_cursor(35,i);
					nokia_lcd_write_string("|",1);
				}
				if(button1){
					//nokia_lcd_set_cursor(0,0);
					//nokia_lcd_write_string("5", 2);
					//nokia_lcd_clear();
				}
				nokia_lcd_render();
			}
			break;
		case SM_Reset:
			LCD_Cursor(17);
			for(int i = 0; i < 14; ++i){
				LCD_WriteData(reset[i]);
			}
			LCD_Cursor(0);
			break;
		default: break;
	}
}


int main(){
	DDRA = 0xFF; PORTA = 0x00;
	DDRB = 0x00; PORTB = 0xFF;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	
	//TimerSet(100);
	//TimerOn();

	LCD_init();
	LCD_ClearScreen();
	LCD_Custom_Char(6, heart); //build smiley at position 1 in CG RAM
	LCD_Custom_Char(4, frowny); // build frowny at position 0 in CG RAM
	nokia_lcd_init();
	nokia_lcd_clear();
	nokia_lcd_power(1);

	srand(time(0));
	ran_num = rand() % 4;
	state = SM_Start;
	
	while(1){
		
		SM_Tick(state);

		//while(!TimerFlag)
		//TimerFlag = 0;
	}
}

