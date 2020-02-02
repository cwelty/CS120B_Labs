  /*	Author: Carson Welty (cwelt001)
 *  Partner(s) Name: Christian Lee (clee244)
 *	Lab Section: 024
 *	Assignment: Lab #6  Exercise #1
 *	Exercise Description: [optional - include for your own benefit]
 *	
 *	I acknowledge all content contained herein, excluding template or example
 *	code, is my own original work.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

unsigned char tmpA = 0x00;
unsigned char tmpC = 0x00;

enum States{init, light1, wait1, light2, wait2, light3, wait3}state;
 
void Tick(){
	switch (state) {
		case init:
			state = light1;
			break;
			case light1:
			if(tmpA) {
				state = wait1;
			}
			else {
				state = light2;
			}
			break;
		 case wait1:
			if(tmpA) {
				state = light1;
			}
			else{
				state = wait1;
			}
			break;
		 case light2:
			if(tmpA) {
				state = wait2;
			}
			else {
				state = light3;
			}
		 break;
		 case wait2:
			if(tmpA) {
				state = light2;
			}
			else {
				state = wait2;
			}
			break;
		 case light3:
			if(tmpA){
				state = wait3;
			}
			else{
				state = light1;
			}
			break;
		 case wait3:
			if(tmpA) {
				state = light3;
			}
			else {
				state = wait3;
			}
			break;
		}
		
	switch (state){
		case init:
		break;
		case light1:
		tmpC = 0x01;
		break;
		case wait1:
		tmpC = 0x01;
		break;
		case light2:
		tmpC = 0x02;
		break;
		case wait2:
		tmpC = 0x02;
		break;
		case light3:
		tmpC = 0x04;
		break;
		case wait3:
		tmpC = 0x04;
		break;
	}
}
 void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B = 0x0B;// bit3 = 0: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A = 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register
	TIMSK1 = 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1=0;

	_avr_timer_cntcurr = _avr_timer_M;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds

	//Enable global interrupts
	SREG |= 0x80; // 0x80: 1000000
}

void TimerOff() {
	TCCR1B = 0x00; // bit3bit1bit0=000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; // Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { // results in a more efficient compare
		TimerISR(); // Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}
  
void main() {
	DDRA = 0x00; PORTA = 0xFF; // Initiate DDRA to inputs
	DDRC = 0xFF; PORTC = 0x00; // Initiate DDRC to outputs
	TimerSet(300);
	TimerOn();
	state = init;
	while(1) {
		tmpA = ~PINA & 0x02;
		// User code (i.e. synchSM calls)
		Tick();
		while (!TimerFlag);	// Wait 1 sec
		TimerFlag = 0;
		// Note: For the above a better style would use a synchSM with TickSM()
		// This example just illustrates the use of the ISR and flag
		PORTC = tmpC;  
	}
	return 0;
}
