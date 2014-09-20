/**
 * Conversion project to implement the KSPIO plugin
 * Arduino code for Kerbal Space Program to native AVR C.
 *
 * KSPIO is released under Creative Commons BY License
 * by a developer I only know as 'Zitronen'. The development 
 * thread can be found on the KSP forums:
 * http://forum.kerbalspaceprogram.com/threads/66393
 */
 
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "uart/uart.h"
#include "kspio/kspio.h"

/* 38400 baud */
#define UART_BAUD_RATE      38400   
/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void init() {
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	kspio_init();
    sei();
}

int main(void) {
	init();
	
    while(1) {
		kspio_input();
		kspio_output();
	}
    	
	return 0;
}