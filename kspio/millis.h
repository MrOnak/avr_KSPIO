/**
 * AVR C implementation of the millis() function
 * known from the Arduino GUI
 */

#ifndef AVR_MILLIS_H
#define AVR_MILLIS_H

extern void millisInit();
extern uint32_t millis();

#endif // AVR_MILLIS_H