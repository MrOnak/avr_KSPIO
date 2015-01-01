/**
 * Port of the KSPIO code (c) by 'zitronen' to AVR C
 * http://forum.kerbalspaceprogram.com/threads/66393
 *
 * Zitronen's code is released under Creative Commons Attribution (BY).
 * The same is true for my port.
 *
 * Zitronen's code was based around the Arduino library and 
 * mostly intended as a demonstration, to get people started.
 *
 * I'm porting this to AVR's variant of C to open it up to Atmel 
 * microprocessors without the need for the Arduino GUI. 
 *
 * This port is based off of the "KSPIODemo8" and expects to 
 * be communicating with v0.15.3 of the KSPIO plugin.
 *
 * @author Dominique Stender <dst@st-webdevelopment.com>
 */
 
#ifndef KSPIO_H
#define KSPIO_H

extern void kspio_init();
extern int8_t kspio_input();
extern void kspio_output();

void kspio_initTXPackets();
void kspio_controlsInit();
void kspio_controls();
void kspio_mainControls(uint8_t n, uint8_t s);
void kspio_controlGroups(uint8_t n, uint8_t s);
void kspio_handshake();
uint8_t kspio_boardReceiveData();
void kspio_boardSendData(uint8_t * address, uint8_t len);


void kspio_indicators();
void kspio_LEDSAllOff();
void kspio_initLEDS();

#endif // KSPIO_H
