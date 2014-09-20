/**
 * AVR C implementation of the millis() function
 * from the Arduino libraries
 */
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint32_t timer0_overflow_count = 0;
volatile uint32_t timer0_millis = 0;
static uint8_t timer0_fract = 0;

#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)
 
 void millisInit() {
   // timer/counter control register 0 A: fast PWM
  TCCR0A |= (1 << WGM01) | (1 << WGM00);  
  // timer/counter control register 0 B: clock select: 64th CPU clock 
  TCCR0B |= (1<<CS01) | (1<<CS00); // 64th frequency
  // timer/counter interrupt mask register: timer/counter 0 overflow interrupt enable
  TIMSK0 |= (1<<TOIE0);
  
  //sei(); // enable interrupts - done globally
}

uint32_t millis() {
	uint32_t m;
	uint8_t oldSREG = SREG;

	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

SIGNAL(TIMER0_OVF_vect)
{
	uint32_t m = timer0_millis;
	uint8_t f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

