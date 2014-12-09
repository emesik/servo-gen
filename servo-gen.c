#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PWM_MASK (1 << PB1)
#define MUX_MASK ((1 << MUX1) | (1 << MUX0))

// Three phases of servo signal:
#define PH_INIT 0	// initial part of the pulse, 1ms in standard mode
#define PH_WORK 1	// the data carrying part of the pulse, 1ms
#define PH_IDLE 2	// the idle time, 18ms

/* The standard duration of the work phase is 100-200 ticks (1.0-2.0 ms)
 * where the proportion of high and low state indicates the desired angle
 * of mechanical rotation.
 *
 * We expand it a bit to cover wider angle of movement allowed by most modern
 * servos. These particular values have been tested with Turnigy TG9e
 *
 * When adapting to your needs, just make sure that the sum is equal to 2000,
 * although longer idle periods should do no harm.
 */
#define TICKS_INIT 90
#define TICKS_WORK 168
#define TICKS_IDLE 1742

// Dead margin of the potentiometer. Instead of rescaling integers, just cut
// off both margins.
#define MARGIN (256 - TICKS_WORK) / 2

uint8_t signal_length = 0x80;

ISR(TIMER0_COMPA_vect) {
	static uint8_t phase = PH_IDLE;
	static uint16_t ticks = 0;

	ticks++;
	switch (phase) {
		case PH_INIT:
			if (ticks >= TICKS_INIT) {
				phase = PH_WORK;
				ticks = 0;
			}
			break;
		case PH_WORK:
			if ((ticks >= signal_length) | (ticks >= TICKS_WORK)) {
				// if signal length is finished, turn PWM off
				PORTB &= ~PWM_MASK;
			}
			if (ticks >= TICKS_WORK) {
				phase = PH_IDLE;
				ticks = 0;
				// start next conversion
				ADCSRA |= (1 << ADSC);
			}
		case PH_IDLE:
			if (ticks >= TICKS_IDLE) {
				// turn PWM on and go to init phase
				PORTB |= PWM_MASK;
				phase = PH_INIT;
				ticks = 0;
			}
			break;
	}
}

ISR(ADC_vect) {
	uint8_t val = ADCH;
	if (val < MARGIN) val = 0;
	else val -= MARGIN;
	signal_length = val;
}

int main() {
	cli();
	// init ADC: Vcc as reference, 8bit, 128 prescaler to save cycles
	ADMUX = (1 << ADLAR) | MUX_MASK;
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// CTC, no prescaler
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS00);
	OCR0A = 39;
	// enable interrupt
	TIMSK |= (1 << OCIE0A);

	PORTB &= ~PWM_MASK;
	DDRB |= PWM_MASK;

	sei();

	// start first conversion
	ADCSRA |= (1 << ADSC);

	while (1) {}
}
