#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "Wind.h"

#include "LED.h"

static volatile uint16_t _Wind_pulses = 0;
static volatile uint8_t _Wind_done;
static volatile uint8_t _Wind_measure_init;

void Wind_init() {

	DDRD &= ~(1 << DDD3);	/* configure PD3 (INT1) as INPUT */

	/* disable internal pull-up (saving power) */
	PORTD &= ~(1 << PORTD3);

	/* D4 is OUTPUT ("power" for anemometer) */
	DDRD |= (1 << DDD4);	/* configure PD4 as OUTPUT */
	PORTD &= ~(1 << PD4);

	cli();

	/* EICRA (External Interrupt Control Register A): enable INT1 */
	/* Rising edge of INT1 will fire the ISR */
	EICRA |= ((1 << ISC11) | (1 << ISC10));

	sei();

}

uint16_t Wind_get() {
	cli();
	/* clear watchdog reset flag */
	MCUSR &= ~(1<<WDRF);
	/* allow access to the watchdog & prescaler configuration */
	WDTCSR = (1<<WDCE) | (1<<WDE);
	/* set prescaler to 8sec and enable watchdog interrupts */
	WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);

	/* reset watchdog timer */
	wdt_reset();

	_Wind_pulses = 0;
	_Wind_measure_init = 0;

	/* enable interrupt INT1 */
	EIMSK |= (1 << INT1);

	sei();

	power_spi_disable();
	power_twi_disable();

	/* enable internal pull-up */
	PORTD |= (1 << PORTD3);

	PORTD |= (1 << PD4);	/* power on */

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);	// ###TODO### Standby better? https://www.mikrocontroller.net/articles/Sleep_Mode
	sleep_enable();

	_Wind_done = 0;
	while (!_Wind_done) sleep_cpu();

	sleep_disable();

	/* disable watchdog timer */
	cli();
	/* disable interrupt INT1 */
	EIMSK &= ~(1 << INT1);

	MCUSR &= ~(1<<WDRF);
	WDTCSR = (1<<WDCE) | (1<<WDE);
	WDTCSR = 0x00;
	sei();

	power_spi_enable();
	power_twi_enable();

	/* disable internal pull-up (saving power) */
	PORTD &= ~(1 << PORTD3);
	PORTD &= ~(1 << PD4);

	return(_Wind_pulses);
}

ISR(INT1_vect) {
	if (_Wind_measure_init == 0) {
		_Wind_measure_init = 1;
		/* reset watchdog timer to ignore first anemometer event */
		wdt_reset();
	} else {
		_Wind_pulses++;
	}
}

ISR(WDT_vect) {
	// wakeup...
	_Wind_done = 1;
}
