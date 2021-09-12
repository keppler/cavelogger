#include "pins.h"
#include <avr/io.h>
#include <util/delay.h>
#include "LED.h"

#ifdef LED_PORT

void LED_init(void) {
	DDR(LED_PORT) |= (1 << PORTPIN(LED_PORT, LED_PIN));
	PORT(LED_PORT) &= ~(1 << PORTPIN(LED_PORT, LED_PIN));
}

void LED_blink(void) {
	LED_on();
	_delay_ms(100);
	LED_off();
	_delay_ms(100);
}

void LED_on(void) {
	PORT(LED_PORT) |= (1 << PORTPIN(LED_PORT, LED_PIN));  // LED on
}

void LED_off(void) {
	PORT(LED_PORT) &= ~(1 << PORTPIN(LED_PORT, LED_PIN)); // LED off
}

#else /* !LED_PORT */

#define LED_init()
#define LED_blink()
#define LED_on()
#define LED_off()

#endif /* LED_PORT */
