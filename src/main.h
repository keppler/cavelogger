/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * main.h
 * Global variables
 */

#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>

extern uint16_t int0Count;
extern volatile uint8_t inputButton;
extern uint8_t cfgLoraOff;
extern uint8_t cfgLoraSF;

/* recent measurements */
extern int32_t measTemp;
extern uint16_t measHum;
extern uint16_t measPress;
extern uint16_t measVCC;
extern uint16_t measWind;

#endif /* !__MAIN_H */
