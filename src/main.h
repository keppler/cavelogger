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
#include <time.h>

//#define ENABLE_RFM95
//#define ENABLE_BMP280

#define CL_VERSION "1.5"
#define CL_DATE "01/2021"

extern uint16_t int0Count;
extern volatile uint8_t inputButton;
extern uint8_t cfgLoraOff;
extern uint8_t cfgLoraSF;

/* recent measurements */
extern uint32_t measCount;
extern int32_t measTemp;
extern uint16_t measHum;
extern uint16_t measPress;
extern uint16_t measVCC;
extern uint16_t measWind;
extern time_t measTsFirst;
extern time_t measTsLast;

#endif /* !__MAIN_H */
