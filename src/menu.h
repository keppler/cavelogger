/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * menu.h
 * Configuration/Control menu
 */

#ifndef __MENU_H
#define __MENU_H

#include <avr/eeprom.h>

extern const unsigned char Logo[] EEMEM;
extern const uint8_t NWKSKEY[] EEMEM;
extern const uint8_t APPSKEY[] EEMEM;
extern const uint32_t DEVADDR EEMEM;

void menu_main();

#endif /* !__MENU_H */
