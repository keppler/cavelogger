/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * menu.c
 * Configuration/Control menu
 */

#include "menu.h"
#include "main.h"
#include "pins.h"
#include <util/delay.h>
#include <stddef.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include "SSD1306.h"
#include "../lib/arduino-lmic/lmic/lmic.h"
#include "RX6110.h"

static const char *sf2pstr(uint8_t sf) {
	switch(sf) {
		case DR_SF7: return(PSTR("SF7 "));
		case DR_SF8: return(PSTR("SF8 "));
		case DR_SF9: return(PSTR("SF9 "));
		case DR_SF10: return(PSTR("SF10"));
		case DR_SF11: return(PSTR("SF11"));
		case DR_SF12: return(PSTR("SF12"));
		default: return(PSTR("????"));
	}
}

/*
typedef struct {
	uint8_t titlePos;
	const char *title;
	uint8_t valuePos;
	const char *value;
} menu_st;

menu_st menuData[5];

static void menu_init() {
	menuData[0] = (menu_st){ 0 , PSTR("LORA:"), 6, cfgLoraOff ? PSTR("<OFF>") : PSTR("<ON> ")};
	menuData[1] = (menu_st){ 0, NULL, 6 , sf2pstr(cfgLoraSF) };
	menuData[2] = (menu_st){ 0, NULL, 0, PSTR("<STATUS>") };
	menuData[3] = (menu_st){ 0, NULL, 0, PSTR("<ERASE FLASH>") };
	menuData[4] = (menu_st){ 0, NULL, 0, PSTR("<EXIT>") };
}

static void showMenu(const menu_st *mnu, uint8_t line, uint8_t count, uint8_t pos) {
	uint8_t idx;
	for (idx=0; idx < count; idx++, line++) {
		if (mnu[idx].title != NULL) {
			SSD1306_writeString(mnu[idx].titlePos, line, mnu[idx].title, 1);
		}
		if (mnu[idx].value != NULL) {
			SSD1306_writeString(mnu[idx].valuePos, line, mnu[idx].value, pos == idx ? 3 : 1);
		}
	}
}
*/

void menu_erase_flash() {
	uint8_t line = 0;
	uint8_t _state = 0;
	SSD1306_clear();
	SSD1306_writeString(0, line, PSTR("==ERASE FLASH=="), 1);

UPDATE_MENU:
	line=2;
	SSD1306_writeString(0, line, PSTR("<ABORT>"), _state == 0 ? 3 : 1);
	line++;
	SSD1306_writeString(0, line, PSTR("<ERASE FLASH>"), _state == 1 ? 3 : 1);


	uint8_t delayCount = 0;
	inputButton = 0;
	do {
		_delay_ms(100);

		if (inputButton & 0x01) {
			_state++;
			if (_state == 2) _state = 0;
			goto UPDATE_MENU;
		}

		if (inputButton & 0x02) {
			switch (_state) {
				case 0:
					/* abort */
					return;
				case 1:
					/* ERASE FLASH */
					SSD1306_writeString(0, 5, PSTR("ERASING FLASH..."), 1);
					_delay_ms(1000);
//					FLASH_erase();
					_delay_ms(1000);
					return;
				default:
					break;
					// ignore...
			}
		}

	} while (delayCount++ < 100);

}

void menu_status() {
	uint8_t line;
	uint8_t _state = 0;
	SSD1306_clear();
	SSD1306_writeString(0, 0, PSTR("==STATUS=="), 1);

	line=2;
	SSD1306_writeString(0, line, PSTR("TEMP:"), 1);
	SSD1306_writeInt(7, line, measTemp, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("PRESS:"), 1);
	SSD1306_writeInt(7, line, (int32_t)measPress, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("HUMI:"), 1);
	SSD1306_writeInt(7, line, (int32_t)measHum, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("VCC:"), 1);
	SSD1306_writeInt(7, line, measVCC, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("WIND:"), 1);
	SSD1306_writeInt(7, line, measWind, 10);

	SSD1306_writeString(10, 7, PSTR("<NEXT>"), 3);

	uint8_t delayCount = 0;
	inputButton = 0;
	do {
		_delay_ms(100);

		if (inputButton & 0x02) {
			break;
		}

	} while (delayCount++ < 100);

	SSD1306_clear();
	SSD1306_writeString(0, 0, PSTR("==STATUS=="), 1);

	line=2;
	struct RTC_ts ts;
	RTC_get(&ts);
	SSD1306_writeString(0, line, PSTR("  .  .    "), 1);
	SSD1306_writeInt(0, line, ts.ts_mday, 10);
	SSD1306_writeInt(3, line, ts.ts_mon + 1, 10);
	SSD1306_writeInt(6, line, ts.ts_year + 1900, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("  :  :  "), 1);
	SSD1306_writeInt(0, line, ts.ts_hour, 10);
	SSD1306_writeInt(3, line, ts.ts_min, 10);
	SSD1306_writeInt(6, line, ts.ts_sec, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("TX FRAMES:"), 1);
	SSD1306_writeInt(11, line, LMIC.seqnoUp, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("RTC INTS:"), 1);
	SSD1306_writeInt(11, line, int0Count, 10);

	line++;
	SSD1306_writeString(0, line, PSTR("<RETURN>"), 3);

	delayCount = 0;
	inputButton = 0;
	do {
		_delay_ms(100);

		if (inputButton & 0x02) {
			return;
		}

	} while (delayCount++ < 100);

}

/* -------------------------------------------------------------------------
 * show menu
 * ---------------------------------------------------------------------- */
void menu_main() {
	uint8_t line;
	uint8_t _state = 0;
SHOW_MENU:
	line = 0;
	SSD1306_clear();
	SSD1306_writeString(0, line, PSTR("==MENU=="), 1);
//	menu_init();

UPDATE_MENU:
//	showMenu(menuData, 2, 5, 0);

	line=2;
	SSD1306_writeString(0, line, PSTR("LORA:"), 1);
	SSD1306_writeString(5, line, cfgLoraOff ? PSTR("<OFF>") : PSTR("<ON> "), _state == 0 ? 3 : 1);
	line++;
	SSD1306_writeString(5, line, sf2pstr(cfgLoraSF), _state == 1 ? 3 : 1);
	SSD1306_writeString(10, line, PSTR("<TEST>"), _state == 2 ? 3 : 1);
	line++;
	SSD1306_writeString(0, line, PSTR("<STATUS>"), _state == 3 ? 3 : 1);
	line++;
	SSD1306_writeString(0, line, PSTR("<ERASE FLASH>"), _state == 4 ? 3 : 1);
	line++;
	SSD1306_writeString(0, line, PSTR("<EXIT>"), _state == 5 ? 3 : 1);


	uint8_t delayCount = 0;
	inputButton = 0;
	do {
		_delay_ms(100);

		if (inputButton & 0x01) {
			_state++;
			if (cfgLoraOff && (_state == 1 || _state == 2)) _state=3;
			if (_state == 6) _state = 0;
			goto UPDATE_MENU;
		}

		if (inputButton & 0x02) {
			switch (_state) {
				case 0:
					cfgLoraOff ^= 0x01;
					goto UPDATE_MENU;
				case 1:
					/* cycle SF */
					switch (cfgLoraSF) {
						case DR_SF7: cfgLoraSF = DR_SF8; break;
						case DR_SF8: cfgLoraSF = DR_SF9; break;
						case DR_SF9: cfgLoraSF = DR_SF10; break;
						case DR_SF10: cfgLoraSF = DR_SF11; break;
						case DR_SF11: cfgLoraSF = DR_SF12; break;
						case DR_SF12: cfgLoraSF = DR_SF7; break;
						default: cfgLoraSF = DR_SF7;
					}
					/* Set data rate and transmit power for uplink */
					LMIC_setDrTxpow(cfgLoraSF, 14);
					goto UPDATE_MENU;
				case 2:
					// LoRa test!
					//menu_status();
					goto SHOW_MENU;
				case 3:
					// show status
					menu_status();
					goto SHOW_MENU;
				case 4:
					// erase flash menu
					menu_erase_flash();
					goto SHOW_MENU;
				case 5:
					// leave menu
					return;
				default:
					break;
					// ignore...
			}
		}

	} while (delayCount++ < 100);

	// return to main (sleep)
}
