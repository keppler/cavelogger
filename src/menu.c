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
#include "RX6110.h"

#ifdef FLASH_NSS_PORT
#include "AT45DB081E.h"
#endif /* FLASH_NSS_PORT */

#ifdef ENABLE_RFM95
#include "../lib/arduino-lmic/lmic/lmic.h"
#endif /* ENABLE_RFM95 */

#include "../lib/petitfat/pff.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>

// https://javl.github.io/image2cpp/
// invert image colors; vertical, 1 bit per pixel
const unsigned char Logo[] EEMEM = {
// 'logo-cavelogger-64x64', 64x64px
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 
0xe0, 0xf0, 0xf8, 0xf8, 0xfc, 0xf8, 0xfc, 0xfe, 0xfc, 0xfe, 0xfe, 0xff, 0xfe, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0x7f, 0x7e, 0x7e, 0x7c, 0x7c, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 
0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 
0x7f, 0x3f, 0x0f, 0x0f, 0x07, 0x1f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 
0x0f, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x1f, 0x3f, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0xf8, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x01, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0f, 0x1f, 0x1f, 0x07, 0x01, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
0xe0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xe0, 0xf0, 0xfc, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x1f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x87, 0x8f, 0x1f, 0xbf, 
0xbf, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe0, 0xe0, 0xc0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xbf, 0xbf, 
0x1f, 0x8f, 0x87, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x80, 0x80, 0x80, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 
0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0xbf, 0x3f, 0x3f, 0x2d, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x0c, 0x0f, 0x1f, 0x1f, 0x1f, 0x1f, 0x3f, 0x1f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 
0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0x20, 0x20, 0x20, 0x11, 0x00, 0x32, 0x2a, 0x2a, 0x3e, 0x00, 0x02, 0x1c, 0x20, 0x1c, 
0x02, 0x00, 0x1c, 0x2a, 0x2a, 0x2c, 0x00, 0x00, 0x3f, 0x20, 0x20, 0x20, 0x1c, 0x22, 0x22, 0x1c, 
0x00, 0x5c, 0xa2, 0xa2, 0x7e, 0x00, 0x5c, 0xa2, 0xa2, 0x7e, 0x00, 0x1c, 0x2a, 0x2a, 0x2c, 0x00, 
0x3e, 0x02, 0x02, 0x00, 0x20, 0x00, 0x1c, 0x22, 0x22, 0x3f, 0x00, 0x1c, 0x2a, 0x2a, 0x2c, 0x00
};

#ifdef ENABLE_RFM95
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
#endif /* ENABLE_RFM95 */

/* -------------------------------------------------------------------------
 * Flash menu
 * ---------------------------------------------------------------------- */
void menu_flash() {
	unsigned char line = 0;
	unsigned char _state = 0;
	SSD1306_writeString(0, line++, PSTR("-----FLASH------"), 1);

UPDATE_MENU:
	line=3;
	SSD1306_writeString(0, 3, PSTR("<EXIT>"), _state == 0 ? 3 : 1);
	SSD1306_writeString(0, 4, PSTR("<RESET>"), _state == 1 ? 3 : 1);

	inputButton=0;
	while (1) {
  		set_sleep_mode(SLEEP_MODE_IDLE);
  		sleep_mode();

		if (inputButton & 0x01) {
			_state++;
			if (_state == 2) _state = 0;
			goto UPDATE_MENU;
		}

		if (inputButton & 0x02) {
			// Button 2
			if (_state == 0) {
				SSD1306_clear();
				return;
			}
			if (_state == 1) {
				// reset flash
				struct data_header hdr;
				hdr.magic[0] = 'C';
				hdr.magic[1] = 'L';
				hdr.magic[2] = 'G';
				hdr.version = 1;
				hdr.nrec = 0;
				// write to address 0:
				//FLASH_write(0, sizeof(hdr), (uint8_t*)&hdr);
				FLASH_writeBuffer(0, sizeof(hdr), (uint8_t*)&hdr);
				FLASH_writePage(0);
				flashBlock = 0;
				flashRec = 0;
				// read data (to verify)
				hdr.magic[0] = 0x00;
				hdr.magic[1] = 0x00;
				hdr.magic[2] = 0x00;
				hdr.version = 0;
				hdr.nrec = 0xFF;
				FLASH_read(0, sizeof(hdr), (uint8_t*)&hdr);
				if (hdr.magic[0] == 'C' && hdr.magic[1] == 'L' && hdr.magic[2] == 'G' && hdr.version == 1 && hdr.nrec == 0) {
					// compare ok
					SSD1306_writeString(0, 6, PSTR("WRITE OK"), 1);
				} else {
					// comparison failed!
					SSD1306_writeString(0, 6, PSTR("WRITE FAILED"), 1);
				}
				_delay_ms(2000);
				SSD1306_writeString(0, 6, PSTR("            "), 1);
				goto UPDATE_MENU;
			}
		}
	}

}

/* -------------------------------------------------------------------------
 * Setup menu
 * ---------------------------------------------------------------------- */
void menu_setup() {
	unsigned char _state;
#ifdef ENABLE_RFM95
	_state = 0;
#else
	_state = 2;
#endif /* ENABLE_RFM95 */
	SSD1306_writeString(0, 0, PSTR("------SETUP-----"), 1);

	SSD1306_writeString(0, 1, PSTR("LORA"), 1);
UPDATE_MENU:
#ifdef ENABLE_RFM95
	SSD1306_writeString(5, 1, cfgLoraOff ? PSTR("<OFF>") : PSTR("<ON> "), _state == 0 ? 3 : 1);
	SSD1306_writeString(11, 1, sf2pstr(cfgLoraSF), _state == 1 ? 3 : 1);
#else
	SSD1306_writeString(5, 1, PSTR("DISABLED"), 1);
#endif /* ENABLE_RFM95 */
	SSD1306_writeString(0, 2, PSTR("<DATE/TIME>"), _state == 2 ? 3 : 1);
	SSD1306_writeString(0, 3, PSTR("<STATUS>"), _state == 3 ? 3 : 1);
	SSD1306_writeString(0, 4, PSTR("<RESET>"), _state == 4 ? 3 : 1);
	SSD1306_writeString(0, 5, PSTR("<ERASE FLASH>"), _state == 5 ? 3 : 1);
	SSD1306_writeString(0, 6, PSTR("<RETURN>"), _state == 6 ? 3 : 1);

	inputButton=0;
	while (1) {
  		set_sleep_mode(SLEEP_MODE_IDLE);
  		sleep_mode();

		if (inputButton & 0x01) {
			_state++;
#ifdef ENABLE_RFM95
			if (_state == 7) _state = 0;
#else
			if (_state == 7) _state = 2;
#endif
			goto UPDATE_MENU;
		}

		if (inputButton & 0x02) {
			// Button 2
#ifdef ENABLE_RFM95
			if (_state == 0) {
				cfgLoraOff ^= 0x01;
				goto UPDATE_MENU;
			} else if (_state == 1) {
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
				LMIC_setDrTxpow(cfgLoraSF, 20);
				goto UPDATE_MENU;
			}
#endif /* ENABLE_RFM95 */

			SSD1306_clear();
			if (_state == 5) {
				menu_flash();
				return;
			}
			if (_state == 6) return;
		}

	}
}

/* -------------------------------------------------------------------------
 * Info menu
 * ---------------------------------------------------------------------- */
void menu_info() {
	unsigned char line = 0;
	SSD1306_writeString(0, line++, PSTR("------INFO------"), 1);
	SSD1306_writeString(0, line++, PSTR("DEVICE ID:"), 1);

	/* load device address from EEPROM into memory */
	uint32_t devaddr;
	eeprom_read_block(&devaddr, &DEVADDR, sizeof(DEVADDR));
	//SSD1306_writeString(0, line++, PSTR("XX-XX-XX-XX"), 1);
	SSD1306_writeInt(0, line, devaddr >> 16, 16);
	SSD1306_writeInt(4, line++, (int16_t)devaddr, 16);

	SSD1306_writeString(0, line, PSTR("VERSION:"), 1);
	SSD1306_writeString(9, line++, PSTR(CL_VERSION), 1);
	SSD1306_writeString(0, line, PSTR("DATE:"), 1);
	SSD1306_writeString(6, line++, PSTR(CL_DATE), 1);
	line++;
	SSD1306_writeString(0, line++, PSTR("<OK>"), 3);

	inputButton=0;
	while (1) {
  		set_sleep_mode(SLEEP_MODE_IDLE);
  		sleep_mode();

		if (inputButton & 0x02) {
			// Button 2
			SSD1306_clear();
			return;
		}
	}
}

/* -------------------------------------------------------------------------
 * "Home" menu
 * ---------------------------------------------------------------------- */
void menu_home() {
	uint8_t _state = 0;
	unsigned char line;
	struct tm *tm;
AGAIN:
	line = 0;
	SSD1306_writeString(0, line, PSTR("COUNT:"), 1);
	SSD1306_writeInt(7, line++, measCount, 10);

	/* first measurement */
	//SSD1306_writeString(0, line++, PSTR("??.??.???? ??:??"), 1);	// first measurement
	tm = gmtime(&measTsFirst);
	SSD1306_writeInt(0, line, tm->tm_mday, 10);
	SSD1306_writeString(2, line, ".", 0);
	SSD1306_writeInt(3, line, tm->tm_mon, 10);
	SSD1306_writeString(5, line, ".", 0);
	SSD1306_writeInt(6, line, tm->tm_year+1900, 10);

	SSD1306_writeInt(11, line, tm->tm_hour, 10);
	SSD1306_writeString(13, line, ":", 0);
	SSD1306_writeInt(14, line, tm->tm_min, 10);
	line++;

	/* latest measurement */
	//SSD1306_writeString(0, line++, PSTR("??.??.???? ??:??"), 1);	// recent measurement
	tm = gmtime(&measTsLast);
	SSD1306_writeInt(0, line, tm->tm_mday, 10);
	SSD1306_writeString(2, line, ".", 0);
	SSD1306_writeInt(3, line, tm->tm_mon, 10);
	SSD1306_writeString(5, line, ".", 0);
	SSD1306_writeInt(6, line, tm->tm_year+1900, 10);

	SSD1306_writeInt(11, line, tm->tm_hour, 10);
	SSD1306_writeString(13, line, ":", 0);
	SSD1306_writeInt(14, line, tm->tm_min, 10);
	line++;

	SSD1306_writeInt(0, line, measVCC, 10);
	SSD1306_writeString(5, line, "V", 0);

	SSD1306_writeInt(7, line, measPress, 10);
	SSD1306_writeString(13, line++, "HPA", 0);

	SSD1306_writeInt(0, line, measTemp, 10);
	SSD1306_writeString(5, line, "C", 0);

	SSD1306_writeInt(7, line, measHum, 10);
	SSD1306_writeString(13, line++, "%", 0);

	SSD1306_writeInt(0, line, measWind, 10);
	SSD1306_writeString(3, line, "M/S", 0);

#ifdef ENABLE_RFM95
	SSD1306_writeInt(7, line, LMIC.seqnoUp, 10);
	SSD1306_writeString(14, line, "TX", 0);
#endif /* ENABLE_RFM95 */
	line++;

UPDATE_MENU:
	SSD1306_writeString(0, 6, PSTR("<SETUP>"), _state == 0 ? 3 : 1);
	SSD1306_writeString(8, 6, PSTR("<TEST>"), _state == 1 ? 3 : 1);
	SSD1306_writeString(0, 7, PSTR("<INFO>"), _state == 2 ? 3 : 1);
	SSD1306_writeString(7, 7, PSTR("<EXIT>"), _state == 3 ? 3 : 1);

	inputButton = 0;
	while (1) {
  		set_sleep_mode(SLEEP_MODE_IDLE);
  		sleep_mode();

		if (inputButton & 0x01) {
			_state++;
			if (_state == 4) _state = 0;
			goto UPDATE_MENU;
		}

		if (inputButton & 0x02) {
			// Button 2
			SSD1306_clear();
			if (_state == 0) menu_setup();
			if (_state == 2) return; //menu_info();
			if (_state == 3) return;
			goto AGAIN;
		}

	}

}

static volatile unsigned char menu_timeout;

ISR (TIMER1_COMPA_vect) {
	/* timeout interrupt */
	menu_timeout = 1;
}

/* -------------------------------------------------------------------------
 * main menu
 * ---------------------------------------------------------------------- */
void menu_main() {

	SSD1306_writeImg(64, 64, Logo, EEPROM);

	struct RTC_ts ts;
	RTC_get(&ts);

	unsigned char line = 0;
	SSD1306_writeInt(8, line, ts.ts_mday, 10);
	SSD1306_writeString(10, line, ".", 0);
	SSD1306_writeInt(11, line, ts.ts_mon + 1, 10);
	SSD1306_writeString(13, line, ".", 0);
	SSD1306_writeInt(14, line++, ts.ts_year - 100, 10);

	SSD1306_writeInt(8, line, ts.ts_hour, 10);
	SSD1306_writeString(10, line, ":", 0);
	SSD1306_writeInt(11, line, ts.ts_min, 10);
	SSD1306_writeString(13, line, ":", 0);
	SSD1306_writeInt(14, line++, ts.ts_sec, 10);

	// voltage
	SSD1306_writeInt(8, line, measVCC, 10);
	SSD1306_writeString(15, line++, "V", 0);

	// temp
	SSD1306_writeInt(8, line, measTemp, 10);
	SSD1306_writeString(15, line++, "C", 0);

	// humity? pressure? wind?

	// flash buffer pos
	SSD1306_writeString(8, line, "F", 0);
	SSD1306_writeInt(9, line, flashBlock, 10);
	SSD1306_writeString(13, line, "/", 0);
	SSD1306_writeInt(14, line++, flashRec, 10);

//	SSD1306_writeString(8, 7, PSTR("B1: MORE"), 1);

	/* configure timer1 to automatically quit info display after timeout */
	cli();
  	TCCR1A = 0;
  	TCCR1B = 0;
  	TCNT1 = 0;
  	OCR1A = ((F_CPU / 256) * 6) - 1;	/* wait for 6 seconds */
  	TCCR1B |= (1 << CS12);				/* set prescaler to 256 */
  	TCCR1B |= (1 << WGM12);				/* configure timer 1 for CTC mode */
  	TIMSK1 |= (1 << OCIE1A);			/* enable CTC interrupt */
  	sei();
  	menu_timeout = 0;
  	inputButton = 0;
  	while (menu_timeout == 0) {
  		set_sleep_mode(SLEEP_MODE_IDLE);
  		sleep_mode();

  		if (inputButton & 0x01) {
  			break;
  		}

  	}

	SSD1306_clear();
  	TCCR1A = 0;
	TCCR1B = 0;

	if (inputButton & 0x01) {
		// dive into menu...
		menu_home();
	}
}

/* -------------------------------------------------------------------------
 * SD card inserted
 * ---------------------------------------------------------------------- */
void menu_sd_insert() {
	unsigned char line = 0;
	unsigned char _state = 0;
	SSD1306_writeString(0, line++, PSTR("-----SDCARD-----"), 1);
//	SSD1306_writeString(0, line++, PSTR("..."), 1);

	// TODO: show nr of data records

UPDATE_MENU:
	line=3;
	SSD1306_writeString(0, 3, PSTR("<COPY>"), _state == 0 ? 3 : 1);
	SSD1306_writeString(0, 4, PSTR("<EXIT>"), _state == 1 ? 3 : 1);

	inputButton=0;
	FRESULT fr;
	while (1) {
  		set_sleep_mode(SLEEP_MODE_IDLE);
  		sleep_mode();

		if (inputButton & 0x01) {
			_state++;
			if (_state == 2) _state = 0;
			goto UPDATE_MENU;
		}

		if (inputButton & 0x02) {
			// Button 2
			if (_state == 0) {
				// copy data...
				SSD1306_writeString(0, 3, PSTR("MOUNT..."), 1);
				SSD1306_writeString(0, 4, PSTR("      "), 1);
				FATFS fs;
				fr = pf_mount(&fs);
				if (fr != FR_OK) {
					goto PF_ERROR;
				}
				// open file
				SSD1306_writeString(0, 3, PSTR("OPEN... "), 1);
				fr = pf_open("CAVELOG.DAT");
				if (fr != FR_OK) {
					goto PF_ERROR;
				}
				// write data
				UINT wsz = 0;
				uint32_t i;
				unsigned char buf[512];
				for (i=0; i < flashBlock; i++) {
					// read data from flash into memory
					FLASH_read(i * 512, 256, buf);
					FLASH_read((i * 512) + 256, 256, buf+256);
					// write data from memory onto SD
					fr = pf_write(buf, sizeof(buf), &wsz);
					if (fr != FR_OK) goto PF_ERROR;
				}

//				const char data[] = "Hello, World";
//				fr = pf_write(data, sizeof(data), &wsz);
//				SSD1306_writeInt(0, 3, wsz, 10);
//				SSD1306_writeString(8, 3, PSTR("BYTES"), 1);
				// commit write
				pf_write(0, 0, &wsz);
				SSD1306_writeString(0, 4, PSTR("DONE.   "), 1);
				_delay_ms(3000);
//				SSD1306_writeString(0, 3, PSTR("             "), 1);
				goto UPDATE_MENU;
			}
			if (_state == 1) {
				SSD1306_clear();
				return;
			}
		}
		continue;
PF_ERROR:
//		SSD1306_writeString(0, 4, PSTR("ERR:"), 1);
//		SSD1306_writeInt(5, 4, fr, 10);
		SSD1306_writeInt(14, 3, fr, 10);
		_delay_ms(2000);
		goto UPDATE_MENU;
	}

}
