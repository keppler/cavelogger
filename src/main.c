/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * main.c
 */

#include "main.h"
#include "pins.h"

#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "BMP280.h"
#include "LED.h"
#include "../lib/i2cmaster/i2cmaster.h"
#include "SPI.h"
#if 1
#include "SSD1306.h"
#else
#define SSD1306_on() {};
#define SSD1306_off() {};
#define SSD1306_clear() {};
#define SSD1306_init() {};
#define SSD1306_writeString(a,b,c,d) {};
#define SSD1306_writeInt(a,b,c,d) {};
#endif
#include "RX6110.h"
#include "VCC.h"
#include "Wind.h"
#include <time.h>
#include "../lib/petitfat/pff.h"
#ifdef FLASH_NSS_PORT
#include "AT45DB081E.h"
#endif /* FLASH_NSS_PORT */
#include "menu.h"

/* LMIC */
#include "../lib/arduino-lmic/lmic/lmic.h"
#include "../lib/arduino-lmic/lmic/lmic_eu_like.h"
#include "../lib/arduino-lmic/lmic/hal.h"

static const u1_t EEMEM NWKSKEY[16] = { 0 };
static const u1_t EEMEM APPSKEY[16] = { 0 };
static const u4_t EEMEM DEVADDR = 0x0;

uint16_t int0Count = 0;
static volatile uint8_t _lmic_done;
static volatile uint8_t int_rtc = 0;
volatile uint8_t inputButton = 0x00;
uint8_t cfgLoraOff = 0;
uint8_t cfgLoraSF = DR_SF11;
uint8_t displayOff = 0;
uint8_t powerLoss = 0;
uint16_t sendTS[5] = {0,0,0,0,0};	/* ring buffer of send timestamp, to calculate time */
int32_t measTemp = 0;
uint16_t measHum = 0;
uint16_t measPress;
uint16_t measVCC;
uint16_t measWind;
uint32_t measureCount = 0;

// https://javl.github.io/image2cpp/
// invert image colors; vertical, 1 bit per pixel
static const unsigned char Logo[] EEMEM = {
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


/* interrupt handler for RTC */
ISR(INT0_vect) {
	int0Count++;
	int_rtc = 1;
}

/* interrupt handler for Button_1 */
ISR(PCINT0_vect) {
	if (!(PINB & (1 << PINB0))) {
		inputButton |= 0x01;
		LED_blink();
	}
	if (!(PINB & (1 << PINB1))) {
		inputButton |= 0x02;
		LED_blink();
	}
}

static void parseRX(uint8_t *data, size_t dataLen) {

	if (dataLen != 9) return;

	if (!(data[0] & 0x01)) return;	/* unknown payload */

	int16_t rssi = ((int16_t)data[1] << 8) | data[2];
	uint8_t snr = data[3];

	if (powerLoss) {
		/* set clock */
		uint32_t ts = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | data[7];
		ts -= UNIX_OFFSET;
		if (data[8] < 5) {
			ts += sendTS[data[8]];
		}

		/* save time to RTC */
		struct tm *tm = gmtime(&ts);
		struct RTC_ts rts;
		rts.ts_sec = tm->tm_sec;
		rts.ts_min = tm->tm_min;
		rts.ts_hour = tm->tm_hour;
		rts.ts_wday = tm->tm_wday;
		rts.ts_mday = tm->tm_mday;
		rts.ts_mon = tm->tm_mon;
		rts.ts_year = tm->tm_year;
		RTC_set(&rts);
		powerLoss = 0;

		SSD1306_clear();	/* clear display */

		SSD1306_writeString(0, 0, PSTR("  .  .    "), 1);
		SSD1306_writeInt(0, 0, tm->tm_mday, 10);
		SSD1306_writeInt(3, 0, tm->tm_mon + 1, 10);
		SSD1306_writeInt(6, 0, tm->tm_year + 1900, 10);

		SSD1306_writeString(0, 1, PSTR("  :  :  "), 1);

		SSD1306_writeInt(0, 1, tm->tm_hour, 10);
		SSD1306_writeInt(3, 1, tm->tm_min, 10);
		SSD1306_writeInt(6, 1, tm->tm_sec, 10);

	} else {
		SSD1306_clear();	/* clear display */
	}

	SSD1306_writeString(0, 2, "UP:", 0);
	SSD1306_writeInt(4, 2, rssi, 10);
	SSD1306_writeInt(12, 2, snr, 10);

	SSD1306_writeString(0, 3, "DN:", 0);
	SSD1306_writeInt(4, 3, LMIC.rssi, 10);
	SSD1306_writeInt(12, 3, LMIC.snr, 10);

}

unsigned char Data[11];
static osjob_t sendjob;

void do_send(osjob_t* j){
	if (LMIC.opmode & OP_TXRXPEND) {
//		Serial.println(F("OP_TXRXPEND, not sending"));
	} else {
		// Prepare upstream data transmission at the next possible time.
		LMIC_setTxData2(1, Data, sizeof(Data), 0);
	}
}

void onEvent (ev_t ev) {
	switch(ev) {
		case EV_SCAN_TIMEOUT:
			break;
		case EV_BEACON_FOUND:
			break;
		case EV_BEACON_MISSED:
			break;
		case EV_BEACON_TRACKED:
			break;
		case EV_JOINING:
			break;
		case EV_JOINED:
			break;
		case EV_JOIN_FAILED:
			break;
		case EV_REJOIN_FAILED:
			break;
		case EV_TXCOMPLETE:
			if (!displayOff) SSD1306_writeString(0, 1, "TX DONE", 0);
//			Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
//			if (LMIC.txrxFlags & TXRX_ACK)
//			  Serial.println(F("Received ack"));
			if (LMIC.dataLen) {
				parseRX(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
			}
			_lmic_done = 1;
			// Schedule next transmission
			//os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
			break;
		case EV_LOST_TSYNC:
			break;
		case EV_RESET:
			break;
		case EV_RXCOMPLETE:
			break;
		case EV_LINK_DEAD:
			break;
		case EV_LINK_ALIVE:
			break;
		case EV_TXSTART:
			break;
		case EV_TXCANCELED:
			break;
		case EV_RXSTART:
			break;
		case EV_JOIN_TXCOMPLETE:
			break;
		default:
			break;
	}
}

#if 0
static int nr_of_files_on_card(void) {
	// https://github.com/spanceac/dariabox/blob/master/daria-box.c#L102
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int count = 0;

	res = pf_opendir(&dir, "/");
	if (res == FR_OK) {
		for (;;) {
			res = pf_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0) break;
			if (!(fno.fattrib & AM_DIR)) //if it's a file name and not a dir
				count ++;
		}
	}
	return count;
}
#endif

int main(void) {
	uint8_t line;

	/* turn off unused modules */
	power_adc_disable();	/* disable ADC converter */
	power_usart0_disable();	/* disable USART */

	LED_init();
	i2c_init();

	/* Setup SPI bus
	 * Initialize all CS lines before accessing *any* device on the SPI bus! */
	SPI_init();
	RTC_setup();
	BMP280_setup();
#ifdef FLASH_NSS_PORT
	FLASH_setup();
#endif /* FLASH_NSS_PORT */
#if 0
	DDR(SD_NSS_PORT) |= (1 << PORTPIN(SD_NSS_PORT, SD_NSS_PIN));
	PORT(SD_NSS_PORT) |= (1 << PORTPIN(SD_NSS_PORT, SD_NSS_PIN));	/* set NSS HIGH */
#ifdef SD_PWR_PORT
	DDR(SD_PWR_PORT) |= (1 << PORTPIN(SD_PWR_PORT, SD_PWR_PIN));
	PORT(SD_PWR_PORT) |= (1 << PORTPIN(SD_PWR_PORT, SD_PWR_PIN));	/* power on */
#endif /* SD_PWR_PORT */
#endif

#ifdef OLED_PWR_PORT
	DDR(OLED_PWR_PORT) |= (1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));
	PORT(OLED_PWR_PORT) |= (1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));	/* power on */
#endif /* OLED_PWR_PORT */
	DDR(RFM95_NSS_PORT) |= (1 << PORTPIN(RFM95_NSS_PORT, RFM95_NSS_PIN));
	PORT(RFM95_NSS_PORT) |= (1 << PORTPIN(RFM95_NSS_PORT, RFM95_NSS_PIN));	/* set NSS HIGH */

	SSD1306_init();
	SSD1306_clear();	/* clear display */
	SSD1306_on();

	// load device address from EEPROM into memory
	u4_t devaddr;
	eeprom_read_block(&devaddr, &DEVADDR, sizeof(DEVADDR));

	SSD1306_writeImg(64, 64, Logo, EEPROM);
	SSD1306_writeString(8, 0, PSTR("VERSION:"), 1);
	SSD1306_writeString(8, 1, PSTR("1.5"), 1);
	SSD1306_writeString(8, 2, PSTR("01/2021"), 1);
	SSD1306_writeString(8, 4, PSTR("ADDR/ID:"), 1);
	SSD1306_writeInt(8, 5, devaddr >> 16, 16);
	SSD1306_writeInt(12, 5, (int16_t)devaddr, 16);
	SSD1306_writeString(8, 6, "XX XX XX", 0);

	_delay_ms(4000);

	SSD1306_clear();
	line=0;
	SSD1306_writeString(0, line++, PSTR("INITIALIZING..."), 1);

	SSD1306_writeString(0, line, PSTR("RTC:"), 1);
	if (RTC_init() != 0) {
		SSD1306_writeString(8, line++, PSTR("ERROR!"), 1);
		while(1) LED_blink();
	}
	// if (RTC_powerLoss()) {
	if (1) {
		powerLoss = 1;
		SSD1306_writeString(0, 7, PSTR("POWER LOSS!"), 1);
		RTC_reset();
		RTC_setTimer(60);
	} else {
		RTC_setTimer(900);
	}
	SSD1306_writeString(8, line++, PSTR("OK."), 1);
	Wind_init();
	SSD1306_writeString(0, line, PSTR("BMP280:"), 1);
	if (BMP280_init() == BMP280_TYPE_UNKNOWN) {
		SSD1306_writeString(8, line++, PSTR("ERROR!"), 1);
		while(1) LED_blink();
	}
#ifdef BMP280_PWR_PORT
	BMP280_off();
#endif /* BMP280_PWR_PORT */
	SSD1306_writeString(8, line++, PSTR("OK."), 1);

#ifdef FLASH_NSS_PORT
	SSD1306_writeString(0, line, PSTR("FLASH:"), 1);
	if (FLASH_init() != 0) {
		SSD1306_writeString(8, line++, PSTR("ERROR!"), 1);
		while(1) LED_blink();
	}
	SSD1306_writeString(8, line++, PSTR("OK."), 1);

	// send flash to sleep mode...
	FLASH_enter_sleep();

#endif /* FLASH_NSS_PORT */

#if 0
	/* SD Card */
	FATFS FatFs;
	if ((u = pf_mount(&FatFs)) != FR_OK) {
		/* error while mounting SD card */
		SSD1306_writeString(0, 0, "SD ERROR:", 0);
		SSD1306_writeInt(10, 0, u, 10);
		while(1) LED_blink();
	}
	SSD1306_writeString(0, 0, "SD OK", 0);
	_delay_ms(1000);
	SSD1306_writeString(0, 0, "FILES", 0);
	SSD1306_writeInt(0, 1, nr_of_files_on_card(), 10);
	_delay_ms(1000);
	SSD1306_writeString(0, 0, "     ", 0);
	SSD1306_writeString(0, 1, "     ", 0);
#endif

	/* LMIC */
	SSD1306_writeString(0, line, PSTR("RFM95:"), 1);
	os_init();
	LMIC_reset();
	uint8_t nwkskey[sizeof(NWKSKEY)];
	eeprom_read_block(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	uint8_t appskey[sizeof(APPSKEY)];
	eeprom_read_block(appskey, APPSKEY, sizeof(APPSKEY));
	LMIC_setSession (0x13, devaddr, nwkskey, appskey);

#if defined(CFG_eu868)
	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);	  // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);	  // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);	  // g2-band
	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.
#else
	#error LMIC: Unsupported band!
#endif

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// Let LMIC compensate for +/- 5% clock error
	LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink
	LMIC_setDrTxpow(cfgLoraSF, 14);

	DDRD &= ~(1 << DDD2);	/* configure PD2 (INT0) as INPUT */
	PORTD |= (1 << PORTD2);	/* enable internal pull-up */

	SSD1306_writeString(8, line++, PSTR("OK."), 1);

	cli();

	DDRB &= ~((1 << DDB0) | (1 << DDB1));		/* configure PB0 as INPUT */
	PORTB |= ((1 << PORTB0) | (1 << PORTB1));	/* enable internal pull-up */
	PCICR |= (1 << PCIE0);						/* enable interrupts on PORTB */
	PCMSK0 |= ((1 << PCINT0) | (1 << PCINT1));	/* allow int on PB0 */

	/* EICRA (External Interrupt Control Register A): enable INT0 */
	/* Falling edge of INT0 will fire the ISR */
	EICRA &= ~(1 << ISC00);
	EICRA |= (1 << ISC01);

	/* enable interrupt INT0 */
	EIMSK |= (1 << INT0);

	sei();

	SSD1306_writeString(0, line++, PSTR("READY. "), 1);

	_delay_ms(2000);
	SSD1306_clear();

	uint8_t cnt = 0;

	/* trigger measurement */
	int_rtc = 1;

	while(1) {

		if (int_rtc) {
			int_rtc = 0;
			measureCount++;

			if (!displayOff) {
				LED_blink();
				LED_blink();

				SSD1306_writeString(0, 0, PSTR("TEMP:"), 1);
				SSD1306_writeString(0, 1, PSTR("PRESS:"), 1);
				SSD1306_writeString(0, 2, PSTR("HUMI:"), 1);
				SSD1306_writeString(0, 3, PSTR("VCC:"), 1);
				SSD1306_writeString(0, 4, PSTR("TIME:"), 1);
				SSD1306_writeString(0, 5, PSTR("WIND:"), 1);
			}

#ifdef BMP280_PWR_PORT
			BMP280_on();
#endif /* BMP280_PWR_PORT */
			if (!displayOff) SSD1306_writeString(1, 6, "A", 0);
			measTemp = BMP280_temp();
			measHum = ((BME280_humidity() >> 6) * 10) >> 4;;
			//uint8_t hum = BME280_humidity() >> 10;
			measPress = BMP280_pressure() / 10;
#ifdef BMP280_PWR_PORT
			BMP280_off();
#endif /* BMP280_PWR_PORT */

			if (!displayOff) SSD1306_writeString(1, 6, "B", 0);
			if (!displayOff) {
				SSD1306_writeInt(7, 0, measTemp, 10);
				SSD1306_writeInt(7, 1, (int32_t)measPress, 10);
				SSD1306_writeInt(7, 2, (int32_t)measHum, 10);
			}

			if (!displayOff) SSD1306_writeString(1, 6, "C", 0);
			measVCC = VCC_get();
			if (!displayOff) SSD1306_writeInt(7, 3, measVCC, 10);

			if (!displayOff) SSD1306_writeString(1, 6, "D", 0);
			struct RTC_ts ts;
			RTC_get(&ts);
			if (!displayOff) {
				SSD1306_writeInt(7, 4, ts.ts_hour, 10);
				SSD1306_writeString(9, 4, ":", 0);
				SSD1306_writeInt(10, 4, ts.ts_min, 10);
				SSD1306_writeString(12, 4, ":", 0);
				SSD1306_writeInt(13, 4, ts.ts_sec, 10);
			}

			if (!displayOff) SSD1306_writeString(1, 6, "E", 0);
			if (!displayOff) SSD1306_writeString(0, 6, "W", 0);
			measWind = Wind_get();
			if (!displayOff) {
				SSD1306_writeString(0, 6, ".", 0);
				SSD1306_writeString(7, 5, "     ", 0);
				SSD1306_writeInt(7, 5, measWind, 10);
			}
//			uint8_t wind = 0;

			if (!displayOff) SSD1306_writeInt(7, 6, int0Count, 10);

			if (!displayOff) SSD1306_writeInt(12, 6, LMIC.seqnoUp, 10);

			if (!displayOff) SSD1306_writeString(1, 6, "F", 0);
			measTemp /= 10;
			Data[0] = 0x01;	// header (payload format)
			if (powerLoss) {
				/* set "power loss" flag */
				Data[0] |= 0x80;
				/* save timestamp in ring buffer */
				sendTS[LMIC.seqnoUp % 5] = (ts.ts_hour * 3600) + (ts.ts_min * 60) + ts.ts_sec;
			}
			Data[1] = (measTemp >> 8) & 0xFF;
			Data[2] = measTemp & 0xFF;
			Data[3] = (measPress >> 8) & 0xff;
			Data[4] = measPress & 0xff;
			Data[5] = (measHum >> 8) & 0xff;
			Data[6] = measHum & 0xff;
			Data[7] = measVCC >> 8;
			Data[8] = measVCC & 0xFF;
			Data[9] = (measWind >> 8) & 0xff;
			Data[10] = measWind & 0xff;

			_lmic_done = 0;

			if (!cfgLoraOff) {
				do_send(&sendjob);

				while(!_lmic_done) os_runloop_once();

				LED_blink();

				if (LMIC.seqnoUp == 3) {
					/* increase wait time to 15min after 3rd frame */
					RTC_setTimer(900);
				}

				if (LMIC.seqnoUp < 3) {
					_delay_ms(4000);
				} else {
					_delay_ms(1000);
				}
			}

		}

		if (inputButton & 0x01) {
			inputButton &= ~(0x01);
			// button triggered...
			//SSD1306_writeString(0, 0, "BUTTON1...", 0);
			//_delay_ms(1000);
			if (displayOff) {
#ifdef OLED_PWR_PORT
				PORT(OLED_PWR_PORT) |= (1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));	/* power on */
				SSD1306_init();
#endif /* OLED_PWR_PORT */

				if (!displayOff) {
					SSD1306_clear();
					SSD1306_on();
				}
			}
			menu_main();
			SSD1306_off();
			if (displayOff) SSD1306_off();
		}

		if (!displayOff) SSD1306_off();

#ifdef SD_PWR_PORT
		PORT(SD_PWR_PORT) &= ~(1 << PORTPIN(SD_PWR_PORT, SD_PWR_PIN));	/* power off */
#endif /* SD_PWR_PORT */
#ifdef OLED_PWR_PORT
		if (!displayOff) PORT(OLED_PWR_PORT) &= ~(1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));	/* power off */
#endif /* OLED_PWR_PORT */

		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//		sleep_enable();
		power_spi_disable();
		power_twi_disable();

		/* only INT0 should be able to wake us up... */
//		sleep_cpu();

//		sleep_disable();
		sleep_mode();

		power_twi_enable();
		power_spi_enable();

#ifdef OLED_PWR_PORT
		if (!displayOff) {
			PORT(OLED_PWR_PORT) |= (1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));	/* power on */
			SSD1306_init();
		}
#endif /* OLED_PWR_PORT */

		if (!displayOff) {
			SSD1306_clear();
			SSD1306_on();
		}

		if (int_rtc && measureCount == 3) {
			/* after 3rd measurement: power-off display during all measurements*/
			displayOff = 1;
		}

	}

}
