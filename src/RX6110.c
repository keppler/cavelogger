#include "pins.h"
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <string.h>			/* ffs() */
#include "SPI.h"
#include "RX6110.h"

// https://github.com/torvalds/linux/blob/master/drivers/rtc/rtc-rx6110.c

/* some shortcuts for better readability */
#define _RTC_NSS_DDR		DDR(RTC_NSS_PORT)
#define _RTC_NSS_PORT		PORT(RTC_NSS_PORT)
#define _RTC_NSS_PORTPIN	PORTPIN(RTC_NSS_PORT, RTC_NSS_PIN)

/* RX-6110 Register definitions */
#define RX6110_REG_SEC		0x10
#define RX6110_REG_MIN		0x11
#define RX6110_REG_HOUR		0x12
#define RX6110_REG_WDAY		0x13
#define RX6110_REG_MDAY		0x14
#define RX6110_REG_MONTH	0x15
#define RX6110_REG_YEAR		0x16
#define RX6110_REG_RES1		0x17
#define RX6110_REG_ALMIN	0x18
#define RX6110_REG_ALHOUR	0x19
#define RX6110_REG_ALWDAY	0x1A
#define RX6110_REG_TCOUNT0	0x1B
#define RX6110_REG_TCOUNT1	0x1C
#define RX6110_REG_EXT		0x1D
#define RX6110_REG_FLAG		0x1E
#define RX6110_REG_CTRL		0x1F
#define RX6110_REG_USER0	0x20
#define RX6110_REG_USER1	0x21
#define RX6110_REG_USER2	0x22
#define RX6110_REG_USER3	0x23
#define RX6110_REG_USER4	0x24
#define RX6110_REG_USER5	0x25
#define RX6110_REG_USER6	0x26
#define RX6110_REG_USER7	0x27
#define RX6110_REG_USER8	0x28
#define RX6110_REG_USER9	0x29
#define RX6110_REG_USERA	0x2A
#define RX6110_REG_USERB	0x2B
#define RX6110_REG_USERC	0x2C
#define RX6110_REG_USERD	0x2D
#define RX6110_REG_USERE	0x2E
#define RX6110_REG_USERF	0x2F
#define RX6110_REG_RES2		0x30
#define RX6110_REG_RES3		0x31
#define RX6110_REG_IRQ		0x32

#define BIT(x)	(1 << x)

#define RX6110_BIT_ALARM_EN		BIT(7)

/* Extension Register (1Dh) bit positions */
#define RX6110_BIT_EXT_TSEL0	BIT(0)
#define RX6110_BIT_EXT_TSEL1	BIT(1)
#define RX6110_BIT_EXT_TSEL2	BIT(2)
#define RX6110_BIT_EXT_WADA		BIT(3)
#define RX6110_BIT_EXT_TE		BIT(4)
#define RX6110_BIT_EXT_USEL		BIT(5)
#define RX6110_BIT_EXT_FSEL0	BIT(6)
#define RX6110_BIT_EXT_FSEL1	BIT(7)

/* Flag Register (1Eh) bit positions */
#define RX6110_BIT_FLAG_VLF		BIT(1)
#define RX6110_BIT_FLAG_AF		BIT(3)
#define RX6110_BIT_FLAG_TF		BIT(4)
#define RX6110_BIT_FLAG_UF		BIT(5)

/* Control Register (1Fh) bit positions */
#define RX6110_BIT_CTRL_TBKE	BIT(0)
#define RX6110_BIT_CTRL_TBKON	BIT(1)
#define RX6110_BIT_CTRL_TSTP	BIT(2)
#define RX6110_BIT_CTRL_AIE		BIT(3)
#define RX6110_BIT_CTRL_TIE		BIT(4)
#define RX6110_BIT_CTRL_UIE		BIT(5)
#define RX6110_BIT_CTRL_STOP	BIT(6)
#define RX6110_BIT_CTRL_TEST	BIT(7)

#define RX6110_BIT_BKSMP0		BIT(0)
#define RX6110_BIT_BKSMP1		BIT(1)
#define RX6110_BIT_BKSOFF		BIT(2)
#define RX6110_BIT_BKSON		BIT(3)
#define RX6110_BIT_IOCUTEN		BIT(4)

/* IRQ Register (32h) bit positions */
#define RX6110_BIT_IRQ_TMPIN	BIT(2)

enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WDAY,
	RTC_MDAY,
	RTC_MONTH,
	RTC_YEAR,
	RTC_NR_TIME
};

static void _RX6110_write(uint8_t addr, uint8_t data) {

	/* set NSS HIGH */
	_RTC_NSS_PORT |= (1 << _RTC_NSS_PORTPIN);

	(void)SPI_transfer(addr & 0x7F);
	(void)SPI_transfer(data);

	/* set NSS LOW */
	_RTC_NSS_PORT &= ~(1 << _RTC_NSS_PORTPIN);

}

static void _RX6110_write_array(uint8_t addr, uint8_t len, uint8_t *data) {

	/* set NSS HIGH */
	_RTC_NSS_PORT |= (1 << _RTC_NSS_PORTPIN);

	(void)SPI_transfer(addr & 0x7F);
	while (len--) {
		(void)SPI_transfer(*data);
		data++;
	}

	/* set NSS LOW */
	_RTC_NSS_PORT &= ~(1 << _RTC_NSS_PORTPIN);

}

static uint8_t _RX6110_read(uint8_t addr) {
	uint8_t data;

	/* set NSS HIGH */
	_RTC_NSS_PORT |= (1 << _RTC_NSS_PORTPIN);

	(void)SPI_transfer(addr | 0x80);
	data = SPI_transfer(0x00);

	/* set NSS LOW */
	_RTC_NSS_PORT &= ~(1 << _RTC_NSS_PORTPIN);

	return(data);
}

static void _RX6110_read_array(uint8_t addr, uint8_t len, uint8_t *data) {

	/* set NSS HIGH */
	_RTC_NSS_PORT |= (1 << _RTC_NSS_PORTPIN);

	(void)SPI_transfer(addr | 0x80);
	while (len--) {
		*data = SPI_transfer(0x00);
		data++;
	}

	/* set NSS LOW */
	_RTC_NSS_PORT &= ~(1 << _RTC_NSS_PORTPIN);

}

static uint8_t bin2bcd(uint8_t val) {
	return ((val / 10) << 4) + val % 10;
}

static uint8_t bcd2bin(uint8_t val) {
	return (val & 0x0f) + (val >> 4) * 10;
}

void RTC_setup(void) {
	/* configure OUTPUT pins */
	_RTC_NSS_DDR |= (1 << _RTC_NSS_PORTPIN);
	_RTC_NSS_PORT &= ~(1 << _RTC_NSS_PORTPIN);	/* set NSS LOW */
}

uint8_t RTC_init(void) {
	uint8_t u;

	/* wait 40 ms for oscillator start */
	_delay_ms(40);

	u = _RX6110_read(RX6110_REG_EXT);
	u &= ~(RX6110_BIT_EXT_TE);
	_RX6110_write(RX6110_REG_EXT, u);

	/* Power-on Reset */
	_RX6110_write(RX6110_REG_RES1,   0xB8);
	_RX6110_write(RX6110_REG_RES2,   0x00);
	_RX6110_write(RX6110_REG_RES3,   RX6110_BIT_IOCUTEN | RX6110_BIT_BKSOFF);
	_RX6110_write(RX6110_REG_IRQ,    0x00);
	_RX6110_write(RX6110_REG_ALMIN,  0x00);
	_RX6110_write(RX6110_REG_ALHOUR, 0x00);
	_RX6110_write(RX6110_REG_ALWDAY, 0x00);

	/* clear all flags BUT VLF */
	u = _RX6110_read(RX6110_REG_FLAG);
	u &= ~(RX6110_BIT_FLAG_AF | RX6110_BIT_FLAG_UF | RX6110_BIT_FLAG_TF);
	_RX6110_write(RX6110_REG_FLAG, u);

	/* wait 2ms */
	_delay_ms(2);

	/* check reserved flags registers if initialization looks good */
	u = _RX6110_read(RX6110_REG_RES1);
	if (u != 0xA8 && u != 0xB8) return(1);

	/* initialization ok */
	return(0);

}

void RTC_get(struct RTC_ts *ts) {
	uint8_t data[7];

	_RX6110_read_array(RX6110_REG_SEC, 7, data);
	ts->ts_sec = bcd2bin(data[RTC_SEC] & 0x7F);
	ts->ts_min = bcd2bin(data[RTC_MIN] & 0x7F);
	ts->ts_hour = bcd2bin(data[RTC_HOUR] & 0x3f);
	ts->ts_wday = ffs(data[RTC_WDAY] & 0x7f);
	ts->ts_mday = bcd2bin(data[RTC_MDAY] & 0x3f);
	ts->ts_mon = bcd2bin(data[RTC_MONTH] & 0x1f) - 1;
	ts->ts_year = bcd2bin(data[RTC_YEAR]) + 100;
}

void RTC_reset(void) {

	/* set date */
	struct RTC_ts ts = {0, 0, 0, 6, 1, 1, 0};	/* 01.01.2000 00:00:00 */
	RTC_set(&ts);

}

void RTC_setTimer(uint16_t interval) {
	uint8_t u;

	/* clear TE */
	u = _RX6110_read(RX6110_REG_EXT);
	u &= ~(RX6110_BIT_EXT_TE);
	_RX6110_write(RX6110_REG_EXT, u);

	/* set timer counter */
	_RX6110_write(RX6110_REG_TCOUNT0, interval & 0xFF);
	_RX6110_write(RX6110_REG_TCOUNT1, interval >> 8);

	/* set TMPIN=1 (IRQ1) */
	u = _RX6110_read(RX6110_REG_IRQ);
	u |= (RX6110_BIT_IRQ_TMPIN);
	_RX6110_write(RX6110_REG_IRQ, u);

	/* set TIE=1 (Timer Interrupt Enable) */
	u = _RX6110_read(RX6110_REG_CTRL);
	u |= (RX6110_BIT_CTRL_TIE);
	_RX6110_write(RX6110_REG_CTRL, u);

	/* set TE (Timer Enable) */
	u = _RX6110_read(RX6110_REG_EXT);
	u &= ~(RX6110_BIT_EXT_TSEL0 | RX6110_BIT_EXT_TSEL2);
	u |= (RX6110_BIT_EXT_TE | RX6110_BIT_EXT_TSEL1);
	_RX6110_write(RX6110_REG_EXT, u);

}

void RTC_set(const struct RTC_ts *ts) {
	uint8_t data[RTC_NR_TIME], u;

	data[RTC_SEC]	= bin2bcd(ts->ts_sec);
	data[RTC_MIN]	= bin2bcd(ts->ts_min);
	data[RTC_HOUR]	= bin2bcd(ts->ts_hour);
	data[RTC_WDAY]	= BIT(bin2bcd(ts->ts_wday));
	data[RTC_MDAY]	= bin2bcd(ts->ts_mday);
	data[RTC_MONTH]	= bin2bcd(ts->ts_mon + 1);
	data[RTC_YEAR]	= bin2bcd(ts->ts_year % 100);

	/* set STOP bit */
	u = _RX6110_read(RX6110_REG_CTRL);
	u |= (RX6110_BIT_CTRL_STOP);
	_RX6110_write(RX6110_REG_CTRL, u);

	/* set time/date */
	_RX6110_write_array(RX6110_REG_SEC, RTC_NR_TIME, data);

	/* clear VLF */
	u = _RX6110_read(RX6110_REG_FLAG);
	u &= ~(RX6110_BIT_FLAG_VLF);
	_RX6110_write(RX6110_REG_FLAG, u);

	/* clear STOP bit */
	u = _RX6110_read(RX6110_REG_CTRL);
	u &= ~(RX6110_BIT_CTRL_STOP);
	_RX6110_write(RX6110_REG_CTRL, u);

}

uint8_t RTC_powerLoss() {
	uint8_t u;
	/* check if VLF bit is set */
	u = _RX6110_read(RX6110_REG_FLAG);
	return((u & RX6110_BIT_FLAG_VLF) ? 1 : 0);
}
