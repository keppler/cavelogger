#include "pins.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stddef.h>
#include "SPI.h"
#include "AT45DB081E.h"

// https://cdn-reichelt.de/documents/datenblatt/A300/AT45DB081E-XXX.pdf

/* some shortcuts for better readability */
#define _FLASH_NSS_DDR		DDR(FLASH_NSS_PORT)
#define _FLASH_NSS_PORT		PORT(FLASH_NSS_PORT)
#define _FLASH_NSS_PORTPIN	PORTPIN(FLASH_NSS_PORT, FLASH_NSS_PIN)

static const uint8_t _FLASH_CMD_READ_ID[]			= { 0x9F };
static const uint8_t _FLASH_CMD_READ_STATUS[]		= { 0xD7 };
static const uint8_t _FLASH_CMD_PAGE_CONFIG_256[]	= { 0x3D, 0x2A, 0x80, 0xA6 };

static void _FLASH_transfer(uint8_t wlen, const uint8_t *wdata, uint8_t rlen, uint8_t *rdata) {

	/* set NSS LOW */
	_FLASH_NSS_PORT &= ~(1 << _FLASH_NSS_PORTPIN);

	while (wlen--) {
		(void)SPI_transfer(*wdata);
		wdata++;
	}

	while (rlen--) {
		*rdata = SPI_transfer(0xFF);
		rdata++;
	}

	/* set NSS HIGH */
	_FLASH_NSS_PORT |= (1 << _FLASH_NSS_PORTPIN);

}

static uint16_t _FLASH_readStatus() {
	uint16_t status;

	/* read status register */
	_FLASH_transfer(sizeof(_FLASH_CMD_READ_STATUS), _FLASH_CMD_READ_STATUS, 2, (uint8_t*)&status);

	return(status);
}

void FLASH_setup(void) {
	/* configure OUTPUT pins */
	_FLASH_NSS_DDR |= (1 << _FLASH_NSS_PORTPIN);
	_FLASH_NSS_PORT |= (1 << _FLASH_NSS_PORTPIN);	/* set NSS HIGH */
}

uint8_t FLASH_init() {
	uint8_t data[3];

	/* read manufacturer and device id */
	_FLASH_transfer(sizeof(_FLASH_CMD_READ_ID), _FLASH_CMD_READ_ID, 3, data);

	if (data[0] != 0x1F || data[1] != 0x25 || data[2] != 0x00) return(1);

	/* read status */
	uint16_t status = _FLASH_readStatus();

	if (status & (1 << 8)) {
		/* page size is configured to 264 bytes;
		 * reconfigure to 256 bytes: */
		_FLASH_transfer(sizeof(_FLASH_CMD_PAGE_CONFIG_256), _FLASH_CMD_PAGE_CONFIG_256, 0, NULL);
		/* wait until operation completed */
		do {
			_delay_ms(50);
		} while(!(_FLASH_readStatus() & (1 << 7)));
	}

	return(0);
}

void FLASH_read(uint32_t addr, uint8_t len, uint8_t *data) {
	uint8_t addrbuf[3];

	addrbuf[2] = (addr >> 16) & 0xFF;
	addrbuf[1] = (addr >> 8) & 0xFF;
	addrbuf[0] = addr & 0xFF;

	_FLASH_transfer(3, addrbuf, len, data);
}

void FLASH_enter_sleep(void) {
	/* enter "ultra-deep power-down" */
	/* buffer contents will be cleared! */

	/* set NSS LOW */
	_FLASH_NSS_PORT &= ~(1 << _FLASH_NSS_PORTPIN);

	(void)SPI_transfer(0x79);

	/* set NSS HIGH */
	_FLASH_NSS_PORT |= (1 << _FLASH_NSS_PORTPIN);

}

void FLASH_leave_sleep(void) {
	/* leave "ultra-deep power-down" */
	// page 34; send dummy opcode...

	/* set NSS LOW */
	_FLASH_NSS_PORT &= ~(1 << _FLASH_NSS_PORTPIN);

	(void)SPI_transfer(0x00);

	/* set NSS HIGH */
	_FLASH_NSS_PORT |= (1 << _FLASH_NSS_PORTPIN);

}
