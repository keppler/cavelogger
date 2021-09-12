/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * AT45DB081E.h
 * Interface to adesto AT45DB081E SPI Serial Flash Memory
 */

// Datasheet: https://cdn-reichelt.de/documents/datenblatt/A300/AT45DB081E-XXX.pdf

#include "pins.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stddef.h>
#include "SPI.h"
#include "AT45DB081E.h"

/* some shortcuts for better readability */
#define _FLASH_NSS_DDR		DDR(FLASH_NSS_PORT)
#define _FLASH_NSS_PORT		PORT(FLASH_NSS_PORT)
#define _FLASH_NSS_PORTPIN	PORTPIN(FLASH_NSS_PORT, FLASH_NSS_PIN)

static const uint8_t _FLASH_CMD_READ_ID[]			= { 0x9F };
static const uint8_t _FLASH_CMD_READ_STATUS[]		= { 0xD7 };
static const uint8_t _FLASH_CMD_PAGE_CONFIG_256[]	= { 0x3D, 0x2A, 0x80, 0xA6 };

static void _FLASH_transfer(uint8_t wlen, const uint8_t *wdata, uint16_t rlen, uint8_t *rdata) {

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

static uint16_t _FLASH_readStatus(void) {
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

uint8_t FLASH_init(void) {
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

void FLASH_read(uint32_t addr, uint16_t len, uint8_t *data) {
	unsigned char cmd[4];
	cmd[0] = 0x01;	// 5.5 Continuous Array Read (Low Power Mode)
	// 256 byte size
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = addr & 0xFF;

	_FLASH_transfer(sizeof(cmd), (uint8_t*)&cmd, len, data);
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

uint8_t FLASH_loadPage(uint32_t addr) {
	// load page from flash into BUFFER1
	uint8_t i;

	// 9.1 Main Memory Page to Buffer Transfer
	unsigned char cmd[4];
	cmd[0] = 0x53;	// read into Buffer 1
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = 0x00;	// offset address = dummy bits

	_FLASH_transfer(sizeof(cmd), (uint8_t*)&cmd, 0, NULL);

	/* wait for operation to finish */
	uint16_t status;
	for (i=0; i<20; i++) {	/* try for 1 second */
		_delay_ms(50);
		status = _FLASH_readStatus();
		if (status & 0x8000) return 0;
		// TODO detect error (count nr of loops?) to prevent deadlock! (reset flash chip then, etc.)
	}
	return 1;
}

uint8_t FLASH_writePage(uint32_t addr) {
	// write page (with erase) from BUFFER1 into flash
	uint8_t i;

	// 6.2 Buffer to Main Memory Page Program with Built-In Erase
	unsigned char cmd[4];
	cmd[0] = 0x83;	// read into Buffer 1
	cmd[1] = (addr >> 16) & 0xFF;
	cmd[2] = (addr >> 8) & 0xFF;
	cmd[3] = 0x00;	// offset address = dummy bits

	_FLASH_transfer(sizeof(cmd), (uint8_t*)&cmd, 0, NULL);

	/* wait for operation to finish */
	uint16_t status;
	for (i=0; i<20; i++) {	/* try for 1 second */
		_delay_ms(50);
		status = _FLASH_readStatus();
		if (status & 0x8000) break;
		// TODO detect error (count nr of loops?) to prevent deadlock! (reset flash chip then, etc.)
	}

	if (status & 0x20) {
		// Erase/Program Error !!!
		// TODO -> save in a variable... (maybe with timestamp?)
		return 2;
	}

	return 1;
}

void FLASH_writeBuffer(uint8_t addr, uint8_t len, uint8_t *data) {
	// write data into BUFFER1

	// 6.1 Buffer Write
	uint8_t i;
	unsigned char cmd[4];
	cmd[0] = 0x84;	// read into Buffer 1
	cmd[1] = 0x00;	// dummy bits
	cmd[2] = 0x00;	// dummy bits
	cmd[3] = addr;	// offset address

	/* set NSS LOW */
	_FLASH_NSS_PORT &= ~(1 << _FLASH_NSS_PORTPIN);

	for (i=0; i<4; i++) {
		(void)SPI_transfer(cmd[i]);
	}

	while (len--) {
		SPI_transfer(*data);
		data++;
	}

	/* set NSS HIGH */
	_FLASH_NSS_PORT |= (1 << _FLASH_NSS_PORTPIN);

}
