/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for Petit FatFs (C)ChaN, 2014      */
/*-----------------------------------------------------------------------*/

// https://github.com/Traumflug/Teacup_Firmware/blob/master/pff_diskio.c
// https://github.com/spanceac/dariabox/blob/master/petitfat/sd-mmc.c

#include "../../src/pins.h"
#include "pff.h"
#include "pffconf.h"
#include "diskio.h"
#include <avr/io.h>
#include <util/delay.h>
#include "../../src/SPI.h"

/* some shortcuts for better readability */
#define _SD_NSS_DDR		DDR(SD_NSS_PORT)
#define _SD_NSS_PORT	PORT(SD_NSS_PORT)
#define _SD_NSS_PORTPIN	PORTPIN(SD_NSS_PORT, SD_NSS_PIN)

/* Definitions for MMC/SDC command. */
#define CMD0	(0x40 + 0)	/* GO_IDLE_STATE */
#define CMD1	(0x40 + 1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0 + 41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40 + 8)	/* SEND_IF_COND */
#define CMD16	(0x40 + 16)	/* SET_BLOCKLEN */
#define CMD17	(0x40 + 17)	/* READ_SINGLE_BLOCK */
#define CMD24	(0x40 + 24)	/* WRITE_BLOCK */
#define CMD55	(0x40 + 55)	/* APP_CMD */
#define CMD58	(0x40 + 58)	/* READ_OCR */

/* Card type flags (card_type). */
#define CT_MMC		0x01	/* MMC ver 3 */
#define CT_SD1		0x02	/* SD ver 1 */
#define CT_SD2		0x04	/* SD ver 2 */
#define CT_BLOCK	0x08	/* Block addressing */

static BYTE card_type;		/* Card type flags. */

/** Send a command packet to MMC/SD card.
  \param cmd Command.
  \param arg Argument (32 bit).
  \return response byte (bit 7 == 1: send failed).
*/
static BYTE send_cmd(BYTE cmd, DWORD arg) {
	BYTE n, res;

	if (cmd & 0x80) {   /* ACMD<n> is the command sequence of CMD55-CMD<n>. */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card */
	_SD_NSS_PORT |= (1 << _SD_NSS_PORTPIN);
	(void)SPI_transfer(0xFF);
	_SD_NSS_PORT &= ~(1 << _SD_NSS_PORTPIN);
	(void)SPI_transfer(0xFF);

	/* Send command packet */
	(void)SPI_transfer(cmd);                        /* Start + Command index */
	(void)SPI_transfer((BYTE)(arg >> 24));          /* Argument[31..24] */
	(void)SPI_transfer((BYTE)(arg >> 16));          /* Argument[23..16] */
	(void)SPI_transfer((BYTE)(arg >> 8));           /* Argument[15..8] */
	(void)SPI_transfer((BYTE)arg);                  /* Argument[7..0] */
	n = 0x01;                           /* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;          /* Valid CRC for CMD0(0) + Stop */
	if (cmd == CMD8) n = 0x87;          /* Valid CRC for CMD8(0x1AA) Stop */
	(void)SPI_transfer(n);

	/* Receive command response */
	n = 10;                             /* Wait for a response, try 10 times. */
	do {
		res = SPI_transfer(0xFF);
		n--;
	} while ((res & 0x80) && n);

	return res;
}

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

/**	Initialize disk drive.
	\return 0 on success, else STA_NOINIT.
	Initialize disk drive and also find out which kind of card we're talking to.
	Description about what's going on here see
	http://elm-chan.org/docs/mmc/mmc_e.html, center section.
*/

DSTATUS disk_initialize(void) {
	BYTE n, cmd, ty, ocr[4];
	UINT timeout;

	_SD_NSS_PORT |= (1 << _SD_NSS_PORTPIN);
	/* 80 dummy clocks with CS = high. */
	for (n = 10; n; n--) (void)SPI_transfer(0xFF);

	/* Find card type: MMCv3, SDv1 or SDv2. */
	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Software reset, enter idle state. */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? (rejected on others) */
			for (n = 0; n < 4; n++)			/* Get trailing return value of R7. */
				ocr[n] = SPI_transfer(0xFF);

			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
				/* The card can work at Vdd range of 2.7 - 3.6V. */

				/* Wait for leaving idle state (ACMD41 with HCS bit). */
				for (timeout = 10000; timeout && send_cmd(ACMD41, 1UL << 30); timeout--)
					_delay_us(100);

				/* Find out whether it's a block device (CCS bit in OCR). */
				if (timeout && send_cmd(CMD58, 0) == 0) {
					for (n = 0; n < 4; n++)
						ocr[n] = SPI_transfer(0xFF);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;  /* SDv2 */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) {
				ty = CT_SD1; cmd = ACMD41;  /* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;    /* MMCv3 */
			}

			/* Wait for leaving idle state. */
			for (timeout = 10000; timeout && send_cmd(cmd, 0); timeout--)
				_delay_us(100);

			/* Set R/W block length to 512. */
			if ( ! timeout || send_cmd(CMD16, 512) != 0)
				ty = 0;
		}
	}

	card_type = ty;
	_SD_NSS_PORT |= (1 << _SD_NSS_PORTPIN);
	(void)SPI_transfer(0xFF);

	return ty ? 0 : STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Partial Sector                                                   */
/*-----------------------------------------------------------------------*/

/**	Read partial sector.
	\param buffer  Received data should go in here.
	\param sector  Sector number (LBA).
	\param offset  Offset into the sector.
	\param count   Byte count (bit 15: destination).
	\return RES_OK on success, else RES_ERROR.
	This is the main reading function. Forming this into directory listings,
	file reads and such stuff is done by Petit FatFs it's self.
	Description about what's going on here see
	http://elm-chan.org/docs/mmc/mmc_e.html, bottom section.
*/

DRESULT disk_readp(BYTE* buffer, DWORD sector, UINT offset, UINT count) {
	DRESULT result;
	BYTE token = 0xFF;
	uint16_t timeout;
	UINT trailing = 514 - offset - count;  /* 514 = block size + 2 bytes CRC */

	/* Convert to byte address on non-block cards. */
	if ( ! (card_type & CT_BLOCK)) sector *= 512;

	/* Read one sector, copy only as many bytes as required. */
	result = RES_ERROR;
	if (send_cmd(CMD17, sector) == 0) {  /* Read single block. */
		/* Wait for data packet. */
		for (timeout = 40000; timeout && (token == 0xFF); timeout--)
			token = SPI_transfer(0xFF);

		if (token == 0xFE) {    /* Valid data arrived. */
			/* Skip leading bytes. */
			while (offset--) (void)SPI_transfer(0xFF);

			/* Receive the requested part of the sector. */
			do {
				/**
				Note that this isn't optimised for performance. The increment and
				the dereferencing could be done while waiting for the SPIF bit
				to become set in spi_rw(). See http://www.matuschek.net/atmega-spi/.
				*/
				*buffer++ = SPI_transfer(0xFF);
			} while (--count);

			/* Skip trailing bytes and CRC. */
			while (trailing--) (void)SPI_transfer(0xFF);

			result = RES_OK;
		}
	}

	_SD_NSS_PORT |= (1 << _SD_NSS_PORTPIN);	/* Every send_cmd() selects. */
	(void)SPI_transfer(0xFF);

	return result;
}

/*-----------------------------------------------------------------------*/
/* Write Partial Sector                                                  */
/*-----------------------------------------------------------------------*/

/**	Write partial pector.
	\param buff   Pointer to the data to be written.
	              NULL: Initiate/Finalize write operation.
	\param sc     Sector number (LBA) or number of bytes to send.
	\return RES_OK on success, else RES_ERROR.
	This is the main writing function. Forming this into file writes and such
	stuff is done by Petit FatFs it's self.
	Description about what's going on here see
	http://elm-chan.org/docs/mmc/mmc_e.html, bottom section.
*/

#if PF_USE_WRITE
DRESULT disk_writep (const BYTE *buff, DWORD sc) {
	DRESULT res;
	UINT bc;
	static WORD wc;

	res = RES_ERROR;

	if (buff) {		/* Send data bytes */
		bc = (WORD)sc;
		while (bc && wc) {		/* Send data bytes to the card */
			(void)SPI_transfer(*buff++);
			wc--; bc--;
		}
		res = RES_OK;
	} else {
		if (sc) {	/* Initiate sector write process */
			if (!(card_type & CT_BLOCK)) sc *= 512;	/* Convert to byte address if needed */
			if (send_cmd(CMD24, sc) == 0) {			/* WRITE_SINGLE_BLOCK */
				(void)SPI_transfer(0xFF);
				(void)SPI_transfer(0xFE);			/* Data block header */
				wc = 512;							/* Set byte counter */
				res = RES_OK;
			}
		} else {	/* Finalize sector write process */
			bc = wc + 2;
			while (bc--) (void)SPI_transfer(0x00);	/* Fill left bytes and CRC with zeros */
			if ((SPI_transfer(0xFF) & 0x1F) == 0x05) {	/* Receive data resp and wait for end of write process in timeout of 500ms */
				for (bc = 5000; SPI_transfer(0xFF) != 0xFF && bc; bc--) _delay_us(100);	/* Wait ready */
				if (bc) res = RES_OK;
			}
			_SD_NSS_PORT |= (1 << _SD_NSS_PORTPIN);
			(void)SPI_transfer(0xFF);
		}
	}

	return res;
}
#endif /* PF_USE_WRITE */
