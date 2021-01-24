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

#define ENABLE_RFM95
#define ENABLE_BMP280

#define CL_VERSION "1.5"
#define CL_DATE "01/2021"

extern uint16_t int0Count;
extern volatile uint8_t inputButton;
extern uint8_t cfgLoraOff;
extern uint8_t cfgLoraSF;
extern uint16_t cfgMeasIntv;	/* measurement interval */
extern uint32_t flashBlock;
extern uint32_t flashRec;

/* recent measurements */
extern uint32_t measCount;
extern int32_t measTemp;
extern uint16_t measHum;
extern uint16_t measPress;
extern uint16_t measVCC;
extern uint16_t measWind;
extern time_t measTsFirst;
extern time_t measTsLast;

/* 1 MB: 2048 blocks with 512 bytes each
 * every 512 byte block has a 9 byte header (data_header) + 38 data records
 * This way we have a "natural" boundary of 256 byte (hdr + 19 data records) which
 * is important because flash memory page size is 256 byte!
 */
struct data_header {
	uint8_t magic[3];		/* CLG */
	uint8_t version;		/* 1 */
	uint16_t nrec;			/* number of record in this block; XXX = full */
	uint8_t _reserved[3];
};

struct data_record {
	uint32_t ts;
	uint16_t temp;
	uint16_t pressure;
	uint16_t humidity;
	uint16_t voltage;
	uint8_t wind;
};

#endif /* !__MAIN_H */
