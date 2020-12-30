#include "pins.h"
#include <avr/io.h>
#include <util/delay.h>
#include "SPI.h"
#include "BMP280.h"

#include "LED.h"

#ifndef BMP280_NSS_PORT
#error BMP280_NSS_PORT not defined. Please edit 'pins.h' accordingly.
#endif

/* some shortcuts for better readability */
#define _BMP280_NSS_DDR		DDR(BMP280_NSS_PORT)
#define _BMP280_NSS_PORT	PORT(BMP280_NSS_PORT)
#define _BMP280_NSS_PORTPIN	PORTPIN(BMP280_NSS_PORT, BMP280_NSS_PIN)

#ifdef BMP280_PWR_PORT
#define _BMP280_PWR_DDR		DDR(BMP280_PWR_PORT)
#define _BMP280_PWR_PORT	PORT(BMP280_PWR_PORT)
#define _BMP280_PWR_PORTPIN	PORTPIN(BMP280_PWR_PORT, BMP280_PWR_PIN)
#endif

/* Calibration parameter register addresses */
#define BMP280_DIG_T1_LSB_ADDR	0x88
#define BMP280_DIG_T1_MSB_ADDR	0x89
#define BMP280_DIG_T2_LSB_ADDR	0x8A
#define BMP280_DIG_T2_MSB_ADDR	0x8B
#define BMP280_DIG_T3_LSB_ADDR	0x8C
#define BMP280_DIG_T3_MSB_ADDR	0x8D
#define BMP280_DIG_P1_LSB_ADDR	0x8E
#define BMP280_DIG_P1_MSB_ADDR	0x8F
#define BMP280_DIG_P2_LSB_ADDR	0x90
#define BMP280_DIG_P2_MSB_ADDR	0x91
#define BMP280_DIG_P3_LSB_ADDR	0x92
#define BMP280_DIG_P3_MSB_ADDR	0x93
#define BMP280_DIG_P4_LSB_ADDR	0x94
#define BMP280_DIG_P4_MSB_ADDR	0x95
#define BMP280_DIG_P5_LSB_ADDR	0x96
#define BMP280_DIG_P5_MSB_ADDR	0x97
#define BMP280_DIG_P6_LSB_ADDR	0x98
#define BMP280_DIG_P6_MSB_ADDR	0x99
#define BMP280_DIG_P7_LSB_ADDR	0x9A
#define BMP280_DIG_P7_MSB_ADDR	0x9B
#define BMP280_DIG_P8_LSB_ADDR	0x9C
#define BMP280_DIG_P8_MSB_ADDR	0x9D
#define BMP280_DIG_P9_LSB_ADDR	0x9E
#define BMP280_DIG_P9_MSB_ADDR	0x9F
#define BME280_DIG_H1_ADDR		0xA1
#define BME280_DIG_H2_LSB_ADDR	0xE1
#define BME280_DIG_H2_MSB_ADDR	0xE2
#define BME280_DIG_H3_ADDR		0xE3
#define BME280_DIG_H4_MSB_ADDR	0xE4
#define BME280_DIG_H45_LSB_ADDR	0xE5
#define BME280_DIG_H5_MSB_ADDR	0xE6
#define BME280_DIG_H6_ADDR		0xE7

/* Other registers */
#define BMP280_CHIP_ID_ADDR		0xD0
#define BMP280_SOFT_RESET_ADDR	0xE0
#define BME280_CTRL_HUM_ADDR	0xF2
#define BMP280_STATUS_ADDR		0xF3
#define BMP280_CTRL_MEAS_ADDR	0xF4
#define BMP280_CONFIG_ADDR		0xF5
#define BMP280_PRES_MSB_ADDR	0xF7
#define BMP280_PRES_LSB_ADDR	0xF8
#define BMP280_PRES_XLSB_ADDR	0xF9
#define BMP280_TEMP_MSB_ADDR	0xFA
#define BMP280_TEMP_LSB_ADDR	0xFB
#define BMP280_TEMP_XLSB_ADDR	0xFC
#define BME280_HUM_MSB_ADDR		0xFD
#define BME280_HUM_LSB_ADDR		0xFE

/* ODR options */
#define BMP280_ODR_0_5_MS		0x00
#define BMP280_ODR_62_5_MS		0x01
#define BMP280_ODR_125_MS		0x02
#define BMP280_ODR_250_MS		0x03
#define BMP280_ODR_500_MS		0x04
#define BMP280_ODR_1000_MS		0x05
#define BMP280_ODR_2000_MS		0x06
#define BMP280_ODR_4000_MS		0x07

/* Over-sampling macros */
#define BMP280_OS_NONE		0x00
#define BMP280_OS_1X		0x01
#define BMP280_OS_2X		0x02
#define BMP280_OS_4X		0x03
#define BMP280_OS_8X		0x04
#define BMP280_OS_16X		0x05

/* Power modes */
#define BMP280_SLEEP_MODE	0x00
#define BMP280_FORCED_MODE	0x01
#define BMP280_NORMAL_MODE	0x03

/* Filter coefficient macros */
#define BMP280_FILTER_OFF		0x00
#define BMP280_FILTER_COEFF_2	0x01
#define BMP280_FILTER_COEFF_4	0x02
#define BMP280_FILTER_COEFF_8	0x03
#define BMP280_FILTER_COEFF_16	0x04

/* SPI 3-Wire macros */
#define BMP280_SPI3_WIRE_ENABLE		1
#define BMP280_SPI3_WIRE_DISABLE	0

/* valid chip IDs */
#define BMP280_CHIP_ID1		0x56	/* BMP280 samples */
#define BMP280_CHIP_ID2		0x57	/* BMP280 samples */
#define BMP280_CHIP_ID3		0x58	/* BMP280 mass production */
#define BMP280_CHIP_BME280	0x60	/* BME280 */

/* Soft reset command */
#define BMP280_SOFT_RESET_CMD	0xB6

/* calibration data */
static uint16_t dig_t1;
static int16_t dig_t2;
static int16_t dig_t3;
static uint16_t dig_p1;
static int16_t dig_p2;
static int16_t dig_p3;
static int16_t dig_p4;
static int16_t dig_p5;
static int16_t dig_p6;
static int16_t dig_p7;
static int16_t dig_p8;
static int16_t dig_p9;
static uint8_t dig_h1;
static int16_t dig_h2;
static uint8_t dig_h3;
static int16_t dig_h4;
static int16_t dig_h5;
static int8_t dig_h6;
static int32_t t_fine;

static enum BMP280_TYPE_T _chip_type = BMP280_TYPE_UNKNOWN;

void _BMP280_write(uint8_t addr, uint8_t data) {
	/* set NSS pin LOW to start communication */
	_BMP280_NSS_PORT &= ~(1 << _BMP280_NSS_PORTPIN);

	/* send address with MSB 0 to make it a WRITE command */
	(void)SPI_transfer(addr & 0x7F);
	/* send data */
	(void)SPI_transfer(data);

	/* set NSS pin HIGH to end communication */
	_BMP280_NSS_PORT |= (1 << _BMP280_NSS_PORTPIN);
}

static uint8_t _BMP280_read(uint8_t addr) {
	uint8_t data;

	/* set NSS pin LOW to start communication */
	_BMP280_NSS_PORT &= ~(1 << _BMP280_NSS_PORTPIN);

	/* send address with MSB 1 to make it a READ command */
	(void)SPI_transfer(addr | 0x80);

	/* read data */
	data = SPI_transfer(0x00);

	/* set NSS pin HIGH to end communication */
	_BMP280_NSS_PORT |= (1 << _BMP280_NSS_PORTPIN);

	return(data);
}

void BMP280_setup() {
	/* configure OUTPUT pins */
	_BMP280_NSS_DDR |= (1 << _BMP280_NSS_PORTPIN);
	_BMP280_NSS_PORT |= (1 << _BMP280_NSS_PORTPIN);	/* set NSS HIGH */

#ifdef BMP280_PWR_PORT
	_BMP280_PWR_DDR |= (1 << _BMP280_PWR_PORTPIN);
	_BMP280_PWR_PORT |= (1 << _BMP280_PWR_PORTPIN);	/* set PWR HIGH */
#endif /* BMP280_PWR_PORT */
}

enum BMP280_TYPE_T BMP280_init() {
	uint8_t lsb, msb;

	/* As per the datasheet, startup time is 2 ms. */
	_delay_ms(2);

	/* wait until NVM data has been copied to image registers */
	while (_BMP280_read(BMP280_STATUS_ADDR) & 0x01) _delay_ms(1);

	/* bits 7..5 (t_sb): standby time in normal mode
	 * bits 4..2 (filter): IIR filter constant
	 * bits 1..0 (spi3w_en): enable 3 wire SPI interface */
	lsb = (BMP280_ODR_0_5_MS << 5) | (BMP280_FILTER_OFF << 2) | (BMP280_SPI3_WIRE_DISABLE);
	_BMP280_write(BMP280_CONFIG_ADDR, lsb);

	/* read calibration data */
	lsb = _BMP280_read(BMP280_DIG_T1_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_T1_MSB_ADDR);
	dig_t1 = (uint16_t) (((uint16_t) msb << 8) | ((uint16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_T2_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_T2_MSB_ADDR);
	dig_t2 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_T3_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_T3_MSB_ADDR);
	dig_t3 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P1_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P1_MSB_ADDR);
	dig_p1 = (uint16_t) (((uint16_t) msb << 8) | ((uint16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P2_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P2_MSB_ADDR);
	dig_p2 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P3_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P3_MSB_ADDR);
	dig_p3 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P4_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P4_MSB_ADDR);
	dig_p4 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P5_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P5_MSB_ADDR);
	dig_p5 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P6_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P6_MSB_ADDR);
	dig_p6 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P7_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P7_MSB_ADDR);
	dig_p7 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P8_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P8_MSB_ADDR);
	dig_p8 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_DIG_P9_LSB_ADDR);
	msb = _BMP280_read(BMP280_DIG_P9_MSB_ADDR);
	dig_p9 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));

	lsb = _BMP280_read(BMP280_CHIP_ID_ADDR);

	_BMP280_write(BMP280_CTRL_MEAS_ADDR, BMP280_SLEEP_MODE);

	if (lsb == BMP280_CHIP_ID1 || lsb == BMP280_CHIP_ID2 || lsb == BMP280_CHIP_ID3) {
		_chip_type = BMP280_TYPE_BMP280;
	} else if (lsb == BMP280_CHIP_BME280) {
		_chip_type = BMP280_TYPE_BME280;
		/* read compensation values for humidity */
		dig_h1 = _BMP280_read(BME280_DIG_H1_ADDR);
		lsb = _BMP280_read(BME280_DIG_H2_LSB_ADDR);
		msb = _BMP280_read(BME280_DIG_H2_MSB_ADDR);
		dig_h2 = (int16_t) (((int16_t) msb << 8) | ((int16_t) lsb));
		dig_h3 = _BMP280_read(BME280_DIG_H3_ADDR);
		lsb = _BMP280_read(BME280_DIG_H45_LSB_ADDR);
		msb = _BMP280_read(BME280_DIG_H4_MSB_ADDR);
		dig_h4 = (int16_t) (((int16_t) msb << 4) | ((int16_t) lsb & 0x0F));
		msb = _BMP280_read(BME280_DIG_H5_MSB_ADDR);
		dig_h5 = (int16_t) (((int16_t) msb << 4) | ((int16_t) (lsb >> 4) & 0x0F));
		dig_h6 = _BMP280_read(BME280_DIG_H6_ADDR);

	} else {
		_chip_type = BMP280_TYPE_UNKNOWN;
	}

	return(_chip_type);

}

int32_t BMP280_temp() {
	uint8_t u;

	if (_chip_type == BMP280_TYPE_BME280) {
		/* configure oversampling for humidity */
		u = BMP280_OS_1X;
		_BMP280_write(BME280_CTRL_HUM_ADDR, u);
	}

	/* configure oversampling, set forced mode */
	/* bits 7..5: temperature oversampling: 1 = 1x
	 * bits 4..2: pressure oversampling: 1 = 1x
	 * bits 1..0: power mode: 01 = forced mode */
	u = (BMP280_OS_1X << 5) | (BMP280_OS_1X << 2) | (BMP280_FORCED_MODE);
	_BMP280_write(BMP280_CTRL_MEAS_ADDR, u);

	/* wait until measurement is done */
	/* this value depends on the oversampling values, see specification 3.8.1 (Measurement time) */
	_delay_ms(12);

	uint8_t temp_msb	= _BMP280_read(BMP280_TEMP_MSB_ADDR);
	uint8_t temp_lsb	= _BMP280_read(BMP280_TEMP_LSB_ADDR);
	uint8_t temp_xlsb	= _BMP280_read(BMP280_TEMP_XLSB_ADDR);
	int32_t uncomp_temp = (int32_t) ((((int32_t) (temp_msb)) << 12) |
									 (((int32_t) (temp_lsb)) << 4) |
									 (((int32_t) (temp_xlsb)) >> 4));

	int32_t var1, var2;
	var1 = ((((uncomp_temp >> 3) - ((int32_t) dig_t1 << 1))) * ((int32_t) dig_t2)) >> 11;
	var2 = (((((uncomp_temp >> 4) - ((int32_t) dig_t1)) * ((uncomp_temp >> 4) - ((int32_t) dig_t1))) >> 12) * ((int32_t) dig_t3)) >> 16;
	t_fine = var1 + var2;
	return((t_fine * 5 + 128) >> 8);
}

uint32_t BMP280_pressure() {
	uint8_t press_msb	= _BMP280_read(BMP280_PRES_MSB_ADDR);
	uint8_t press_lsb	= _BMP280_read(BMP280_PRES_LSB_ADDR);
	uint8_t press_xlsb	= _BMP280_read(BMP280_PRES_XLSB_ADDR);
	uint32_t uncomp_pres = (uint32_t) ((((uint32_t) (press_msb)) << 12) |
									   (((uint32_t) (press_lsb)) << 4) |
									   (((uint32_t) (press_xlsb)) >> 4));
	/* calculate compensated pressure */
	uint32_t comp_pres;
	int32_t var1, var2;
	var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) dig_p6);
	var2 = var2 + ((var1 * ((int32_t) dig_p5)) << 1);
	var2 = (var2 >> 2) + (((int32_t) dig_p4) << 16);
	var1 = (((dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) dig_p2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t) dig_p1)) >> 15);

	/* Avoid exception caused by division with zero */
	if (var1 == 0) return(0);

	/* Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1 */
	comp_pres = (uint32_t)(((int32_t)(1048576 - uncomp_pres) - (var2 >> 12)) * 3125);
	if (comp_pres < 0x80000000) {
		comp_pres = (comp_pres << 1) / ((uint32_t) var1);
	} else {
		comp_pres = (comp_pres / (uint32_t) var1) << 1;
	}
	var1 = (((int32_t) dig_p9) * ((int32_t) (((comp_pres >> 3) * (comp_pres >> 3)) >> 13))) >> 12;
	var2 = (((int32_t) (comp_pres >> 2)) * ((int32_t) dig_p8)) >> 13;
	comp_pres = (uint32_t) ((int32_t) comp_pres + ((var1 + var2 + dig_p7) >> 4));
	return(comp_pres);
}

uint32_t BME280_humidity() {

	if (_chip_type != BMP280_TYPE_BME280) return(-1);

	uint8_t hum_msb	= _BMP280_read(BME280_HUM_MSB_ADDR);
	uint8_t hum_lsb	= _BMP280_read(BME280_HUM_LSB_ADDR);
	int32_t uncomp_hum = (int32_t) ((((uint32_t) (hum_msb)) << 8) |
									(((uint32_t) (hum_lsb))));

	int32_t var1;

	var1 = (t_fine - ((int32_t)76800));
	var1 = ((((uncomp_hum << 14) - (((int32_t)dig_h4) << 20) -
			(((int32_t)dig_h5) * var1)) + ((int32_t)16384)) >> 15) *
			(((((((var1 * ((int32_t)dig_h6)) >> 10) * (((var1 *
			((int32_t)dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
			((int32_t)2097152)) * ((int32_t)dig_h2) + 8192) >> 14);
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) *
		((int32_t)dig_h1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	var1 = (uint32_t)(var1 >> 12);

	return(uint32_t)(var1);
}


#ifdef BMP280_PWR_PORT
void BMP280_on() {
	/* power-on */
	_BMP280_NSS_PORT |= (1 << _BMP280_NSS_PORTPIN);	/* set NSS HIGH */
	_BMP280_PWR_PORT |= (1 << _BMP280_PWR_PORTPIN);	/* set PWR HIGH */

	/* As per the datasheet, startup time is 2 ms. */
	_delay_ms(2);

	/* wait until NVM data has been copied to image registers */
	while (_BMP280_read(BMP280_STATUS_ADDR) & 0x01) _delay_ms(1);

	/* bits 7..5 (t_sb): standby time in normal mode
	 * bits 4..2 (filter): IIR filter constant
	 * bits 1..0 (spi3w_en): enable 3 wire SPI interface */
	lsb = (BMP280_ODR_0_5_MS << 5) | (BMP280_FILTER_OFF << 2) | (BMP280_SPI3_WIRE_DISABLE);
	_BMP280_write(BMP280_CONFIG_ADDR, lsb);

}

void BMP280_off() {
	/* just power-off */
	_BMP280_NSS_PORT &= ~(1 << _BMP280_NSS_PORTPIN);	/* set NSS HIGH */
	_BMP280_PWR_PORT &= ~(1 << _BMP280_PWR_PORTPIN);	/* set PWR LOW */	
}

#endif /* BMP280_PWR_PORT */
