#ifndef __BMP280_H
#define __BMP280_H

enum BMP280_TYPE_T {
	BMP280_TYPE_UNKNOWN	= 0,
	BMP280_TYPE_BMP280	= 1,
	BMP280_TYPE_BME280	= 2
};

void BMP280_setup(void);
enum BMP280_TYPE_T BMP280_init(void);
int32_t BMP280_temp(void);
uint32_t BMP280_pressure(void);
uint32_t BME280_humidity(void);			/* watch the modified prefix! "BME_" ! */

#ifdef BMP280_PWR_PORT
void BMP280_on(void);
void BMP280_off(void);
#endif /* BMP280_PWR_PORT */

#endif /* !__BMP280_H */
