/* PIN configuration */

#ifndef __PINS_H
#define __PINS_H

#define F_CPU 1000000L

#define __CONCAT2(x,y) x ## y
#define __CONCAT3(x,y,z) x ## y ## z
#define DD(port, pin) __CONCAT3(DD, port, pin)
#define DDR(name) __CONCAT2(DDR, name)
#define PORT(name) __CONCAT2(PORT, name)
#define PORTIN(name) __CONCAT2(PIN, name)
#define PORTPIN(port, pin) __CONCAT3(P, port, pin)

/* SPI */
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define SPI_PORT		A
#define SPI_USCK_PIN	4
#define SPI_DO_PIN		5
#define SPI_DI_PIN		6
#elif defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define SPI_PORT		B
#define SPI_USCK_PIN	2
#define SPI_DO_PIN		1
#define SPI_DI_PIN		0
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define SPI_PORT		B
#define SPI_SCK_PIN		5
#define SPI_MOSI_PIN	3
#define SPI_MISO_PIN	4
#else
#error UNKNOWN PLATFORM
#endif

/* I2C */
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
/* use software TWI interface */
#undef I2C_HARDWARE
#define I2C_PORT		C
#define I2C_SDA_PIN		4
#define I2C_SCL_PIN		5
#else
#error UNKNOWN PLATFORM
#endif

/* RFM95 */
#define RFM95_NSS_PORT	D
#define RFM95_NSS_PIN	6
#define RFM95_DIO0_PORT	D
#define RFM95_DIO0_PIN	5
#define RFM95_DIO1_PORT	B
#define RFM95_DIO1_PIN	7

/* BMP280 */
#define BMP280_NSS_PORT	C
#define BMP280_NSS_PIN	0
/* if defined, this pin will enable switching on/off the sensor */
//#define BMP280_PWR_PORT	B
//#define BMP280_PWR_PIN	0

/* Debug LED */
#define LED_PORT	C
#define LED_PIN		1

/* SD Card */
#define SD_NSS_PORT		D
#define SD_NSS_PIN		3
// CD: Card Detected
#define SD_CD_PORT		C
#define SD_CD_PIN		3
// INT: PCINT11 !

/* Real Time Clock */
#define RTC_NSS_PORT	D
#define RTC_NSS_PIN		7

/* SSD1306 OLED display */
#define OLED_PWR_PORT	C
#define OLED_PWR_PIN	2

/* flash memory */
#define FLASH_NSS_PORT	D
#define FLASH_NSS_PIN	1

/* PCB layout patches for 1.0 */
#ifdef PCB_V1_0
/* use PC4 as SCL and PC5 as SDA due to layout bug */
#undef I2C_SDA_PIN
#define I2C_SDA_PIN		5
#undef I2C_SCL_PIN
#define I2C_SCL_PIN		4
#endif /* PCB_V1_0 */

#endif /* __PINS_H */
