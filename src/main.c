/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * https://github.com/keppler/cavelogger
 * main.c
 * Program entry...
 */

#include "main.h"
#include "pins.h"
#include "LED.h"
#include "RX6110.h"
#include "SPI.h"
#include "SSD1306.h"
#include "../lib/i2cmaster/i2cmaster.h"

#ifdef ENABLE_BMP280
#include "BMP280.h"
#endif /* ENABLE_BMP280 */

#ifdef FLASH_NSS_PORT
#include "AT45DB081E.h"
#endif /* FLASH_NSS_PORT */

int main(void) {
    LED_init();
    i2c_init();
    /* Setup SPI bus
     * Initialize all CS lines before accessing *any* device on the SPI bus! */
    SPI_init();
    RTC_setup();

#ifdef ENABLE_BMP280
    BMP280_setup();
#endif /* ENABLE_BMP280 */

#ifdef FLASH_NSS_PORT
    FLASH_setup();
#endif /* FLASH_NSS_PORT */

    /* SD card */
    DDR(SD_NSS_PORT) |= (1 << PORTPIN(SD_NSS_PORT, SD_NSS_PIN));
    PORT(SD_NSS_PORT) |= (1 << PORTPIN(SD_NSS_PORT, SD_NSS_PIN));	/* set NSS HIGH */
    DDR(SD_CD_PORT) &= ~(1 << PORTPIN(SD_CD_PORT, SD_CD_PIN));		/* configure PC3 as INPUT */
    PORT(SD_CD_PORT) |= (1 << PORTPIN(SD_CD_PORT, SD_CD_PIN));		/* enable internal pull-up */

#ifdef OLED_PWR_PORT
    DDR(OLED_PWR_PORT) |= (1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));
    PORT(OLED_PWR_PORT) |= (1 << PORTPIN(OLED_PWR_PORT, OLED_PWR_PIN));	/* power on */
#endif /* OLED_PWR_PORT */
    DDR(RFM95_NSS_PORT) |= (1 << PORTPIN(RFM95_NSS_PORT, RFM95_NSS_PIN));
    PORT(RFM95_NSS_PORT) |= (1 << PORTPIN(RFM95_NSS_PORT, RFM95_NSS_PIN));	/* set NSS HIGH */

    SSD1306_init();
    SSD1306_clear();	/* clear display */
    SSD1306_on();

    return 0;
}
