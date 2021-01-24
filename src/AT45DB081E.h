/*  ___              _
 * / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
 *| (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
 * \___\__,_|\_/\___|____\___/\__, \__, \___|_|
 *                            |___/|___/
 * AT45DB081E.h
 * Interface to adesto AT45DB081E SPI Serial Flash Memory
 */

#ifndef __AT45DB081E_H
#define __AT45DB081E_H

void FLASH_setup(void);
uint8_t FLASH_init(void);	/* 0=ok, 1=error */
void FLASH_read(uint32_t addr, uint16_t len, uint8_t *data);
uint8_t FLASH_loadPage(uint32_t addr);
uint8_t FLASH_writePage(uint32_t addr);
void FLASH_writeBuffer(uint8_t addr, uint8_t len, uint8_t *data);

void FLASH_enter_sleep(void);
void FLASH_leave_sleep(void);

#endif /* !AT45DB081E_H */
