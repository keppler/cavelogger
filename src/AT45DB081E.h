#ifndef __AT45DB081E_H
#define __AT45DB081E_H

void FLASH_setup(void);
uint8_t FLASH_init(void);	/* 0=ok, 1=error */
void FLASH_read(uint32_t addr, uint8_t len, uint8_t *data);

void FLASH_enter_sleep(void);
void FLASH_leave_sleep(void);

#endif /* !AT45DB081E_H */
