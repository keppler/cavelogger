#ifndef __SSD1306_H
#define __SSD1306_H

enum addr_t { MEM, FLASH, EEPROM };

void SSD1306_init();
void SSD1306_writeImg(uint8_t width, uint8_t height, const uint8_t *img, enum addr_t addrType);
void SSD1306_writeChar(uint8_t x, uint8_t y, uint8_t ch, uint8_t flags);
uint8_t SSD1306_writeString(uint8_t x, uint8_t y, const char *str, uint8_t fromFlash);
void SSD1306_clear(void);
void SSD1306_on(void);
void SSD1306_off(void);
uint8_t SSD1306_writeInt(uint8_t x, uint8_t y, int16_t value, int base);

#endif /* !__SSD1306_H */
