#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>

void SPI_init(void);
uint8_t SPI_transfer(uint8_t data);

#endif /* !__SPI_H */
