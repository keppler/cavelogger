#include "pins.h"
#include <avr/io.h>
#include "SPI.h"

void SPI_init(void) {
	/* Set MOSI and SCK output, set LOW */
//	DDR(SPI_PORT) |= (1 << SPI_MOSI_PIN) | (1 << SPI_SCK_PIN);
//	PORT(SPI_PORT) &= ~((1 << PORTPIN(SPI_PORT, SPI_MOSI_PIN)) | (1 << PORTPIN(SPI_PORT, SPI_SCK_PIN)));
	/* Set MISO to input */
//	DDR(SPI_PORT) &= ~(1 << SPI_MISO_PIN);

	DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2);
	DDRB &= ~(1 << PB4);

	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	//SPCR = (1 << SPE) | (1 << MSTR);
	SPSR = (1 << SPI2X);
}

uint8_t SPI_transfer(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
	return(SPDR);
}
