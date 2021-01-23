#   ___              _
#  / __|__ ___ _____| |   ___  __ _ __ _ ___ _ _
# | (__/ _` \ V / -_) |__/ _ \/ _` / _` / -_) '_|
# \___\__,_|\_/\___|____\___/\__, \__, \___|_|
#                             |___/|___/
# Makefile

# Flashing:
# avrdude -c usbasp -p atmega328p -P usb -v -U lfuse:w:0x62:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m -U flash:w:cavelogger.hex:i
# avrdude -c usbasp -p atmega328p -P usb -v -U lfuse:w:0x62:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m -U eeprom:w:cavelogger.eep:i

CFLAGS = -Wall -Os -mmcu=atmega328p -fdata-sections -ffunction-sections
LDFLAGS = -Wl,--gc-sections
CXXFLAGS = -Wall -Os -mmcu=atmega328p -std=c++11
CPPFLAGS = -DARDUINO_LMIC_PROJECT_CONFIG_H=../../../src/lmic-config.h -DUSE_IDEETRON_AES -DPCB_V1_0

SOURCES_C = \
		lmic-hal-avr.c \
		../lib/arduino-lmic/lmic/lmic.c \
		../lib/arduino-lmic/lmic/lmic_eu868.c \
		../lib/arduino-lmic/lmic/lmic_eu_like.c \
		../lib/arduino-lmic/lmic/oslmic.c \
		../lib/arduino-lmic/aes/other.c \
		../lib/arduino-lmic/lmic/radio.c \
		../lib/i2cmaster/twimaster.c \
		../lib/petitfat/diskio.c \
		../lib/petitfat/pff.c \
		AT45DB081E.c \
		BMP280.c \
		LED.c \
		RX6110.c \
		SPI.c \
		SSD1306.c \
		VCC.c \
		Wind.c \
		main.c \
		menu.c

SOURCES_CPP = \
		../lib/arduino-lmic/aes/ideetron/AES-128_V10.cpp

SOURCES_ASM= \
		../lib/i2cmaster/i2cmaster.S

OBJECTS = $(SOURCES_C:%.c=%.o) $(SOURCES_ASM:%.S=%.o) $(SOURCES_CPP:%.cpp=%.o)

.c.o .S.o:
	avr-gcc -c $(CFLAGS) $(CPPFLAGS) $(INCLUDE) -o $@ $<

.cpp.o:
	avr-g++ -c $(CXXFLAGS) $(CPPFLAGS) $(INCLUDE) -o $@ $<

all: cavelogger.hex cavelogger.eep

# Include dependencies (.depend) if existing...
ifeq (.depend,$(wildcard .depend))
include .depend
endif

cavelogger.hex: cavelogger.elf
	avr-objcopy -R .eeprom -j .text -j .data -O ihex $< $@
	avr-size $@

cavelogger.eep: cavelogger.elf
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $< $@
	avr-size $@

cavelogger.elf: .depend $(OBJECTS)
	avr-gcc $(CFLAGS) $(LDFLAGS) -o $@ $(OBJECTS)

clean:
	rm -f $(OBJECTS) .depend cavelogger.elf cavelogger.hex

dep .depend:
	@echo Making dependencies...
	avr-gcc -MM -I. $(CFLAGS) $(CPPFLAGS) $(INCLUDE) $(SOURCES_C) > .depend
