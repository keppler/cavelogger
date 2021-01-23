# CaveLogger

(tbd)

## Data

During measeurement, we collect the following data:

Data | Size
---- | ----
Timestamp | 4 byte
Temperature | 2 byte
Air Pressure | 2 byte
Humidity | 2 byte
Battery Voltage | 2 byte
Wind speed | 1 byte

That's up to 13 byte per measurement. But we don't need the timestamp and the battery voltage every time (it's sufficient to save these e.g. once per hour).

So when we measure data every 15 minutes, we produce *(4 x 7) + 6 = 34 bytes* per hour (= 816 bytes per day, =300kB per year).

## Saving the data

The initial idea was to save data on a (micro) SD card. But during development several problems came up:

- the SD card has a really large peak current during power-on (>500mA) - this can be problematic on low battery
- the SD card usually requires a minimum voltage of 2.7V. When we plan to run on battery "as long as possible" (e.g. around 1.7V) we won't be able to write any more
- there's a really annoying leak current of around 50uA, even when switching off GND via a N-Ch MOSFET (e.g. BS170)

So we now use a dedicated NOR-flash memory chip (AT45DB081E-SHN). It only draws 400nA (!) during deep sleep, runs with 1.7-3.6V, has no mechanical interfaces (think of corrosion!) and so on.

The capacity of this flash memory is 8 MBit = 1 MB, so we can save data for more than three years. Data retention is specified with 20 years in industrial environments (-40 to +85 °C).

The SD Card connector anyway remains. If a card insertion is detected (this is signalled by connecting a pin on the connector to GND), we run a function to mount that SD card and copy the contents of the flash memory to the SD card. An "empty" connector doesn't use any power.

## Battery lifetime

The circuit uses around 1.5μA during idle time. The duty cycle draws around 5mA for around 5 seconds. Most power is drawn when sending data via LoRaWAN, we estimate 20mA for 1 second (###TODO###).
So within a 15 minute loop, we draw:

- idle: 15min * 1.5μA = 0.375μAh
- power-on: 5sec * 5mA = 6.95μAh
- sending: 1sec * 20mA = 5.56μAh
- total: 12.9μAh = **113mAh per year**

*(missing: power can be saved by only sending data every 2 or 4 measurements; writing data to the flash chip requires some additional power once or twice a day - depending on how often data is written)*

When we use 2 AA batteries (to get a voltage of 3.0V) we should have at least 1.500-2.000mAh available - more than enough for some years, even in a cold cave. Self-discharge is propably a bigger problem, so we should prefer industrial-grade cells.

A CR2032 is used as backup battery for the real-time clock. The flash memory doesn't need power to retain its data.

## Parts

Name | Description | Price | Datasheet
---- | ----------- | ----- | ---------
AT45DB081E-SHN | NOR Flash Memory, 8 MBit, SPI, 1.7V | [1,75 €](https://www.reichelt.de/de/de/nor-flash-speicher-8mb-1-7v-seriell-spi-85mhz-sol-8-at45db081e-shn-p266509.html) | [Datasheet](https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=A300%2FAT45DB081E-XXX.pdf)
EPSON RX6110SA B | Real Time Clock (RTC) with SPI, ultra low power (130nA) | [1,99 €](https://www.reichelt.de/serial-interface-rtc-module-5-0ppm-i-c-spi-sop-14-rx6110sa-b-p262533.html) | [Datasheet](https://www.reichelt.de/index.html?ACTION=7&LA=3&OPEN=0&INDEX=0&FILENAME=A300%2FRX6110SAB.pdf)
ATmega 328P-PU | 8 Bit AVR Microcontroller, 32kb flash | [2,25 €](https://www.reichelt.de/mcu-atmega-avr-risc-32-kb-20-mhz-pdip-28-atmega-328p-pu-p119685.html?r=1) | [Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
BMP280 | Sensor for temperature and pressure. Check for SPI pins on the breakout board! | [1,15 €](https://www.reichelt.de/entwicklerboards-temperatur-und-drucksensor-bmp280-debo-bmp280-p266034.html?&trstct=pos_0) | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/environmental_sensors_2/pressure_sensors_1/bmp280/bst-bmp280-ds001.pdf)
HopeRF RFM95W | LoRa transceiver, based on SX1276 chipset (868MHz version for EU) | 6-10 € | [Website](https://www.hoperf.com/modules/lora/RFM95.html) [Datasheet](https://www.hoperf.com/data/upload/portal/20190801/RFM95W-V2.0.pdf)
SSD1306 | OLED display 128x64 px, I²C (for debugging/development only) | ~2 € | -
Wind sensor | Replacement part for anemometer | [20 €](https://www.froggit.de/product_info.php?language=de&info=p138_ersatz-windgeschwindigkeitssensor-fuer-wh1080-wh3080-1090-wh4000.html) | -

## Pin configuration

```
                         _______
        [RESET] PC6   1 |  ( )  | 28  PC5 TWI (I²C) SCL
                PD0   2 |       | 27  PC4 TWI (I²C) SDA
   FLASH SPI CS PD1   3 |       | 26  PC3
INT0 (RTC INT1) PD2   4 |       | 25  PC2 OLED POWER
        WIND IN PD3   5 |       | 24  PC1 LED
       WIND OUT PD4   6 |       | 23  PC0 BME280 SPI CS
                VCC   7 |       | 22  GND
                GND   8 |       | 21  AREF
                PB6   9 |       | 20  AVCC
     RFM95 DIO1 PB7  10 |       | 19  PB5 SPI SCK
     RFM95 DIO0 PD5  11 |       | 18  PB4 SPI MISO
   RFM95 SPI CS PD6  12 |       | 17  PB3 SPI MOSI
     RTC SPI CS PD7  13 |       | 16  PB2
       Button_1 PB0  14 |_______| 15  PB1 Button_2
```

## Software

### Flashing

You need to flash the circuit with **3.3V** (the RFM95 will be killed with 5V). I recommend to [modify an USBASP](https://www.hackster.io/billy-cheung/3-3v-usbasp-modification-c20557) to work with 3.3V output.

### Libraries

All required libraries are included. The following adjustments have been made to the original source:

1. [Petit FAT File System](http://elm-chan.org/fsw/ff/00index_p.html)

   -  adjusted `pffconf.h` to disable all features except `PF_USE_WRITE` and `PF_FS_FAT32`

2. [LMIC library](https://github.com/mcci-catena/arduino-lmic)

   -  defined all configuration settings in `lmic-config.h` (instead of `project_config/lmic_project_config.h`), disabling all features not required
   -  ported hardware driver (HAL) to AVR (`lmic-hal-avr.c`) to be independend of Arduino (a bit "quick&dirty", highly depending on current chipset and layout)

3. [I2C Master Interface](http://www.peterfleury.epizy.com/avr-software.html) by Peter Fleury

   Due to a bug in the PCB layout (see #1) we use the software I²C implementation. Next PCB version will use `twimaster.c` instead (with ATmega's hardware I²C interface).

## Literatur/Links (German)

* [Das Klima der Schrattenhöhle](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/147894/eth-26985-01.pdf?sequence=1&isAllowed=y) - äußerst interessante wissenschaftliche Arbeit zur Klimamessung in Höhlen
* [Hitzedraht-Anemometer](https://www.mikrocontroller.net/attachment/58437/ELV_Hitzdraht-Anemometer_3.._2_.pdf) (mit Schaltplan)
* [Umbau Windrad von Reed auf Hallsensoren](https://www.arduinoforum.de/arduino-Thread-Anemometer-WH1080)
