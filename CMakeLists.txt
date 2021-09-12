# thanks to https://www.kuon.ch/post/2018-07-11-avr-cmake/ for important hints! :)
cmake_minimum_required(VERSION 3.11)

project("CaveLogger")
enable_language(C ASM)

set(F_CPU 1000000UL)
set(MCU atmega328p)

# Use AVR GCC toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)

# Pass defines to compiler
add_definitions(
        -DF_CPU=${F_CPU}
        -DPCB_V1_0 # workaround for PCB v1.0 layout errors
)

# mmcu MUST be passed to bot the compiler and linker, this handle the linker
set(CMAKE_EXE_LINKER_FLAGS -mmcu=${MCU})

add_compile_options(
        -mmcu=${MCU} # MCU
        -std=gnu99 # C99 standard
        -Os # optimize
        -Wall # enable warnings
        -Wno-main
        -Wundef
        -pedantic
        -Wstrict-prototypes
        -Werror
        -Wfatal-errors
        -Wl,--relax,--gc-sections
        -g
        -gdwarf-2
        -funsigned-char # a few optimizations
        -funsigned-bitfields
        -fpack-struct
        -fshort-enums
        -ffunction-sections
        -fdata-sections
        -fno-split-wide-types
        -fno-tree-scev-cprop
)

# Create one target
add_executable(CaveLogger
        src/main.c
        src/AT45DB081E.c
        src/BMP280.c
        src/LED.c
        src/RX6110.c
        src/SPI.c
        src/SSD1306.c
        lib/i2cmaster/i2cmaster.S
        )

set_target_properties(CaveLogger PROPERTIES OUTPUT_NAME cavelogger.elf)

# Strip binary for upload
add_custom_target(strip ALL avr-strip cavelogger.elf DEPENDS CaveLogger)

# Transform binary into hex file, we ignore the eeprom segments in the step
add_custom_target(hex ALL avr-objcopy -R .eeprom -O ihex cavelogger.elf cavelogger.hex DEPENDS strip)

# Transform binary into hex file, this is the eeprom part (empty if you don't
# use eeprom static variables)
add_custom_target(eeprom avr-objcopy -j .eeprom  --set-section-flags=.eeprom="alloc,load"  --change-section-lma .eeprom=0 -O ihex cavelogger.elf cavelogger.eep DEPENDS strip)

# Clean extra files
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "cavelogger.hex;cavelogger.eeprom")
