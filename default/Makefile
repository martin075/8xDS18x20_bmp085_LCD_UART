###############################################################################
# Makefile for the project 8xDS18x20_BMP180_LCD_UART
###############################################################################

## General Flags
PROJECT = 8xDS18x20_BMP180_LCD_UART
MCU = atmega168
TARGET = 8xDS18x20_BMP180_LCD_UART.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99        -DF_CPU=8000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=8xDS18x20_BMP180_LCD_UART.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = main.o crc.o delay.o ds18x20.o lcd.o owi_highlevel.o owi_lowlevel.o uart.o ds2415.o bmp085.o twimaster.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) 8xDS18x20_BMP180_LCD_UART.hex 8xDS18x20_BMP180_LCD_UART.eep 8xDS18x20_BMP180_LCD_UART.lss size

## Compile
main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

crc.o: ../crc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

delay.o: ../delay.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ds18x20.o: ../ds18x20.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

lcd.o: ../lcd.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

owi_highlevel.o: ../owi_highlevel.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

owi_lowlevel.o: ../owi_lowlevel.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart.o: ../uart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ds2415.o: ../ds2415.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

bmp085.o: ../bmp085.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

twimaster.o: ../twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) 8xDS18x20_BMP180_LCD_UART.elf dep/* 8xDS18x20_BMP180_LCD_UART.hex 8xDS18x20_BMP180_LCD_UART.eep 8xDS18x20_BMP180_LCD_UART.lss 8xDS18x20_BMP180_LCD_UART.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

