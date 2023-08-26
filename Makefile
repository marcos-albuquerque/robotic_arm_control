MCU=atmega2560
F_CPU=16000000
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU}
TARGET=main
SRCS=main.c
PORT=/dev/ttyACM0

all:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
	${OBJCOPY} -R .eeprom -O ihex ${TARGET}.bin ${TARGET}.hex

test:
	avrdude -v -p ${MCU} -c wiring -P ${PORT}

flash:
	avrdude -v -p ${MCU} -c wiring -U flash:w:${TARGET}.hex:i -P ${PORT} -D

clean:
	rm -f *.bin *.hex
