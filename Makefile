GCCFLAGS=-g -O3 -mmcu=atmega8a -std=gnu99 -Wall -Wextra -pedantic
LINKFLAGS=-lm
AVRDUDEFLAGS=-B 5 -c usbasp -p m8

all:    main-upload

main.hex:  main.c
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o main.o main.c
	avr-objcopy -O ihex -R .eeprom main.o main.hex

main.ass:  main.hex
	avr-objdump -S -d main.o > main.ass

main-upload:       main.hex
	sudo avrdude ${AVRDUDEFLAGS} -U flash:w:main.hex:a

fuses:
	sudo avrdude -c usbasp -p m8 -U lfuse:w:0xff:m -U hfuse:w:0xC9:m

