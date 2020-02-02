#define F_CPU 8000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include <stdio.h>
#include <avr/pgmspace.h> // PSTR

#define  INPUT2(port,pin)   DDR ## port &= ~_BV(pin)
#define OUTPUT2(port,pin)   DDR ## port |=  _BV(pin)
#define  CLEAR2(port,pin)  PORT ## port &= ~_BV(pin)
#define    SET2(port,pin)  PORT ## port |=  _BV(pin)
#define   READ2(port,pin) ((PIN ## port & _BV(pin))?1:0)

#define  INPUT(x)  INPUT2(x)
#define OUTPUT(x) OUTPUT2(x)
#define  CLEAR(x)  CLEAR2(x)
#define    SET(x)    SET2(x)
#define   READ(x)   READ2(x)
#define  WRITE(x,b) ((b)?(SET2(x)):(CLEAR2(x)))

#define TEST_PIN                C,5

int main(void) {
    OUTPUT(TEST_PIN);

    while (1) {
        _delay_ms(10);
        CLEAR(TEST_PIN);
        _delay_ms(10);
        SET(TEST_PIN);
    }

    return 0;
}

