#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>

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

#define TEST_PIN       C,5
#define SERVO_R_PIN    B,2

// The servos take a 50 Hz PWM signal; 1 ms minimum pulse width (0 deg), 2 ms maximum pulse width (180 deg)

int main(void) {
    OUTPUT(TEST_PIN);
    OUTPUT(SERVO_R_PIN);

    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // FastPWM Mode mode TOP determined by ICR1 - non-inverting Compare Output mode
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);    // set prescaler to 8, FastPWM Mode mode continued
    ICR1 = 20000;      // set period to 20 ms
    OCR1A = 1000;      // set count to 1500 us - 90 degree
    OCR1B = 1000;      // set count to 1500 us - 90 degree
    TCNT1 = 0;         // reset timer

    uint8_t up = 1;

    while (1) {
        _delay_ms(10);
        CLEAR(TEST_PIN);
        _delay_ms(10);
        SET(TEST_PIN);
        if (up  && OCR1B>=2000) up = 0;
        if (!up && OCR1B<=1000) up = 1;
        OCR1B += (up?4:-4);
    }

    return 0;
}

