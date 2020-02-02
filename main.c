#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


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
#define SERVO_L_PIN    B,1
#define SERVO_R_PIN    B,2
#define SERVO_C_PIN    B,3

// The servos take a 50 Hz PWM signal; 1 ms minimum pulse width (0 deg), 2 ms maximum pulse width (180 deg)



int main(void) {
    OUTPUT(TEST_PIN);
    OUTPUT(SERVO_L_PIN);
    OUTPUT(SERVO_R_PIN);
    OUTPUT(SERVO_C_PIN);

    { // init timer1
        TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // fast PWM Mode mode TOP determined by ICR1 - non-inverting Compare Output mode
        TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);    // set prescaler to 8, FastPWM Mode mode continued
        ICR1 = 20000;      // set period to 20 ms
        OCR1A = 1000;      // set count to 1 ms: 0 deg
        OCR1B = 1000;      // set count to 1 ms: 0 deg
        TCNT1 = 0;         // reset timer
    }

    { // init timer2
        TCCR2 = (1<<WGM20)| (1<<WGM21) | (1<<COM21) | (1<<CS22) | (1<<CS20); // fast PWM
        OCR2 = 1000/16; // set count to 1 ms: 0 deg
    }

    TIMSK |= (1<<TICIE1) | (1<<OCIE2); // initialize interrupts, activate
    sei();

    uint8_t sweep_up = 1;
    while (1) {
        _delay_ms(10);
        CLEAR(TEST_PIN);
        _delay_ms(10);
        SET(TEST_PIN);
        if (sweep_up  && OCR1B>=2000) sweep_up = 0;
        if (!sweep_up && OCR1B<=1000) sweep_up = 1;
        OCR1A += (sweep_up?4:-4);
        OCR1B += (sweep_up?4:-4);
        OCR2 = OCR1A/16;
    }

    return 0;
}

ISR(TIMER2_COMP_vect) { // overflow interrupt timer2
    TCCR2 &= 0x00; // delete timer2
}

ISR(TIMER1_CAPT_vect) { // capture interrupt timer1
    TCNT2 = 255;
    TCCR2 = (1<<WGM20) | (1<<WGM21) |(1<<COM21) | (1<<CS22) | (1<<CS20); // set timer2
}

