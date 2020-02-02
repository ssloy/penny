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

// The servos take a 50 Hz PWM signal; 1 ms minimum pulse width (0 deg), 2 ms maximum pulse width (90 deg).
// I have three servos, two of them are attached to a 16 bit timer (timer1), and the third one to a 8 bit timer (timer2).
// I dislike software PWM, therefore both timers are ticking in fast PWM mode.
// Timer1 ticks at 1 MHz (prescaler 8), and ICR1 provides the TOP value (20000), thus it restarts every 20 ms, providing a correct 50 Hz signal.
// OCR1A and OCR1B are the microsecond pulse widths for the left and right servos.
//
// The problem comes with the center servo attached to a 8 bit timer2.
// It does not have a handy ICR1 analog, so the overflowing frequency is controlled via the prescaler only.
// There are no prescalers good enough to approximate 50 Hz, here is an idea that lies somewhere between a software and a hardware PWM:
// we set timer2 to tick at prescaler 128, thus it overflows after 4.096 ms = 256*128/(8*10^6).
// At the overflow we disable the timer2, so it is a one shot pulse.
// At the timer1 capture interrupt we re-arm the (one-shot) timer2.
// 4 ms is superior to a 2 ms max pulse width we need to control, and is well inferior to the 20 ms re-arming beat.
// To sum up, let us say we want to put all three servos to the middle position (1.5 ms pulse width).
// We need to do the following:
// left servo:   OCR1A = 1500
// right servo:  OCR1B = 1500
// center servo: OCR2  = 1500/16

void init_servo_timers() {
    { // init timer1
        TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11); // fast PWM mode, TOP determined by ICR1 - non-inverting Compare Output mode
        TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);    // set prescaler to 8, fast PWM Mode mode continued
        ICR1 = 20000;      // set period to 20 ms
        OCR1A = 1000;      // set count to 1 ms: 0 deg
        OCR1B = 1000;      // set count to 1 ms: 0 deg
        TCNT1 = 0;         // reset timer
    }

    { // init timer2
        TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22) | (1<<CS20); // prescaler 128, fast PWM mode, clear OC2 on Compare Match, set OC2 at bottom (non-inverting mode)
        OCR2 = 1000/16; // set count to 1 ms: 0 deg
    }

    TIMSK |= (1<<TICIE1) | (1<<OCIE2); // initialize interrupts, activate
    sei();
}

ISR(TIMER2_COMP_vect) { // overflow interrupt timer2
    TCCR2 = 0; // stop timer2
}

ISR(TIMER1_CAPT_vect) { // capture interrupt timer1
    TCNT2 = 255;
    TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22) | (1<<CS20); // restart timer2
}

void set_servo_angles(uint8_t left, uint8_t right, uint8_t center) {
    OCR1A =  1000 + ((uint16_t)left  *100)/9;
    OCR1B =  1000 + ((uint16_t)right *100)/9;
    OCR2  = (1000 + ((uint16_t)center*100)/9)/16;
}

int main(void) {
    OUTPUT(TEST_PIN);
    OUTPUT(SERVO_L_PIN);
    OUTPUT(SERVO_R_PIN);
    OUTPUT(SERVO_C_PIN);

    init_servo_timers();

    uint8_t sweep_up = 1;
    uint8_t angle = 0;
    while (1) {
        _delay_ms(40);
        CLEAR(TEST_PIN);
        _delay_ms(40);
        SET(TEST_PIN);
        if ( sweep_up && angle>=90) sweep_up = 0;
        if (!sweep_up && angle==0)  sweep_up = 1;
        angle += (sweep_up?1:-1);
        set_servo_angles(angle, angle, angle);
    }

    return 0;
}

