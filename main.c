#define F_CPU 8000000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint32_t millis = 0; // an approximation of milliseconds elapsed since boot

const uint8_t zero_pos[3] = {50, 45, 40};
const uint8_t min_off[3] = {30, 30, 15};
const uint8_t max_off[3] = {20, 20, 15};

volatile uint8_t cur_pos[3] = {45, 45, 45};


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

void set_servo_angles(uint8_t left, uint8_t right, uint8_t center) {
    OCR1A =  1000 + ((uint16_t)left  *100)/9;
    OCR1B =  1000 + ((uint16_t)right *100)/9;
    OCR2  = (1000 + ((uint16_t)center*100)/9)/16;
}

inline void init_timer1() {
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11); // fast PWM mode, TOP determined by ICR1 - non-inverting Compare Output mode
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);    // set prescaler to 8, fast PWM Mode mode continued
    ICR1 = 20000;      // set period to 20 ms
    TCNT1 = 0;         // reset timer
}

inline void init_timer2() {
    TCNT2 = 255;
    TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22) | (1<<CS20); // prescaler 128, fast PWM mode, clear OC2 on Compare Match, set OC2 at bottom (non-inverting mode)
}

void init_servos() {
    init_timer1();
    init_timer2();
    set_servo_angles(45, 45, 45); // put the servos in the middle position
    TIMSK |= (1<<TICIE1) | (1<<OCIE2); // initialize interrupts, activate
    sei();
}

ISR(TIMER2_COMP_vect) { // overflow interrupt timer2
    TCCR2 = 0; // stop timer2
}

ISR(TIMER1_CAPT_vect) { // capture interrupt timer1
    millis += 20;
    init_timer2();
}

uint16_t adc_read(uint8_t ch) {
    ADMUX = (1<<REFS0) | (ch&7); // AVcc with external capacitor at AREF pin
    ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, start one conversion, set the prescaler to 128 (8000KHz/128 = 62.5KHz)
    while (ADCSRA & (1<<ADSC)); // we have started a single conversion; wait for the conversion to complete
    return ADC;
}

#define MIN(a, b)            (((a) < (b)) ? (a) : (b))
#define MAX(a, b)            (((a) > (b)) ? (a) : (b))
#define CLAMP(x, low, high)  (MIN(MAX((x), (low)), (high)))

void go(uint8_t lgoal, uint8_t rgoal, uint8_t cgoal, float duration) {
    uint8_t start_pos[3] = {cur_pos[0], cur_pos[1], cur_pos[2]};
    uint8_t goal_pos[3] = {lgoal, rgoal, cgoal};
    uint32_t time_start = millis;
    while (1) {
        float t = (millis - time_start)/1000.f;
        for (uint8_t i=0; i<3; i++) 
            cur_pos[i] = CLAMP(start_pos[i] + (goal_pos[i] - start_pos[i])*t/duration, zero_pos[i]-min_off[i], zero_pos[i]+max_off[i]);
        set_servo_angles(cur_pos[0], cur_pos[1], cur_pos[2]);
        _delay_ms(5);
        if (t>duration) break;
    }
    for (uint8_t i=0; i<3; i++) cur_pos[i] = goal_pos[i];
}

int main(void) {
    DDRC &= ~_BV(4); // input: left  phototransistor
    DDRC &= ~_BV(5); // input: right phototransistor
    DDRC |=  _BV(0); // output: IR LED control
    DDRB |=  _BV(1); // output: left servo
    DDRB |=  _BV(2); // output: right servo
    DDRB |=  _BV(3); // output: center servo
    PORTC |= _BV(0); // set the LED control pin to HIGH
    init_servos();

/*
    uint16_t adc_left_eye  = adc_read(4);
    uint16_t adc_right_eye = adc_read(5);

    uint8_t angle_left  = 0;
    uint8_t angle_right = 0;
    while (1) {
        adc_left_eye  = adc_left_eye *.99 + adc_read(4)*.01;
        adc_right_eye = adc_right_eye*.99 + adc_read(5)*.01;

        angle_left  = (adc_left_eye <256 ? 45 : 75);
        angle_right = (adc_right_eye<256 ? 45 : 75);
        set_servo_angles(angle_left, angle_right, 45);

        _delay_ms(1);
    }
*/

    set_servo_angles(zero_pos[0], zero_pos[1], zero_pos[2]);
    _delay_ms(2000);
//  set_servo_angles(left_min, right_min, center_min);
//  _delay_ms(2000);
//  set_servo_angles(left_max, right_max, center_max);
//  _delay_ms(2000);

    uint32_t start = millis;
    while (1) {
    /*
        if (0) {
            // go forward
            go(angle_left, angle_right, center_max, .1f);
            go(left_min, right_min, angle_center, .5f);
            go(angle_left, angle_right, center_min, .1f);
            go(left_max, right_max, angle_center, .5f);
        }
        */

          go(zero_pos[0], zero_pos[1], zero_pos[2]+max_off[2], .25f);
          go(zero_pos[0], zero_pos[1], zero_pos[2]-min_off[2], .25f);
          go(zero_pos[0], zero_pos[1], zero_pos[2]+max_off[2], .25f);
          go(zero_pos[0], zero_pos[1], zero_pos[2]-min_off[2], .25f);

          go(zero_pos[0], zero_pos[1]+max_off[1],   zero_pos[2]-min_off[2], .25f);
          go(zero_pos[0], zero_pos[1]-min_off[1]/2, zero_pos[2]-min_off[2], .25f);

          go(zero_pos[0], zero_pos[1], zero_pos[2]-min_off[2], .25f);
          go(zero_pos[0], zero_pos[1], zero_pos[2]+max_off[2], .25f);
          go(zero_pos[0], zero_pos[1], zero_pos[2]-min_off[2], .25f);
          go(zero_pos[0], zero_pos[1], zero_pos[2]+max_off[2], .25f);

          go(zero_pos[0]+max_off[0], zero_pos[1],   zero_pos[2]+max_off[2], .25f);
          go(zero_pos[0]-min_off[0]/2, zero_pos[1], zero_pos[2]+max_off[2], .25f);


/*
        go(angle_left, angle_right, center_max, .1f);
        go(left_min, right_max, angle_center, .5f);
        go(angle_left, angle_right, center_min, .1f);
        go(left_max, right_min, angle_center, .5f);
        _delay_ms(5);
*/
        if (millis-start>5*1000) break;
    }
    set_servo_angles(zero_pos[0], zero_pos[1], zero_pos[2]);
//  set_servo_angles(45, 45, 45);
    while (1);

    return 0;
}

