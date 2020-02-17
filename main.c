#define F_CPU 8000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint32_t millis = 0; // an approximation of milliseconds elapsed since boot

// these parameters are to tune depending on your actual hardware
const uint8_t  zero[3] = {45, 50, 40};     // zero position of the servo (degrees)
const uint8_t range[3] = {25, 25, 20};     // the servos are allowed to move in the zero[i] +- range[i] interval
const uint16_t distance_threshold = 768;   // ADC threshold for obstacle detection, in principle atmega8 ADC returns [0,1023] values, but the schematics imposes a narrower interval

// movement planner variables
volatile uint8_t     pos[3] = {45, 45, 45};        // current servo position            (degrees)
volatile uint8_t pos_beg[3] = {45, 45, 45};        // starting position for each servo  (degrees)
volatile uint8_t pos_end[3] = {45, 45, 45};        // goal position for each servo      (degrees)
volatile uint32_t time_start[3] = {0, 0, 0};       // beginning of the motion timestamp (milliseconds)
volatile float      duration[3] = {.1f, .1f, .1f}; // movement duration                 (sec)

// gait sequences
const uint8_t steps_per_sequence = 4;                  // each gait is a periodic sequence of 4 steps
const float step_time[4] = {.33f, .166f, .33f, .166f}; // duration of each of 4 steps
const int8_t    advance_sequence[4][3] = {{-1, -1,  1}, {-1, -1, -1}, { 1,  1, -1}, { 1,  1,  1}}; // for each of 4 steps
const int8_t    retreat_sequence[4][3] = {{-1, -1, -1}, {-1, -1,  1}, { 1,  1,  1}, { 1,  1, -1}}; // these numbers determine movement goals,
const int8_t  turn_left_sequence[4][3] = {{ 1, -1,  1}, { 1, -1, -1}, {-1,  1, -1}, {-1,  1,  1}}; // whether we have to add or subtract range[i] from zero[i]
const int8_t turn_right_sequence[4][3] = {{-1,  1,  1}, {-1,  1, -1}, { 1, -1, -1}, { 1, -1,  1}}; // i.e. the goal position of the servo i is zero[i] + range[i]*sequence[step][i]

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

void update_servo_timers() {
    OCR1A =  1000 + ((uint16_t)pos[0]*100)/9;     //  left
    OCR1B =  1000 + ((uint16_t)pos[1]*100)/9;     //  right
    OCR2  = (1000 + ((uint16_t)pos[2]*100)/9)/16; //  center
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
    update_servo_timers();
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

// for the given servo, this function returns its current step phase [0.f, 1.f]
inline float movement_phase(uint8_t servo) {
    return CLAMP((millis - time_start[servo])/(1000.f * duration[servo]), 0.f, 1.f);
}

// update PWM servo signals according to the movement planning
inline void movement_planner() {
    for (uint8_t i=0; i<3; i++)
        pos[i] = CLAMP(pos_beg[i] + (pos_end[i] - pos_beg[i])*movement_phase(i), zero[i]-range[i], zero[i]+range[i]);
    update_servo_timers();
}

// copy the step of the sequence to the movement planner variables
inline void plan_next_movement(uint8_t step, int8_t sequence[][3]) {
    for (uint8_t i=0; i<3; i++) {                           // for each of 3 servos
        pos_beg[i] = pos[i];                                // we go from pos_beg[i]
        pos_end[i] = zero[i] + range[i]*sequence[step][i];  // to pos_end[i],
        time_start[i] = millis;                             // the movement is initiated at time_start[i] (right now)
        duration[i] = step_time[step];                      // and should last for step_time[step]
    }
}

// are we done moving?
inline uint8_t is_movement_finished() {
    uint8_t finished = 1;
    for (uint8_t i=0; i<3; i++)
        finished &= (movement_phase(i)>=1.f);
    return finished;
}

// well duh
void go_to_zero_stance() {
    for (uint8_t i=0; i<3; i++) pos[i] = zero[i];
    update_servo_timers();
}

int main(void) {
    DDRC &= ~_BV(4); // input: right phototransistor
    DDRC &= ~_BV(5); // input: left  phototransistor
    DDRC |=  _BV(0); // output: IR LED control
    DDRB |=  _BV(1); // output: left servo
    DDRB |=  _BV(2); // output: right servo
    DDRB |=  _BV(3); // output: center servo
    PORTC |= _BV(0); // set the LED control pin to HIGH
    init_servos();

    go_to_zero_stance();
    _delay_ms(3000);

    int8_t (*sequence)[3] = (int8_t(*)[3])advance_sequence; // by default we go forward
    uint8_t step = steps_per_sequence-1; // at the initialization stage the (previous) movement is considered to be complete, thus the next movement will be planned starting from the step 0

    uint16_t adc_left_eye  = adc_read(5);
    uint16_t adc_right_eye = adc_read(4);
    uint32_t start = millis;
    while (1) {
        adc_left_eye  = adc_left_eye *.99 + adc_read(5)*.01; // low-pass filter on the ADC readings
        adc_right_eye = adc_right_eye*.99 + adc_read(4)*.01;
        uint8_t lobst = adc_left_eye  < distance_threshold; // obstacle on the left?
        uint8_t robst = adc_right_eye < distance_threshold; // obstacle on the right?

        if (is_movement_finished()) {
            if (!lobst && !robst && step==steps_per_sequence-1) { // the obstacles are checked at every step with an exception of go forward decision, it is made at the end of each sequence
                sequence = (int8_t(*)[3])advance_sequence; // no obstacles => go forward
            } else if (lobst && robst) {
                sequence = (int8_t(*)[3])retreat_sequence; // obstacles left and right => go backwards
            } else if (lobst && !robst) {
                sequence = (int8_t(*)[3])turn_right_sequence; // obstacle on the left => turn right
            } else if (!lobst && robst) {
                sequence = (int8_t(*)[3])turn_left_sequence; // obstacle on the right => turn left
            }
            step = (step + 1) % steps_per_sequence; // if previous movement is complete, then perform the next step
            plan_next_movement(step, sequence); // execute next movement
        }
        movement_planner(); // update the servos position according to the planning

        _delay_ms(1);
        if (millis-start>60L*1000L) break; // 60 seconds total running time
    }

    go_to_zero_stance();
    while (1) _delay_ms(10);
    return 0;
}

