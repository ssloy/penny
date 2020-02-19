# Meet Penny#3, a 3 servo hexapod.
Penny is a low budget (roughly ten bucks) hexapod. It can be a great one-weekend project to entertain yourself and your kids. Check for Penny dancing to funky music (clickable):

[![Penny dances](https://img.youtube.com/vi/quMe5CEoOok/0.jpg)](https://youtu.be/quMe5CEoOok)

Penny avoids obstacles (clickable):

[![Penny dances](https://img.youtube.com/vi/f4vZZBQZLEU/0.jpg)](https://youtu.be/f4vZZBQZLEU)

In her current form, Penny only knows to walk; she can see (measure distance to) nearby obstacles. Her brains, however, are powerful enough to digest data from many other sensors, send me your suggestions!

Penny is a tremendous fun!

<img src="https://raw.githubusercontent.com/ssloy/penny/master/doc/A_playing_with_penny.jpg" width="512"/>

## Credits
Penny has two elder sisters, [Penny](https://youtu.be/7Py03SH5DbE) and [Penny](https://youtu.be/PiVTC8JhZTQ). Note that I have no hardware contributions, all I did is to gather the information, assemble things and write the firmware. I want this wonderful robot to be easy to clone, therefore I created this repository. The original Penny#1 is created by [Jeremy Zimmer](https://www.robotshop.com/community/robots/show/penny). The wiring being cumbersome and cheapduino being discontinued, Dennis van Elteren has designed the motherboard that I also use. Thus Penny#2 was born. Here I present you Penny#3. While I have Dennis' sanction to publish his files, I failed to contact Jeremy. The software, however is distributed under the DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE.

# How to clone
## Bill of materials
Printing the body costs next to nothing if you have a 3d printer. Here are the main things you need to build the bot:
* The motherboard. You can either etch it by yourself, or you can check chinese factories, any normal day it costs ~10€ / 10 pcs (shipping included), with discounts it can cost 2€ / 10 pcs. You can find an alternative like cheapduino or similar, because the schematics is very, very basic.
* [SG90 9G micro servo, 3 * 1.47€ / piece](https://www.aliexpress.com/item/4000595327297.html)
* [4x AAA battery holder, 1.34€ / piece](https://www.aliexpress.com/item/33049875634.html)
* [ATMega8A-AU (QFP-32), 1€ / piece](https://www.aliexpress.com/item/32557093316.html)
* [IR LED + IR phototransistor, 0.20€ / pair](https://www.aliexpress.com/item/32849824664.html)
* [Electrolytic capacitor 1000uF 16V, 0.17€ / piece](https://www.aliexpress.com/item/32954075821.html)
* [2n3904 transistor, 3 * 0.01€ / piece](https://www.aliexpress.com/item/32494899564.html)
* You will need wires, heat shrink, screws, pin headers, few 0805 resistors and capacitors. All electronic components are listed in the [hardware/motherboard/BOM.html](https://github.com/ssloy/penny/blob/master/hardware/motherboard/BOM.html) file.

**NB:** When bying 9g servos, do not forget that they come in different sizes, Penny is designed for the low profile plastic gears servos. It will work with other servos, but you might need to update SketchUp files for it. Moreover, metal gears are an overkill here.

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/9g-servos-comparison.jpg)

## The body
It is quite straightforward, if you have a printer, just print it. You can find the files in the [hardware/body/](https://github.com/ssloy/penny/tree/master/hardware/body) folder. All the body parts are shown here:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny-body-print.jpg)

With my 1mm nozzle the prints were completed in a couple of hours. When assembled, it should look like this beast:
![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny-body-model.jpg)

M3 nylon screws are perfect for the assembly. Use nylon washers between moving parts and lock the nut by the method of your choice.
Personally I have locked the thread with a soldering iron:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/nylon-screws.jpg)

## The motherboard
### The brain
The motherboard itself is pretty basic. It has an ATMega8 microcontroller and the proximity sensor circuit, nothing else. 
The source files, the gerber files and the bill of materials can be found in the [hardware/motherboard/](https://github.com/ssloy/penny/tree/master/hardware/motherboard) folder. Here is the brain:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/pcb-mcu.png)

I recommend to solder the bare minimum to power up the processor, and to flash it to be sure that nothing is wrong with the delicate soldering. At this stage the motherboard looks like this:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny_motherboard_mcu.jpg)

**N.B. ATMega8A datasheet specifies its operating voltage at 2.7-5.5V and absolute maximum 6V rating.
A safe option is to power Penny with 4 NiMH 1.2V rechargeable batteries,
but I tried to run Penny with 4 standard alcaline batteries (6.4V in total) and it did not fry the brain.
If you are going this way, I did warn you. It is at your own risk!**

Here is a photo of the motherboard fully populated (with an exception of IR led/phototransistors):

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny_motherboard.jpg)

### The proximity sensor
Penny has two eyes, each one is composed of an infrared LED and a corresponding phototransistor.
The LED emits infrared light; this light propagates through the air and once it hits an object it is reflected back towards the phototransistor.
If the object is close, the reflected light will be stronger than if the object is further away.
Note that while the infrared light is not visible by a human eye, some cameras may see it and show it on the recordings,
it can be a handy tool for debugging Penny:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/ir-leds-on-camera.gif)

The schematics is very simple:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/pcb-proximity-sensor.png)

We power up two LEDs, when the phototransistors are not lit, the collector pins of Q3 and Q4 are tied to Vcc, and if the phototransistors
sense the light, the voltage on the collector pins will go down.
Here is the circuit test before I have installed it in the Penny's eyesockets:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/proximity-sensor.gif)

Note that you may need to adjust the R6 resistor value.
47 Ohms provides 55mA to the LEDs; some leds might need more or less current.
For example, I have scraped a couple of LEDs from a broken toy, and they work perfectly on 3mA (910 Ohms)!

I recommend to assemble first the sensing unit on a breadboard without the LEDs.
Then light up the LED with a CR2032 or a similar battery, pand put it against the phototransistor (I do not know the internal resistance of the battery, but I have never seen a LED fried from such an operation. Correct me if I am wrong).
Verify that the voltage on the Q3 and Q4 collector pins drops as expected.
Once the sensing unit is okay, try to find a good resistor value for the LEDs to obtain the behaivour you see on the above video.
Note that it is important to put a heatshrink around both the LEDs and the phototransistors to cut off parasitic lights.
Moreover, with heatshrink it fits neatly into the eyesockets. When soldering the 2n3904, I recommend to solder first the center pin, and only then the side pins, otherwise it is too easy to create hard-to-remove solder bridges. Personally I find these little basterds harder to solder than the microcontroller itself (but I am bad at soldering!).

If you fail to assemble the proximity sensor, or simply dislike it, there are plenty of options:

* You can use isf471 instead of the phototransistors and all the 2n3904 circuitry.
* You can buy Sharp GP2Y0A21YK0F distance measuring units: <br/> <img src="https://raw.githubusercontent.com/ssloy/penny/master/doc/GP2Y0A21YK0F.jpg" width="320"/>
* Or a basic binary proximity sensor based on a LM393 differential comparator: <br/> <img src="https://raw.githubusercontent.com/ssloy/penny/master/doc/lm393.jpg" width="320"/>

# Firmware explained
Penny can be programmed via arduino environment, but I find it quite obscure for such simple microcontrollers as ATMega8. Let us split the firmware comments into four parts: 
* [how to get the PWM working](https://github.com/ssloy/penny#pwm-generation)
* [how Penny moves legs](https://github.com/ssloy/penny#movement-planner)
* [gait sequences](https://github.com/ssloy/penny#gait-sequences)
* [obstacle avoidance strategy](https://github.com/ssloy/penny#obstacle-detection)

### PWM generation
The servos take a 50 Hz PWM signal; 1 ms minimum pulse width (0 deg), 2 ms maximum pulse width (90 deg). Penny has three servos, two of them are attached to a 16 bit timer (timer1), and the third one to a 8 bit timer (timer2). If I am not mistaken, arduino's Servo.h controls servomotors via software PWM, and I dislike that, therefore both timers are ticking in fast PWM mode.

The microcontroller ticks at 8 MHz, and the timer1 ticks at 1 MHz (prescaler 8), and ICR1 provides the TOP value (20000), thus it restarts every 20 ms, providing a correct 50 Hz signal. OCR1A and OCR1B registers control microsecond pulse widths for the left and right servos.

The problem comes with the center servo attached to a 8 bit timer2. It does not have a handy ICR1 analog, so the overflowing frequency is controlled via the prescaler only. There are no prescalers good enough to approximate 50 Hz, here is an idea that lies somewhere inbetween a software and a hardware PWM:
* we set timer2 to tick at prescaler 128, thus it overflows after 4.096 ms = 256 * 128/(8 * 10^6).
* At the overflow we disable the timer2, so it is a one shot pulse.
* At the timer1 capture interrupt we re-arm the (one-shot) timer2.

4 ms is superior to a 2 ms max pulse width we need to control, and is well inferior to the 20 ms re-arming beat. To sum up, let us say we want to put all three servos to the middle position (1.5 ms pulse width). We need to do the following:
```c
OCR1A = 1500;    // left servo
OCR1B = 1500;    // right servo
OCR2  = 1500/16; // center servo
```
### Movement planner

First of all, there are 6 important constants in the code:
```c
const uint8_t  zero[3] = {45, 50, 40};     // zero position of the servo (degrees)
const uint8_t range[3] = {25, 25, 20};     // the servos are allowed to move in the zero[i] +- range[i] interval
```
The `zero[3]` array provides the angles of all three servos corresponding to the neutral stance (left image below). Ideally, these angles are to be at 45° (midpoint of the servo range), but in practice if we set the servos to 45°, the legs won't be aligned due to descreet teeth position on the shaft. Next, `range[3]` specifies the admissible range. It means that the servo `i` is allowed to move in the `zero[i]-range[i]`to `zero[i]+range[i]` interval. 

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/servos-pos-range.jpg)

Current servo position (in degrees, 0°-90°) is supposed to be stored in the `uint8_t pos[3]` array. When calling `update_servo_timers()`, the timers are updated according to the array. The right image above corresponds to the `pos[i]=zero[i]+range[i]` for all three `i=0,1,2`.

All the movements are planned as constant speed. There are four auxiliary arrays for the movement planner: `pos_beg[3], pos_end[3], time_start[3]` and `duration[3]`. Let us suppose that we want to move the left servo only. All we need to do is:
* copy `pos[0]` to `pos_beg[0]`, it marks the starting point of the movement;
* set `pos_end[0]` to the desired position (still in degrees);
* set `time_start[0]` to the current timestamp (milliseconds since the boot);
* and, finally, set `duration[0]` (in seconds). That is, the speed will be `(pos_end[0]-pos_beg[0])/duration[0]` degrees/sec.

Then in an endless loop I invoke `movement_planner()`, it sets the goal `pos[]` according to the plan, and `update_servo_timers()` to update the PWM generator according to the `pos[]` position. 

### Gait sequences
Note that all movement planner variables are stored in 3-element arrays, thus the movements (including the speeds) can be independent one from another. 
Despite that, my current gait implementation uses synchronized movements of all three servos.
Let us see how Penny goes forward. To advance, Penny repeats in a loop 4 steps.
All 4 steps are linear movements to following goal positions:
* step 1: `{zero[0]-range[0], zero[1]-range[1], zero[2]+range[2]}`
* step 2: `{zero[0]-range[0], zero[1]-range[1], zero[2]-range[2]}`
* step 3: `{zero[0]+range[0], zero[1]+range[1], zero[2]-range[2]}`
* step 4: `{zero[0]+range[0], zero[1]+range[1], zero[2]+range[2]}`

We can express that as a 2d array (4 triplets of goal positions):
```c
const int8_t advance_sequence[4][3] = {{-1, -1,  1}, {-1, -1, -1}, { 1,  1, -1}, { 1,  1,  1}};
```
This array tells us that the goal position of the servo `i` at the step `step` is `zero[i] + range[i]*advance_sequence[step][i]`.
Then the following code makes Penny to go forward indefinitely:
```
    uint8_t step = steps_per_sequence-1; // at the initialization stage the (previous) movement is considered to be complete, thus the next movement will be planned starting from the step 0
    while (1) {
        if (is_movement_finished()) {
            step = (step + 1) % 4; // if previous movement is complete, then perform the next step; this variable loops as 0,1,2,3.
            plan_next_movement(step, advance_sequence); // execute next movement
        }
        movement_planner(); // update the servos position according to the planning
        _delay_ms(1);
    }
```

### Obstacle detection

Recall that our proximity sensor provides a voltage that we read throuh channels 4 and 5 of the ADC. To cut off eventual spikes in the readings (esp. knowing that the servos induce tons of noise), at each loop I update the variables `adc_left_eye` and `adc_right_eye` as a low-pass filter over the ADC readings:
```c
        adc_left_eye  = adc_left_eye *.99 + adc_read(5)*.01; // low-pass filter on the ADC readings
        adc_right_eye = adc_right_eye*.99 + adc_read(4)*.01;
```
The cutoff frequency can be set either by adjusting `_delay_ms()` inside the loop or by changing the `.99` and `1-.99` coefficients in the above code.

The presense of an obstacle is detected by a simple threshold over the ADC readings (recall that C does not have `bool` type so I use `uint8_t` instead):
```c
        uint8_t lobst = adc_left_eye  < distance_threshold; // obstacle on the left?
        uint8_t robst = adc_right_eye < distance_threshold; // obstacle on the right?
```

Then at the end of each step of the current sequence I verify if there is an obstacle present and change the plans accordingly. It is as simple as that!

```c
        if (is_movement_finished()) {
            if (!lobst && !robst) {
                sequence = advance_sequence; // no obstacles => go forward
            } else if (lobst && robst) {
                sequence = retreat_sequence; // obstacles left and right => go backwards
            } else if (lobst && !robst) {
                sequence = turn_right_sequence; // obstacle on the left => turn right
            } else if (!lobst && robst) {
                sequence = turn_left_sequence; // obstacle on the right => turn left
            }
            step = (step + 1) % steps_per_sequence; // if previous movement is complete, then perform the next step
            plan_next_movement(step, sequence); // execute next movement
        }
```

# Wishlist
Any contribution is welcome! Send me your ideas; here is a list of things that I'd like to see improved:

### software:
* Propose me an elegant way to have more natural, life-like movements. Right now it moves by a linear interpolations between keyframes, it would be great if the robot was less shaky. Probably, non-linear interpolation between the same keyframes with pre-computed accelerations?
* Propose new strategies of obstacle detection. Current implementation is very basic and aims a good source code readability rather then WOW robot's behaviour.
* I guess that it would be a good idea to port the code to the arduino environment for those who do not want to call avr-gcc directly (or for those who are afraid of meddling with AVR registers). If you can do it, send me a pull request or fork the repository.

### hardware:
If you are a good soul willing to create a V2 of the motherboard, you are very welcome to do so. Here are the things that I'd like to be fixed/added/modified in the motherboard:
* The main thing is the on/off switch to cut off servo motors power during flashing;
* Remove the crystal, internal RC should be just fine;
* Replace through-hole components by their SMD analogs;
* Replace R6 with a potentiometer to adjust the proximity sensor more easily;
* Propose good (small and foolproof) connectors instead of pin headers and optimize their placement;
* IR LEDs pads are very hard to reach under the center servo. The only viable option with the V1 motherboard is to solder the wires;
* Move a little bit the big capacitor. I had to incline it, otherwise the screw in center legs would destroy it;
* Add test pads easy to access with an oscilloscope;
* Add a couple of debugging LEDs;
* Create good soldering points for unused ATMega8 pins for debugging purposes and further extension.
