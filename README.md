# Meet Penny#3, a 3 servo hexapod.

Penny is a low budget (roughly ten bucks) hexapod. It can be a great one-weekend project to entertain yourself and your kids. Check for Penny dancing to funky music (clickable):

[![Penny dances](https://img.youtube.com/vi/quMe5CEoOok/0.jpg)](https://youtu.be/quMe5CEoOok)

Penny avoids obstacles (clickable):

[![Penny dances](https://img.youtube.com/vi/f4vZZBQZLEU/0.jpg)](https://youtu.be/f4vZZBQZLEU)

In her current form, Penny only knows to walk; she can see (measure distance to) nearby obstacles. Her brains, however, are powerful enough to digest data from many other sensors, send me your suggestions!

Penny is a tremendous fun!

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/A_playing_with_penny.jpg)


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


## The body
It is quite straightforward, if you have a printer, just print it. You can find the files in the [hardware/body/](https://github.com/ssloy/penny/tree/master/hardware/body) folder. All the body parts are shown here:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny-body-print.jpg)

With my 1mm nozzle the prints were completed in a couple of hours. When assembled, it should look like this beast:
![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny-body-model.jpg)

## The motherboard

The motherboard is pretty basic. It has an ATMega8 mcu and the proximity sensor circuit, nothing else. Here is the brain:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/pcb-mcu.png)

I recommend soldering the bare minimum to power up the processor, and to flash it to be sure that nothing is wrong with the fine soldering. At this stage the motherboard looks like this:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny_motherboard_mcu.jpg)


### The proximity sensor
![](https://raw.githubusercontent.com/ssloy/penny/master/doc/pcb-proximity-sensor.png)

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/proximity-sensor.gif)

# Wishlist
If you are a good soul willing to create a V2 of the motherboard, you are very welcome to do so. Here are the things that I'd like to be fixed/added/modified in the motherboard:
* The main thing is the on/off switch to cut off servo motors during flashing;
* Remove the crystal, internal RC should be just enough;
* Replace through-hole components by their SMD analogs;
* Propose good (small and foolproof) connectors instead of pin headers and optimize their placement;
* IR LEDs pads are very hard to reach under the center servo. The only viable option with the V1 motherboard is to solder the wires;
* Move a little bit the big capacitor. I had to incline it, otherwise the screw in center legs would destroy it;
* Add test pads easy to access with an oscilloscope;
* Add a couple of debug LEDs;
* Create good soldering points for unused ATMega8 pins for debugging and further extension.

I guess that it would be a good idea to port the code to arduino environment for those who do not want to call avr-gcc directly. If you can do it, send me a pull request.
