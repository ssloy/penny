# Meet Penny#3, a 3 servo hexapod.

Check Penny dancing to funky music (clickable):

[![Penny dances](https://img.youtube.com/vi/quMe5CEoOok/0.jpg)](https://youtu.be/quMe5CEoOok)

# Credits

Penny has two elder sisters, [Penny](https://youtu.be/7Py03SH5DbE) and [Penny](https://youtu.be/PiVTC8JhZTQ). Note that I have no hardware contributions, all I did is to gather the information, assemble things and write the firmware. I want this wonderful robot to be easy to clone, therefore I created this repository. The original Penny#1 is created by [Jeremy Zimmer](https://www.robotshop.com/community/robots/show/penny). The wiring being cumbersome and cheapduino being discontinued, Dennis van Elteren has designed the motherboard that I also use. Thus Penny#2 was born. Here I present you Penny#3. While I have Dennis' approbation to publish his files, I failed to contact Jeremy. The software, however is distributed under the DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE.

# How to clone
## The body

It is quite straightforward, if you have a printer, just print it. All the body parts are shown here:

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny-body-print.jpg)

When assembled, it should look like this beast:
![](https://raw.githubusercontent.com/ssloy/penny/master/doc/penny-body-model.jpg)

## The motherboard

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/pcb-mcu.png)

### Bill of materials
Printing the body costs next to nothing if you have a 3d printer. Here are the main things you need to build the bot:
* The motherboard. You can either etch it by yourself, or you can check chinese factories, any normal day it costs ~10€ / 10 pcs (shipping included), with discounts it can cost 2€ / 10 pcs.
* [SG90 9G micro servo, 3 * 1.47€ / piece](https://www.aliexpress.com/item/4000595327297.html)
* [4x AAA battery holder, 1.34€ / piece](https://www.aliexpress.com/item/33049875634.html)
* [ATMega8A-AU (QFP-32), 1€ / piece](https://www.aliexpress.com/item/32557093316.html)
* [IR LED + IR phototransistor, 0.20€ / pair](https://www.aliexpress.com/item/32849824664.html)
* [Electrolytic capacitor 1000uF 16V, 0.17€ / piece](https://www.aliexpress.com/item/32954075821.html)
* [2n3904 transistor, 3 * 0.01€ / piece](https://www.aliexpress.com/item/32494899564.html)
* You will need wires, heat shrink, screws, pin headers, few 0805 resistors and capacitors.


### The proximity sensor

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/pcb-proximity-sensor.png)

![](https://raw.githubusercontent.com/ssloy/penny/master/doc/proximity-sensor.gif)

