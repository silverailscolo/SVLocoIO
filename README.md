## Arduino sketch for the GCA50a ##

#### An Arduino Nano based open source LocoIO board with 16 I/O ports ####

Basically it is a LocoNet interface base board for Arduino Nano giving you these functionalities:

- Power source to Arduino Nano via LocoNet
- Sub DB9 and/or RJ12 LocoNet connectors
- J5 and J6 connectors for standard Giling driver boards connection

Having Arduino connected to your LocoNet bus and being able to receive and send any LocoNet command, you can build any type of device: throttles, sound modules, illumination, turnout control, automation, signaling...

The original version of this sketch was written by [Club N Caldes](http://www.clubncaldes.com/), Spain. They have all their system based on two of the most famous standards (DCC and LocoNet), but everything implemented in Open Source and Open Hardware platforms like Arduino.

The code requires GCA50a hardware, available as a PCB or kit from [P. Giling](https://wiki.rocrail.net/doku.php?id=gca51-en).

## Options ##

- The maximum input port address has been increased to 2048.
- Inform about the state of the inputs (sensors) at power on.
When the command station is turned on, or the power on button of your software like Rocrail or JMRI is pressed, the module will send the current state of all ports configured as inputs.
With a bit more of technical detail, when a OPC_GPON command is sent throuth Loconet, the board responds a OPC_INPUT_REP message for each configured input.
If you want to deactivate this functionality, delete or comment the line in the top of the code:
``#define INFORMATPOWERON``
- Blinking outputs. The blink rate is configurable per board (0-15, that's 2-1 seconds)

## Installation ##

Follow the Arduino IDE guidelines.

Download the mrrwa [LocoNet](https://github.com/mrrwa/LocoNet) library as a .ZIP and install it in the IDE using the Sketch > Include Library > Add .ZIP Library... menu.
