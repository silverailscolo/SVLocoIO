![workflow status](https://github.com/silverailscolo/SVLocoIO/actions/workflows/compile-sketches.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/sync-labels.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/code-formatting-check.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/spell-check.yml/badge.svg)

## Overview
**SVLocoIO** is a sketch for an Arduino Nano based LocoIO module.

Version 1.07 includes:
- startup reporting in the Serial Console
- blinking outputs using the Blink Rate board setting
- read/configure ports using commands over serial (type H in Serial Monitor)

The code requires GCA50a hardware, available as a PCB or kit from [P. Giling](https://wiki.rocrail.net/doku.php?id=gca51-en).

Earlier version were successfully used in RocRail and JMRI.

### Authors
- [Dani Guisado](http://www.clubncaldes.com)
- [Egbert Broerse](https://github.com/silverailscolo)

## Description
SVLocoIO is a LocoNet Interface with 16 I/O that can be individually
configured as either Input (block sensors) or Output (switches, lights,...).
The "heart" of the GCA50a is an Arduino Nano plugged into the PCB.
This software emulates the functionality of a GCA50 module from Peter
Giling, plus additional options from Hans Deloofs LocoIO v148, which was
in turn derived from John Jabour's original LocoIO.

Configuration is done through LocoNet SV1 protocol and can be configured
from Rocrail (Programming->GCA->GCA50), from JMRI (LocoIO Tool) or - since version 107 - from the Serial Console over USB.

### Credits
* Based on [MRRwA LocoNet libraries for Arduino](http://mrrwa.org/) and
  its LocoNet Monitor example.
* The included `rfid2ln` lib was adapted from https://github.com/lmmeng/rfid2ln
  to compile in Arduino IDE.
* Inspired on the GCA50 board from [Peter Giling](http://www.phgiling.net/)
* Also inspired by LocoShield from [SPCoast](http://www.scuba.net/)
* Thanks also to [Rocrail group](http://www.rocrail.org)
* 
* Based on MRRwA LocoNet libraries for Arduino - http://mrrwa.org/ and
  the LocoNet Monitor example.
* Inspired on the GCA50 board from Peter Giling - http://www.phgiling.net/
* Idea also inspired by LocoShield from SPCoast - http://www.scuba.net/
* Thanks also to Rocrail group - http://www.rocrail.org

## Installation

Follow the Arduino IDE guidelines. 

- Install the "SerialCommand_Advanced" library by argandas from the Arduino IDE > Tools > Library manager.

- Download the mrrwa [LocoNet library](https://github.com/mrrwa/LocoNet/blob/master/LocoNet.h) as a .ZIP and install it in the IDE using the Sketch > Include Library > Add .ZIP Library... menu.

> ### Tip:
> When uploading the sketch to the Nano in Arduino IDE fails, try Tools > Processor: ATmega328P (Old Bootloader).

### Nano IO pinout
| pin | function                                                    |
|-----|-------------------------------------------------------------|
| 0,1 | Serial, used to debug and LocoNet Monitor (uncomment DEBUG) 
| 2,3,4,5,6 | Configurable I/O from 1 to 5                                
| 7 | LocoNet TX                    
| 8 | LocoNet RX                    
| 9,10,11,12,13 | Configurable I/O from 6 to 10                               
| A0,A1,A2,A3,A4,A5 | Configurable I/O from 11 to 16                              
