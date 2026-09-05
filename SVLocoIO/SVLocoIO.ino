/**************************************************************************
    SVLocoIO a.k.a. GCA50a - Configurable Arduino LocoNet Module on Arduino Nano
    Copyright (C) 2014-2019 Daniel Guisado Serra
    Copyright (C) 2024-2026 EJ Broerse @silverailscolo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

 ------------------------------------------------------------------------
 AUTHOR : Dani Guisado - http://www.clubncaldes.com - dguisado@gmail.com
 AUTHOR : Egbert Broerse - https://github.com/silverailscolo
 AUTHOR : Michael Hochmuth - http://www.simandit.de
 ------------------------------------------------------------------------
 DESCRIPTION:
    SVLocoIO is a LocoNet Interface with 16 I/O that can be individually
    configured as Inputs (block sensors) or Outputs (switches, lights,...).
    The "heart" of the GCA51 is an Arduino Nano plugged into the PCB.
    This software emulates the functionality of a GCA50 module from Peter
    Giling, plus additional options from Hans Deloofs LocoIO v148, which was
    in turn derived from John Jabour's original LocoIO.
    Configuration is done through LocoNet SV1 protocol and can be configured
    from Rocrail (Programming->GCA->GCA50), from JMRI (LocoIO Tool) or
    - since version 107 - from the Serial Console over USB.
 ------------------------------------------------------------------------
 NANO PIN ASSIGNMENT:
   0,1 -> Serial, used to debug and LocoNet Monitor (uncomment DEBUG)
   2,3,4,5,6 -> Configurable I/O from 1 to 5
   7 -> LocoNet TX (connected to GCA185 shield)
   8 -> LocoNet RX (connected to GCA185 shield)
   9,10,11,12,13 -> Configurable I/O from 6 to 10
   A0,A1,A2,A3,A4,A5-> Configurable I/O from 11 to 16
 ------------------------------------------------------------------------
 CREDITS:
 * Based on MRRwA LocoNet libraries for Arduino - http://mrrwa.org/ and 
   the LocoNet Monitor example.
 * Inspired on the GCA50 board from Peter Giling - http://www.phgiling.net/
 * Also inspired by LocoShield from SPCoast - http://www.scuba.net/
 * Thanks also to Rocrail group - http://www.rocrail.org
 ------------------------------------------------------------------------
 LAST CHANGES:
 22/1/2026 Added board configuration options as LocoIO 1.48 (flashing rate/
 flashing outputs)
 22/1/2026 Added config over Serial Commands
 22/1/2026 Store version in SV100, board config in SV0
 Version 110:
 4/9/2026 Support additional option Active High by Michael Hochmuth
 4/9/2026 Support continuous output at contact2 by Michael Hochmuth
*************************************************************************/

#include <Arduino.h>
#include <LocoNet.h>
#include <EEPROM.h>
#include <SerialCommand.h>

#define VERSION       110                      // 106 for GCA50a LocoIO (v148) functions, must be type int
#define DEBUG                                  // Uncomment this line to debug through the serial monitor
#define SERIAL_CMD                             // enable configuration over Serial Monitor
#define INFORMATPOWERON                        // Uncomment this line to not inform of the inputs state at power on
#define LN_TX_PIN       7                      // Arduino Pin used as LocoNet Tx; Rx Pin is always the ICP Pin
#define RST_PIN         6                      // Arduino Pin used as ResetPowerDownPin
#define LocoLED         4                      // LocoLED lights up when there is LocoNet communication
#define LocoLED_wait  200                      // LocoLED is always on for 200 msec
#define PulseTime     300                      // PulseTime for all the Pulse Outputs in msec. - TODO board config SV0 bit y
#define WaitTime      500                      // Wait Time for all block Inputs in msec.
#define FlashTime     250                      // Frequency of flasher - TODO board config SV0 bit x

//#define ELEMENTCOUNT(x) (sizeof(x) / sizeof(int)) // (sizeof(x[0]) / sizeof(int)  // function for cnfg lookup

boolean bSerialOk = false;
uint8_t ucBoardAddrHi = 1;                     // board address high; default 1
uint8_t ucBoardAddrLo = 88;                    // board address low; default 88

namespace {
//#define VIDA_LOCOSHIELD_NANO 1

// Arduino pin assignment to each of the 16 LocoIO ports
#ifdef VIDA_LOCOSHIELD_NANO
uint8_t pinMap[16] = {11, 10, 9, 6, 5, 4, 3, 2, 15, 14, 19, 18, 17, 16, 13, 12};
#else
uint8_t pinMap[16] = {2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
#endif
}

// 3 bytes defining a pin's behavior ( http://wiki.rocrail.net/doku.php?id=loconet-io-en )
typedef struct
{
  uint8_t cnfg;
  uint8_t value1;
  uint8_t value2;
} PIN_CFG;

// Memory map exchanged with SV read and write commands ( http://wiki.rocrail.net/doku.php?id=lnsv-en )
typedef struct
{
  uint8_t board_cnfg;
  uint8_t addr_low;
  uint8_t addr_high;
  PIN_CFG pincfg[16];
} SV_TABLE;

// Union to access the data with the struct or by index
typedef union {
  SV_TABLE svt;
  uint8_t data[51];
} SV_DATA;

SV_DATA svtable;
lnMsg *LnPacket;

// Table with addresses of pins already converted, input numbers are stored in a different way than output numbers.
uint16_t softwareAddress[16]; // composite software address of all the hardware ports

const uint8_t configCodes[16] = {0,15,23,27,31,39,47,55,91,95,103,104,128,129,136,140,144,145,192,208}; // backup for configOptions lookup, requires more mem

// next info would be nice but memory hog

//typedef struct CNFG_OPTIONS
//{
//  uint8_t code;
//  char *description;                    // max length of description = 3 in getConfig(i)
//};

//const PROGMEM CNFG_OPTIONS configOptions[16] = {
// inputs:
//  {0, "un"},            // [0] port unused
//  {15, "tg"},           // [1] toggle
//  {23, "t"},            // [2] single contact "normal" turnout feedback
//  {27, "bld"},          // [3] block - active low, delayed
//  {31, "bl"},           // [4] block - active low
//  {39, "bti"},          // [5] button - active low, indirect
//  {47, "bt"},           // [6] button - active low
//  {55, "t2c"},          // [7] 2 contacts turnout feedback, for 2: .value2 bits 4-7 = 3
//  {91, "bhd"},          // [8] block - active high, delayed
//  {95, "bh"},           // [9] block - active high
//  {103, "bti"},         // [10] button - active high, indirect
//  {104, "bt"},          // [11] button - active high
// outputs (bit 7 == 1)
//  {128, "off"},         // [a] for 1: .value2 bits 4-7 (JMRI HDL LocoIO Value2A) = 1
//  {129, "on"},          // [b] for 2: .value2 bits 4-7 = 3
//  {136, "pls"},         // [c] pulse soft reset
//  {140, "plh"},         // [d] pulse hard reset
//  {144, "ofx"},         // [e] off, flashing
//  {145, "onx"},         // [f] on, flashing
//  {192, "occ"},         // [g] block occupancy
//  {208, "ocx"},         // [h] block occ. flashing
//};
// TODO: Add Active High codes

// Timers for each input that is configured as "delayed"
// inputs defined as "delayed" will keep the signal high at least 2 seconds (why 2s? LocoIO docs says: 1-2*blinkDuration)
unsigned long inpTimer[16];            // block delay per cnfg port
uint8_t blinkRate = 0;                 // default board setting for blinking rate of output ports
uint16_t blinkDuration = 1000;
unsigned long currentBlinkMillis = 0;  // use the same time for all LED flashes to keep them synchronized
unsigned long previousBlinkMillis = 0; // last time LED changed state
uint8_t blinkState[16];

// other board config options
boolean alternateMode = false;
boolean portRefresh = false;          // TODO send update of all input states when SV0 is written

// functions common with GCA51a LocoIO are in GCA51Func.cpp
// extern boolean processPeerPacket();
// extern void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2); // added LocoLED blink, so must override GCA50a method
// extern void notifySwitchRequest(uint16_t Address, uint8_t Output, uint8_t Direction); // idem

#ifdef SERIAL_CMD
  SerialCommand SCmd;  // The SerialCommand object
#endif
// ********************************** Utility methods ***************************************

/*********************************************************************************
    Purpose: signal LocoNet activity on Nano
    Turn LocoLED on  when LocoNet communication starts
    Turn LocoLED off when the waiting time has elapsed
 *********************************************************************************/
void LocoNet_communication(byte on_off)
{
  static unsigned long LED_on_off;

  if (on_off == 1)                        // LocoLED is off
  {
    digitalWrite (LocoLED, HIGH);         // to signal LocoNet communication has started, turn on LocoLED
    LED_on_off = millis();                // remember start time
  }

  if ((LED_on_off + LocoLED_wait) < millis())  // if the wait time has expired
  {
    LED_on_off = 0;
    digitalWrite (LocoLED, LOW);               // turn off LocoLED
  }
}

/*********************************************************************************
    Purpose: calculate the software addresses and store them in global variable softwareAddress[16]
 *********************************************************************************/
void CalculateAddress()
{
  uint8_t n; // used for lookup
  byte odd_even;

  // I/O ports 0-15
  for (n = 0; n < 16; n++)
  {
    // validate
    if (isValidConfig(svtable.svt.pincfg[n].cnfg))
    //if (findConfig(svtable.svt.pincfg[n].cnfg) != -1) // read error? unexpected value for uint8_t, replace by 255?
    {
      if (!bitRead(svtable.svt.pincfg[n].cnfg, 7)) // configured as inputs, active low
      {
        if (bitRead (svtable.svt.pincfg[n].value2, 5)) odd_even = 2; // bitRead for bit 5 in SV5, SV8, SV11, SV14 etc.
        else odd_even = 1;

        softwareAddress[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[n].value1 << 1 ) + odd_even); // Calculate software address of port. For Port 1 .value1 == SV4 and .value2 == SV5
        // (SV5 & 0x0F) << 8 == high byte + SV4 << 1 == low byte + odd_even == software-address of the hardware-port
        Serial.print(F("- Port ")); Serial.print(n); Serial.print(F(" [H")); Serial.print (n); Serial.print(F("] input, address: ")); Serial.print(softwareAddress[n], DEC);
        Serial.print(F(" (cfg: "));
        Serial.print(svtable.svt.pincfg[n].cnfg);
        // Serial.print(F(" "));
        // Serial.print(getConfig(svtable.svt.pincfg[n].cnfg)); // adds pin config description - TODO fix
        Serial.println(F(")"));
      }
      else if (bitRead(svtable.svt.pincfg[n].cnfg, 7)) // configured as outputs
      {
        softwareAddress[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[n].value1) + 1);
        // Calculate software address of the port. E.g. for Port 1 .value1 == SV4 and .value2 == SV5
        Serial.print ("- Port "); Serial.print (n); Serial.print (" [H"); Serial.print(n); Serial.print(F("] output, address: ")); Serial.print(softwareAddress[n], DEC);
        Serial.print(F(" (cfg: "));
        Serial.print(svtable.svt.pincfg[n].cnfg);
        // Serial.print(F(" "));
        // Serial.print(getConfig(svtable.svt.pincfg[n].cnfg)); // adds pin config description - TODO fix
        // add no. 1/2 output pair = .value2 bits 4-7
        int logic = svtable.svt.pincfg[n].value2 & 0xF0;
        if (logic == 3)
        {
          Serial.print(F(" 2 "));
        }
        else if (logic == 1)
        {
          Serial.print(F(" 1 "));
        }
        Serial.println(F(")"));
      }
    }
    else
    {
      Serial.print ("Port"); Serial.print (n); Serial.print(F(" has an unknown setting. Err: cnfg=")); Serial.println(svtable.svt.pincfg[n].cnfg);
    }
  }
}

boolean isValidConfig(int code) {
  for (int i = 0; i < sizeof(configCodes); i++) {
      if (configCodes[i] == code) {
          return true;
      }
    }
  return false;
}

/***************************************************************************************************************************
  Callbacks for SerialCommand prompts
****************************************************************************************************************************/
#ifdef SERIAL_CMD

void turnOn()
{
  uint8_t s_port;
  char *arg;
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it exists, take it
  {
    s_port = atoi(arg);  // convert char string to int
    if (s_port >= 0 && s_port < 16)
    {
        if (bitRead(svtable.svt.pincfg[s_port].cnfg, 7))  // only set outputs
        {
            digitalWrite(pinMap[s_port], HIGH);
            Serial.print(F("Turned on port "));
            Serial.println(s_port);
        }
        else {
            Serial.print(F("Can't set port "));
            Serial.print(s_port);
            Serial.print(F(" because it is an input."));
        }
    }
  }
  else {
      Serial.println(F("LED on"));
      digitalWrite (LocoLED, HIGH);
  }
}

void turnOff()
{
  uint8_t s_port;
  char *arg;
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it exists, take it
  {
    s_port = atoi(arg);  // convert char string to int
    if (s_port >= 0 && s_port < 16)
    {
        if (bitRead(svtable.svt.pincfg[s_port].cnfg, 7))  // only set outputs
        {
            digitalWrite(pinMap[s_port], LOW);
            Serial.print(F("Turned off port "));
            Serial.println(s_port);
        }
        else {
            Serial.print(F("Can't set port "));
            Serial.print(s_port);
            Serial.print(F(" because it is an input."));
        }
    }
    else {
        Serial.print(F("Enter a port number from 0 to 15"));
    }
  }
  else {
      Serial.println(F("LED off"));
      digitalWrite (LocoLED, LOW);
  }
}

void portAddress()
{
  uint8_t s_port;
  uint16_t s_port_addr;
  char *arg;

  // Serial.println(F("Run portAddress()");
  arg = SCmd.next();
  if (arg != NULL)
  {
    s_port = atoi(arg);  // convert char string to int
  }
  else {
    Serial.println(F("Enter a port number (0-1, 8-15)"));
    return;
  }

  arg = SCmd.next();
  if (arg != NULL)
  {
    s_port_addr = atol(arg);  // convert char string to int
    if (0 < s_port_addr <= 2048)
    {
      setPortAddress(s_port, s_port_addr, ((svtable.svt.pincfg[s_port].cnfg & 0x80) == 0));
      Serial.print(F("New address set for "));
    }
    else {
      Serial.print(F("Skipping invalid port address: "));
    }
  }
  else {
    Serial.println(arg);
  }
  // print values
  Serial.print(F("Port "));
  Serial.print(s_port);
  Serial.print(F(" address: "));
  Serial.println(softwareAddress[s_port]);
}

void portFunction()
{
  uint8_t s_port;
  uint16_t s_port_func;
  char *arg;

  // Serial.println(F("Run portFunction()"));
  arg = SCmd.next();
  if (arg != NULL)
  {
    s_port = atoi(arg);  // convert char string to int
  }
  else {
    Serial.println(F("Enter a port number (0-15)"));
    return;
  }

  arg = SCmd.next();
  if (arg != NULL)
  {
    s_port_func = atol(arg);  // convert char string to int
    // validate
    if (isValidConfig(s_port_func)) {
      // update global vars
      svtable.svt.pincfg[s_port].cnfg = s_port_func;
      svtable.data[3 * (s_port + 1)] = s_port_func;
      // set in EEPROM for persistence
      EEPROM.write(3 * (s_port + 1), s_port_func);

      // set blink bit to ON
      if (bitRead(svtable.svt.pincfg[s_port].cnfg, 4)) blinkState[s_port] = 1;
      Serial.print(F("New function set for "));
    }
    else {
      Serial.print(F("Invalid function code: "));
      Serial.println(s_port_func);
      return;
    }
  }
  // print values
  Serial.print(F("Port "));
  Serial.print(s_port);
  Serial.print(F(" function (code): "));
  Serial.println(svtable.svt.pincfg[s_port].cnfg);
}

void portReset()
{
  uint8_t s_port;  // port index, starts at 0
  char *arg;

  // Serial.println(F("Run portReset()"));
  arg = SCmd.next();
  if (arg != NULL)
  {
    s_port = atoi(arg);  // convert char string to int
  }
  else {
    Serial.println(F("Enter a port number (0-15)"));
    return;
  }

  // defaults
  uint16_t s_port_addr = s_port;
  uint16_t s_port_func = 128;  // default: output, default off

  setPortAddress(s_port, s_port_addr, false); // check cnfg bit 7 (input?)

  // set function in global vars
  svtable.svt.pincfg[s_port].cnfg = s_port_func;
  svtable.data[3 * (s_port + 1)] = s_port_func;
  // set in EEPROM for persistence
  EEPROM.write(3 * (s_port + 1), s_port_func);

  Serial.print(F("Port ")); Serial.print(s_port); Serial.print(F(" was reset to "));
  // print values
  Serial.print(F("Port "));
  Serial.print(s_port);
  Serial.print(F(" was reset to address: "));
  Serial.print(softwareAddress[s_port]);
  Serial.print(F(", function: "));
  Serial.println(svtable.svt.pincfg[s_port].cnfg);
}

void moduleAddress()
{
  uint8_t s_board_lo;
  uint8_t s_board_hi;
  char *arg;

  // Serial.println(F("Run moduleAddress()"));
  arg = SCmd.next();
  if (arg != NULL)
  {
    s_board_lo = atoi(arg);  // convert char string to int
    if (s_board_lo > 255 || s_board_lo < 1 || s_board_hi == 80)  // 80/1 is reserved LocoBuffer address
    {
      Serial.print(F("Invalid board low address: "));
      Serial.println(s_board_lo);
      Serial.println(F("Enter a value between 1 and 255, excluding 80."));
      return;
    }

    arg = SCmd.next();  // mod hi address, only for LocoIO GCA50a
    if (arg == NULL) s_board_hi = 1;
    else s_board_hi = atoi(arg);  // convert char string to int
    
    if (s_board_hi > 255 || s_board_hi < 1)
    {
      Serial.print(F("Invalid board high address: "));
      Serial.println(s_board_hi);
      Serial.println(F("Enter a value between 1 and 255 (default: 1)."));
      return;
    }

    // Set address in global vars
    svtable.svt.addr_low = s_board_lo;
    svtable.svt.addr_high = s_board_hi;
    // set in EEPROM for persistence
    EEPROM.write(1, s_board_lo);
    EEPROM.write(2, s_board_hi);

    Serial.println(F("New board address set."));
  }

  // print values
  Serial.print(F("Module lo/hi address: "));
  Serial.print(svtable.svt.addr_low); Serial.print(F("/")); Serial.println(svtable.svt.addr_high);
}

// Board Blink Rate
void moduleBlinkRate()
{
  uint8_t s_board_blink;
  char *arg;

  // Serial.println(F("Run moduleBlinkRate()"));
  arg = SCmd.next();
  if (arg != NULL)
  {
    s_board_blink = atoi(arg);  // convert char string to int
    if (s_board_blink > 15 || s_board_blink < 0)
    {
      Serial.print(F("Invalid board blink rate: "));
      Serial.println(s_board_blink);
      Serial.println(F("Enter a value between 0 (slow) and 15 (fast)."));
      return;
    }

    // Update global vars
    blinkRate = s_board_blink;
    blinkDuration = 1000 - 30 * blinkRate;
    svtable.data[0] = ((blinkRate * 16) | (svtable.data[0] & 0x0F)); // retain bits 0-3
    // set cnfg in EEPROM
    EEPROM.write(0, svtable.data[0]);

    Serial.println(F("New board blink rate set."));
  }

  // print values
  Serial.print(F("Module blink rate: "));
  Serial.println(svtable.data[0] >> 4);
}

void serialHelp()
{
  Serial.println(F("GCA51 Serial Commands Help"));
  Serial.println(F("==="));
  Serial.println(F("H+Enter: Display this command help"));
  Serial.println(F("MA+Enter: Display module board low/high address"));
  Serial.println(F("MA 82 1+Enter: Set module board address to 82/1"));
  Serial.println(F("MB 6+Enter: Set module blink rate to 6"));
  Serial.println(F("MB+Enter: Display module blink rate"));
  Serial.println(F("P 2+Enter: Display software address of port 2"));
  Serial.println(F("P 2 100+Enter: Set software address of port 2 to 100"));
  Serial.println(F("F 2+Enter: Display function of port 2 (code)"));
  Serial.println(F("F 2 128+Enter: Set function of port 2 to 128 =output Off (checks for valid codes: from 15 up to 208)"));
  Serial.println(F("Z 2+Enter: Factory Reset port 2 (address and function)"));
  Serial.println(F("==="));
}

// This gets set as the default handler, and gets called when no other command matches.
// SerialCommand Advanced uses a default handler that receives the command string.
// Use a signature that accepts the command buffer. If your installed SerialCommand
// variant uses a different signature adjust accordingly.
void unrecognized(char *command)
{
  Serial.print(F("What? Unrecognized command: "));
  if (command && command[0]) Serial.println(command);
  else Serial.println();
  Serial.println(F(">> Enter H for Command Help"));
}

// util for SerialCommand callbacks
void setPortAddress(uint8_t s_port, uint16_t s_port_addr, bool isInput)
{
  // split port address
  uint16_t newValue1 = (s_port_addr - 1) & 0x7F;
  uint16_t newValue2 = (s_port_addr & 780) >> 7;  // 4 high bits
  if (isInput) newValue2 = newValue2 | 0x10;  // sensor bit 4 (input)
  if (s_port_addr % 2 == 0) newValue2 = newValue2 | 0x20;  // even bit 5

  svtable.svt.pincfg[s_port].value1 = newValue1; // low byte
  svtable.svt.pincfg[s_port].value2 = newValue2; // high byte, only change bits 0-3
  // update global vars
  softwareAddress[s_port] = s_port_addr;
  svtable.data[(3 * (s_port + 1)) + 1] = newValue1;
  svtable.data[(3 * (s_port + 1)) + 2] = newValue2;
  // set in EEPROM for persistence
  EEPROM.write((3 * (s_port + 1)) + 1, newValue1);
  EEPROM.write((3 * (s_port + 1)) + 2, newValue2);
}
#endif


/********************** SETUP *************************/

void setup()
{
  uint8_t n;
  uint32_t uiStartTimer;
  uint16_t uiElapsedDelay;
  uint16_t uiSerialOKDelay = 5000;
  pinMode (LocoLED, OUTPUT);                    // LocoLED pin to indicate LocoNet communication

  // First initialize the LocoNet interface
  LocoNet.init(LN_TX_PIN); // Use explicit naming of the Tx Pin to avoid confusion

  // Configure the serial port for 57600 baud
#ifdef DEBUG
  Serial.begin(9600);

  Serial.print(F("SVLocoIO (GCA50a) v.")); Serial.println(VERSION);

  if (Serial) { // serial interface OK
    bSerialOk = true;
    Serial.println(F("************************************************"));
  }
#endif

  // Load config from EEPROM
  svtable.svt.board_cnfg = EEPROM.read(0); // contains board blink rate, etc.
  svtable.svt.addr_low = EEPROM.read(1);
  svtable.svt.addr_high = EEPROM.read(2);

#ifdef DEBUG
  Serial.println(F("Start reading EEPROM into svtable.data"));
#endif
  for (n = 0; n < 101; n++) {
    svtable.data[n] = EEPROM.read(n);  // Read the values of SV0 till SV100. The values in EEPROM were OK or standardised in start_setup()
#ifdef DEBUG
    Serial.print(n); Serial.print(F(": ")); Serial.println(svtable.data[n]);
#endif
  }

  // Check for a valid config
  if (svtable.data[100] != VERSION || svtable.svt.addr_low < 1 || svtable.svt.addr_low > 240 || svtable.svt.addr_high < 1 || svtable.svt.addr_high > 100 )
  {
    svtable.data[100] = VERSION;
    svtable.svt.addr_low = ucBoardAddrLo;
    svtable.svt.addr_high = ucBoardAddrHi;
    EEPROM.write(0, 0); // HDL LocoIO compatible board config (stores: refresh at power on, blink rate, etc.)
    EEPROM.write(1, svtable.svt.addr_low);
    EEPROM.write(2, svtable.svt.addr_high);
    EEPROM.write(99, 0); // init JMRI LocoIO decoder storage of o-bits
    EEPROM.write(100, VERSION); // HDL LocoIO compatible SV100, readOnly from LocoNet
    // ReadCV returns offset x, x+1 and x+2 so we simulate returned values in processPeerPacket() for SV > 98

    // reset ports
    uint16_t s_port_func = 128;  // default: output, default off
    for (n = 0; n < 16; n++) {
      setPortAddress(n, n, false);
      // set function in global vars
      svtable.svt.pincfg[n].cnfg = s_port_func;
      svtable.data[3 * (n + 1)] = s_port_func;
      // set in EEPROM for persistence
      EEPROM.write(3 * (n + 1), s_port_func);
    }

    // write 0 to remaining svtable.data & EEPROM 51 - 98
    for (n = 51; n < 99; n++) {
      svtable.data[n] = 0;
      EEPROM.write(n, 0);
    }
    Serial.println(F("Version mismatch; EEPROM was reset to defaults."));
  }
  else
  {
    // Configure I/O
    CalculateAddress(); // Calculate software addresses of pins and store in global variable softwareAddress[16]. Prints config to Console

    // load board settings from SV0
    // from Public_Domain_HDL_LocoIO definition:
    //    <variable CV="0" mask="VVVVXXXX" item="Blink Rate" default="0"> DONE, see blinkRate
    //        <label>Blink Rate:</label> 0=slow to 15=fast
    //    <variable CV="0" mask="XXXVXXXX" item="Board Active High" default="0"> ALWAYS ACTIVE LOW - NO CONFIG
    //        <tooltip>Default: unselected = Active Low</tooltip>
    //    <variable CV="0" mask="XXXXVVXX" item="Action Mode" default="0"> NOT USED - ALWAYS 0
    //    <variable CV="0" mask="XXXXXXVX" item="Alternate Mode" default="0">
    //        0 = Fixed; 1 = Alternating
    //        <tooltip>Button sends alternating or fixed code</tooltip>
    //    <variable CV="0" mask="XXXXXXXV" item="Port Refresh" default="0">

    blinkRate = (svtable.data[0] >> 4);  // actual blinkPeriod was matched to an HDL LocoIO
    blinkDuration = 1000 - 30 * blinkRate; // use 50% of blinkPeriod. See also FlashTime const
#ifdef DEBUG
    Serial.print(F("Board blink rate: ")); Serial.print(blinkRate); Serial.print( ". blink period: "); Serial.print(blinkDuration * 2); Serial.println(F(" ms"));
#endif

    alternateMode = svtable.data[0] & 0x2;
    portRefresh = svtable.data[0] & 0x1;

    Serial.println(F("LocoIO functions compatible with v148/149"));

    // Configure I/O pins and give outputs a start value
#ifdef DEBUG
    Serial.println(F("Initializing pins..."));
#endif 
    for (n = 0; n < 16; n++)
    {
      inpTimer[n] = 0;  // timer initialization

      if (bitRead(svtable.svt.pincfg[n].cnfg, 7))  // Output
      {
        softwareAddress[n] = (svtable.svt.pincfg[n].value2 & B00001111) << 7;
        softwareAddress[n] = softwareAddress[n] | svtable.svt.pincfg[n].value1;
        softwareAddress[n] += 1;
#ifdef DEBUG
        Serial.print(F("Pin ")); Serial.print(pinMap[n]); Serial.print(F(" output ")); Serial.print(n); Serial.print(F(" LOGIC ")); Serial.print(softwareAddress[n]); Serial.println(F(" as OUTPUT"));
#endif
        pinMode(pinMap[n], OUTPUT);
        // IF HIGH at startup AND output type = CONTINUE ...
        if (bitRead(svtable.svt.pincfg[n].cnfg, 0) == 0 && bitRead(svtable.svt.pincfg[n].cnfg, 3) == 0)
          digitalWrite(pinMap[n], HIGH);
        else
          digitalWrite(pinMap[n], LOW);
      }
      else // Input
      {
        softwareAddress[n] = (svtable.svt.pincfg[n].value2 & B00001111) << 7;
        softwareAddress[n] = softwareAddress[n] | (svtable.svt.pincfg[n].value1 << 1 | bitRead(svtable.svt.pincfg[n].value2, 5));
        softwareAddress[n] += 1;
#ifdef DEBUG
        Serial.print(F("Pin ")); Serial.print(pinMap[n]); Serial.print(F(" input ")); Serial.print(n); Serial.print(F(" LOGIC ")); Serial.print(softwareAddress[n]); Serial.println(F(" as INPUT_PULLUP"));
#endif
        pinMode(pinMap[n], INPUT_PULLUP);
        bitWrite(svtable.svt.pincfg[n].value2, 4, digitalRead(pinMap[n]));
      }
    }
  }
  
  Serial.print(F("Module lo/hi address: ")); Serial.print(svtable.svt.addr_low); Serial.print(F("/")); Serial.println(svtable.svt.addr_high);

#ifdef SERIAL_CMD
  // Setup callbacks for SerialCommand commands
  SCmd.addCommand("ON",turnOn);          // No arg. turns on ESP onboard LED/Two num args. turns output on
  SCmd.addCommand("OFF",turnOff);        // No arg. turns off ESP onboard LED/Two num args. turns output off
  SCmd.addCommand("MA",moduleAddress);   // No arg. Reads/Two num args. sets module address and echos new setting
  SCmd.addCommand("MB",moduleBlinkRate); // No arg. Reads/One num args. sets module blink rate and echos new setting
  SCmd.addCommand("P",portAddress);      // One num arg. Reads/Two num args. sets port address and echos new setting
  SCmd.addCommand("F",portFunction);     // One num arg. Reads/Two num args. sets port function and echos new setting
  SCmd.addCommand("Z",portReset);        // One num arg. Resets port address + function to defaults and echos new setting
  SCmd.addCommand("H",serialHelp);       // Display Serial Commands Help
  SCmd.setDefaultHandler(unrecognized);  // Handler for unmatched command (says "What?")
#endif

} // end of setup()

/************************ MAIN LOOP () *************************/

void loop()
{
  uint8_t n;
  bool hasChanged;
  int currentState;
  static unsigned long IO_timing[8];                   // array[8] with Pulse- or Debounce timing for each IO-port
  currentBlinkMillis = millis();                       // capture the latest value of millis()
  static byte remember_input[8];                       // remembers which input was active.  After "waittime" the program will reset this input(s)

  LocoNet_communication(0);                      // turn off LocoLED when the wait time has expired


  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive();
  if (LnPacket)
  {
    #ifdef DEBUG 
    // First print out the packet in HEX
    Serial.print(F("RX: "));
    uint8_t msgLen = getLnMsgSize(LnPacket);
    for (uint8_t x = 0; x < msgLen; x++)
    {
      uint8_t val = LnPacket->data[x];
      // Print a leading 0 if less than 16 to make 2 HEX digits
      if (val < 16) Serial.print(F("0"));
      Serial.print(val, HEX);
      Serial.print(F(" "));
    }
    Serial.println();
    #endif

    // If this packet was not a Switch or Sensor Message, check for PEER packet
    if (!LocoNet.processSwitchSensorMessage(LnPacket))
    {
      processPeerPacket();
    }
  }

#ifdef SERIAL_CMD
  // check for serial commands
  SCmd.readSerial();     // We don't do much, just process serial commands
#endif

  /********************************** OUTPUTS *******************************************
    handled by call-back function notifySwitchRequest to LocoNet.processSwitchSensorMessage
    blinking outputs are checked in the next for loop

  ******************************* HANDLE INPUTS *****************************************
    handles: (delayed) block detectors, toggles/buttons direct/indirect
    TODO switch point feedback contact1/contact 2, alternating code for push buttons (board config)
  */

  for (n = 0; n < 16; n++)
  {
    updateBlink(n);

    if (!bitRead(svtable.svt.pincfg[n].cnfg, 7) && softwareAddress[n] >= 1)  // port is an input
    {
      // Check if state changed 
      currentState = digitalRead(pinMap[n]);
      if (currentState == bitRead(svtable.svt.pincfg[n].value2,4))
      {
        inpTimer[n] = millis();
        continue;
      }
      
      hasChanged = true;
      // Check if port is a BLOCK DETECTOR with DELAYED SWITCH OFF (as we use pullup resistor, deactivation is HIGH)
      if (bitRead(svtable.svt.pincfg[n].cnfg, 4) == 1 && bitRead(svtable.svt.pincfg[n].cnfg, 2) == 0 && currentState == HIGH)
      {
        if ((millis() - inpTimer[n]) < 2000)
          hasChanged = false;
      }

      if (hasChanged)
      {
        LocoNet_communication(1);                                                        // turn on LocoLED
        #ifdef DEBUG
        Serial.print(F("INPUT ")); Serial.print(n);
        Serial.print(F(" IN PIN ")); Serial.print(pinMap[n]);
        #endif
        if (bitRead(svtable.svt.pincfg[n].cnfg, 6)) // configuration "Active High" is On
        {
          if (currentState == 0) {  // sensor / key activated
            #ifdef DEBUG
            Serial.print(" CHANGED, INFORM "); Serial.println(softwareAddress[n]);
            #endif
            LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);
          } else {                // sensor / key released
            if (bitRead(svtable.svt.pincfg[n].value2, 5)) {
              #ifdef DEBUG
              Serial.print(" CHANGED, INFORM "); Serial.println(softwareAddress[n] - 1);
              #endif
              LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2 & 0xDF);
            } else {
              #ifdef DEBUG
              Serial.print(" CHANGED, INFORM "); Serial.println(softwareAddress[n] + 1);
              #endif
              LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2 | 0x20);
            }
          }
        } else {
          #ifdef DEBUG
          Serial.print(F(" CHANGED, INFORM ")); Serial.print(softwareAddress[n]);
          Serial.print(F(" new state: ")); Serial.println(currentState);
          #endif
          LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);
        }
        // Update state to detect flank (use bit in value2 of SV)
        bitWrite(svtable.svt.pincfg[n].value2, 4, currentState);
      }

    }
  }
    
} // end of main loop()

/*************************************************************************/
/*          LOCONET FUNCTIONS                                            */
/*************************************************************************/
// Functions common with GCA51 moved to SVLocoIO.cpp

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch Request messages
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int n;
  
  // Direction must be changed to 0 or 1, not 0 or 32
  Direction ? Direction = 1 : Direction = 0;

#ifdef DEBUG
  Serial.print(F("Switch Request: "));
  Serial.print(Address, DEC);
  Serial.print(F(":"));
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(F(" - "));
  Serial.println(Output ? "On" : "Off");
#endif
  
  // Check if the Address is assigned, configured as output and same Direction
  for (n = 0; n < 16; n++)
  {
    if ((softwareAddress[n] == Address) &&   // Address
        (bitRead(svtable.svt.pincfg[n].cnfg, 7) == 1))   // Setup as an Output
    {
#ifdef DEBUG
      Serial.print(F("Output assigned to port "));
      Serial.print(n + 1); Serial.print(F(" and pin ")); Serial.println(pinMap[n]);
#endif
      // If pulse (always hardware reset) and Direction, only listen ON message
      if (bitRead(svtable.svt.pincfg[n].cnfg, 3) == 1 && bitRead(svtable.svt.pincfg[n].value2, 5) == Direction && Output)
      {
        digitalWrite(pinMap[n], HIGH);
        delay(150);
        digitalWrite(pinMap[n], LOW);
        break;
      }
      // If continue and hardware reset and Direction
      else if (bitRead(svtable.svt.pincfg[n].cnfg, 3) == 0 && bitRead(svtable.svt.pincfg[n].cnfg, 2) == 1 && bitRead(svtable.svt.pincfg[n].value2, 5) == Direction)
      {
        if (Output)
          digitalWrite(pinMap[n], HIGH);
        else
          digitalWrite(pinMap[n], LOW);
        break;
      }
      // If continue and software reset, one Direction ON turns on and other Direction ON turns off
      // OFF messages are not listened for
      else if (bitRead(svtable.svt.pincfg[n].cnfg, 3) == 0 && bitRead(svtable.svt.pincfg[n].cnfg, 2) == 0 && Output)
      {
        if (!Direction)
          digitalWrite(pinMap[n], HIGH);
        else
          digitalWrite(pinMap[n], LOW);
        break;
      }
    }
  }
}

boolean processPeerPacket()
{
  // Check it is an OPC_PEER_XFER message
  if (LnPacket->px.command != OPC_PEER_XFER) return (false);

  // Check it is my destination
  if ((LnPacket->px.dst_l != 0 || LnPacket->px.d5 != 0) &&
      (LnPacket->px.dst_l != 0x7f || LnPacket->px.d5 != svtable.svt.addr_high) &&
      (LnPacket->px.dst_l != svtable.svt.addr_low || LnPacket->px.d5 != svtable.svt.addr_high))
  {
#ifdef DEBUG
    Serial.println(F("OPC_PEER_XFER not for me!"));
    Serial.print(F("LnPacket->px.dst_l: ")); Serial.print(LnPacket->px.dst_l); Serial.print(F(" Addr low: ")); Serial.println(svtable.svt.addr_low);
    Serial.print(F("LnPacket->px.d5: ")); Serial.print(LnPacket->px.d5); Serial.print(F(" Addr high: ")); Serial.println(svtable.svt.addr_high);
    Serial.print(F("LnPacket->px.dst_h: ")); Serial.print(LnPacket->px.dst_h); Serial.print(F(" Addr high: ")); Serial.println(svtable.svt.addr_high);
    Serial.print(F("LnPacket->px.d1: ")); Serial.println(LnPacket->px.d1);
    Serial.print(F("LnPacket->px.d2: ")); Serial.println(LnPacket->px.d2);
#endif
    return (false);
  }

  //Set high bits in correct position
  bitWrite(LnPacket->px.d1, 7, bitRead(LnPacket->px.pxct1, 0));
  bitWrite(LnPacket->px.d2, 7, bitRead(LnPacket->px.pxct1, 1));
  bitWrite(LnPacket->px.d3, 7, bitRead(LnPacket->px.pxct1, 2));
  bitWrite(LnPacket->px.d4, 7, bitRead(LnPacket->px.pxct1, 3));

  bitWrite(LnPacket->px.d5, 7, bitRead(LnPacket->px.pxct2, 0));
  bitWrite(LnPacket->px.d6, 7, bitRead(LnPacket->px.pxct2, 1));
  bitWrite(LnPacket->px.d7, 7, bitRead(LnPacket->px.pxct2, 2));
  bitWrite(LnPacket->px.d8, 7, bitRead(LnPacket->px.pxct2, 3));

  //OPC_PEER_XFER D1 -> Command (1=SV_write, 2=SV_read)
  //OPC_PEER_XFER D2 -> The Register (SV) to read or write

  if (LnPacket->px.d1 == 2) // Read
  {
#ifdef DEBUG
    Serial.print(F("READ ")); Serial.print(LnPacket->px.d2); Serial.print(F(" ")); Serial.print(LnPacket->px.d2 + 1); Serial.print(F(" ")); Serial.println(LnPacket->px.d2 + 2);
#endif

    if (LnPacket->px.d2 >= 0) // SV0 contains board config, SV100 is highest SV on LocoIO/GCA51
    {
      if (LnPacket->px.d2 < 99)
      {
        sendPeerPacket(svtable.data[LnPacket->px.d2], svtable.data[LnPacket->px.d2 + 1], svtable.data[LnPacket->px.d2 + 2]);
        return (true);
      } else if (LnPacket->px.d2 == 99) { // A readReply always includes the d2+1 and d2+2 values...
        sendPeerPacket(svtable.data[LnPacket->px.d2], svtable.data[LnPacket->px.d2 + 1], 0);
        return (true);
      } else if (LnPacket->px.d2 == 100) {  // ...so to read SV100 we must add 2 fake values in the reply
        sendPeerPacket(svtable.data[LnPacket->px.d2], 0, 0);
        return (true);
      } else {
        Serial.print(F("Read offset is outside valid range (0-100): ")); Serial.println(LnPacket->px.d2);
      }
    }
  }

  if (LnPacket->px.d1 == 1) // Write
  {

    if (LnPacket->px.d2 >= 0 && LnPacket->px.d2 < 100) // SV 0 contains board config, SV100 (Version) is read only
    {
      // Store data
      svtable.data[LnPacket->px.d2] = LnPacket->px.d4;
      // set in EEPROM for persistence
      EEPROM.write(LnPacket->px.d2, LnPacket->px.d4);

#ifdef DEBUG
      Serial.print(F("WROTE ")); Serial.print(LnPacket->px.d2); Serial.print(F(" <== "));
      Serial.print(LnPacket->px.d4); Serial.print(F(" | "));
      Serial.print(LnPacket->px.d4, HEX); Serial.print(F(" | "));
      Serial.println(LnPacket->px.d4, BIN);
#endif
    } else {
      Serial.print(F("Write offset is outside valid range (0-99): ")); Serial.println(LnPacket->px.d2);
    }

    // Reply packet
    LocoNet_communication(1); // turn on LocoLED
    sendPeerPacket(0x00, 0x00, LnPacket->px.d4);
#ifdef DEBUG
    Serial.println(F(">> OPC_PEER_XFER reply sent"));
#endif
    return (true);
  }

  return (false);
}

void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2)
{
  lnMsg txPacket;

  txPacket.px.command = OPC_PEER_XFER;
  txPacket.px.mesg_size = 0x10;
  txPacket.px.src = svtable.svt.addr_low;
  txPacket.px.dst_l = LnPacket->px.src;
  txPacket.px.dst_h = LnPacket->px.dst_h;
  txPacket.px.pxct1 = 0x00;
  txPacket.px.d1 = LnPacket->px.d1;       // Original command
  txPacket.px.d2 = LnPacket->px.d2;       // SV requested
  txPacket.px.d3 = svtable.data[100];
  txPacket.px.d4 = 0x00;
  txPacket.px.pxct2 = 0x00;
  txPacket.px.d5 = svtable.svt.addr_high;  // SOURCE high address
  txPacket.px.d6 = p0;
  txPacket.px.d7 = p1;
  txPacket.px.d8 = p2;

  // Set high bits in correct position
  bitWrite(txPacket.px.pxct1, 0, bitRead(txPacket.px.d1, 7));
  bitClear(txPacket.px.d1, 7);
  bitWrite(txPacket.px.pxct1, 1, bitRead(txPacket.px.d2, 7));
  bitClear(txPacket.px.d2, 7);
  bitWrite(txPacket.px.pxct1, 2, bitRead(txPacket.px.d3, 7));
  bitClear(txPacket.px.d3, 7);
  bitWrite(txPacket.px.pxct1, 3, bitRead(txPacket.px.d4, 7));
  bitClear(txPacket.px.d4, 7);
  bitWrite(txPacket.px.pxct2, 0, bitRead(txPacket.px.d5, 7));
  bitClear(txPacket.px.d5, 7);
  bitWrite(txPacket.px.pxct2, 1, bitRead(txPacket.px.d6, 7));
  bitClear(txPacket.px.d6, 7);
  bitWrite(txPacket.px.pxct2, 2, bitRead(txPacket.px.d7, 7));
  bitClear(txPacket.px.d7, 7);
  bitWrite(txPacket.px.pxct2, 3, bitRead(txPacket.px.d8, 7));
  bitClear(txPacket.px.d8, 7);

  LocoNet_communication(1); // turn on LocoLED
  LocoNet.send(&txPacket);

#ifdef DEBUG
  Serial.println(F("OPC_PEER_XFER Packet sent!"));
#endif
}

/*********************************************************************************************************************
  Purpose:      Handle blink timer if output commanded state is ON. Turn output off if commanded state is OFF.
  Description : Adapted from GCA51 v151 LocoIO (n=0; n<16) and .cfg bit 4 checked. New in SVLocoIO v107

  Globals:
  blinkRate (Board setting) is in range 0 - 15
  blinkPeriod is in range 2000 ms (@0) - 250 ms (@15)
  blinkDuration = blinkPeriod/2

  Blink config bit 4:
  0d144 = 0b10010000
  0d145 = 0b10010001
  0d208 = 0b11010000
  No blink:
  0d128 = 0b10000000
  0d129 = 0b10000001
**********************************************************************************************************************/
void updateBlink(uint8_t portIdx) {

  if (bitRead(svtable.svt.pincfg[portIdx].cnfg, 7) && bitRead(svtable.svt.pincfg[portIdx].cnfg, 4))  // only blink for outputs, cnfg 144/145/208
  {
    if (bitRead(svtable.svt.pincfg[portIdx].value2, 4) == HIGH) // only blink when output is ON
    {
      if (blinkState[portIdx] == 1)
      {
        // if output is currently off, wait for the interval to expire before turning it on
        if (currentBlinkMillis - previousBlinkMillis >= blinkDuration)
        { // time is up, so change the state to LOW (on)
          digitalWrite(pinMap[portIdx], LOW);
          blinkState[portIdx] = 0;
          previousBlinkMillis = previousBlinkMillis + blinkDuration; // save the time when we changed to on
        }
      }
      else if (blinkState[portIdx] == 0)
      {
        // if output is currently on, wait for the duration to expire before turning it off
        if (currentBlinkMillis - previousBlinkMillis >= blinkDuration)
        { // time is up, so change the state to HIGH (off)
          digitalWrite(pinMap[portIdx], HIGH);
          blinkState[portIdx] = 1;
          previousBlinkMillis = previousBlinkMillis + blinkDuration; // save the time when we changed to off
        }
      } else {
        Serial.print(F("Unexpected blinkState ")); Serial.print(blinkState[portIdx]);
        Serial.print(F(" for Port ")); Serial.println(portIdx);
      }
    } else { // output commanded state if HIGH (off) so turn off pin
      digitalWrite(pinMap[portIdx], HIGH);
      blinkState[portIdx] == 0;
    }
  }
}
