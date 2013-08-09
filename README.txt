******************************************************
*********************** README ***********************
******************************************************

	RoboCore Albatross Master
		(v1.0 - 09/08/2013)

  Program to use with the Albatross Master Shield from RoboCore
    (for Arduino 1.0.1 and later)

  Copyright 2013 RoboCore (François) ( http://www.RoboCore.net )
  
  ------------------------------------------------------------------------------
  This file is part of Albatross Master.

  Albatross Master is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Albatross Master is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with Albatross Master.  If not, see <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------------------
  
  This program is supports (master version + slaves versions):
    - Albatross Master Shield v1.0
          + Albatross Slave Relay v1.0
  
  The program is used to execute commands via OSC messages sent in the local network.
  The included commands are:
    - toggle on/off an IO port in a Albatross Slave Relay;
    - pulse an IO port in a Albatross Slave Relay;
    - configure a Albatross as Master or Slave XBee;
    - record an IR command;
    - send an IR command (optionally repeat the command);
    - perform a test;
  The Serial is used to display the status of the program but IS NOT necessary for the
    program to run. One can also use the Serial to execute special commands:
    - auto_config >> automatically configure the XBee as Master by scanning through
                      the possible baudrate values;
    - config_slave:<slave_type>
                  >> configure the XBee as slave (presumed the correct baudrate was
                      already set);
    - restore >> restore the XBee's parameter to their factory settings;
    - version >> display the version of the current program;
    - ipconfig >> display the local network configuration;
    - xconfig >> display the XBee network configuration;
    - ip=<ip> >> set the IP of the Albatross Master in the local network;
    - mac=<mac> >> set the MAC of the Albatross Master in the local network;
    - server_port=<server port> >> set the Server Port of the Albatross Master in the local network;
    - client_port=<client port> >> set the Client Port of the Albatroos Master in the local network;
    - id=<id> >> set the XBee network ID;
    - channel=<channel> >> set the XBee network Channel.
    
    NOTE: # after changing ANY of the local network configurations, the Albatross Master must be
            manually RESET to apply the changes;
          # the XBee network ID and Channel must be set BEFORE configuring ANY XBeedevice.




************************************** Pins **************************************
(defined in <VersionPins.h>)

# fixed

ISCP 1, 3, 4 ..... SPI
0, 1 ............. Computer communication (Serial)
2 ................ W5100 interrupt
4 ................ SD SS
10 ............... Ethernet shield SS
11, 12, 13 ....... SPI shield
46 ............... IRremote Send
50, 51, 52, 53 ... SPI Mega


# defined

18,19 ............ XBee communication (Serial1)
48 ............... IRremote Receiver
43 ............... LED RGB Red
45 ............... LED RGB Blue
			(LED RGB Green directly connected to VCC)
			# NOTE: if using SMD, use a separate LED
				for Power




************************************** Library changes **************************************

In order to use the XBee Master shield, one must do the following
changes in the libraries (only to compile the module's code):

	> ArdOSC
		- <Pattern.h>		#define kMaxPatternMatch from 50 to 20

	> IRremote
		- <IRremote.cpp>	in IRrecv::decode(), comment from first
						#ifdef DEBUG to the line before
						if(decodeHash(results))
		- <IRremote.h>		#define RAWBUF from 100 to 230
		- <IRremoteInt.h>	#define _GAP from 5000 to 6000
					comment #define IR_USE_TIMER2 (for Mega)
					uncomment #define IR_USE_TIMER5 (for Mega)




************************************** Libraries **************************************

- <SPI.h>		(comes with Arduino 1.0.1)
- <SD.h>		(comes with Arduino 1.0.1)
- <Ethernet.h>		(comes with Arduino 1.0.1)
- <EEPROM.h>		(comes with Arduino 1.0.1)
- <ArdOSC.h>		RoboCore version v1.0 (modified from Recotana's)
- <IRremote.h> *	Ken Shirriff (https://github.com/shirriff/Arduino-IRremote)
- <Memory.h>		RoboCore - v1.3
- <String_Functions.h>	RoboCore - v1.3
- <Hex_Strings.h>	RoboCore - v1.2
- <XBee_API.h>		RoboCore - v1.4

	* small modifications to use with XBee Master module (not enough to
		create a different library)





************************************** Observations **************************************

# During setup, the RGB LED turns White, then Green when the setup is complete.
	If the SD card was not initialized during this configuration, the LED
	will blink Yellow before turning to Green.

# If the SD card was not initialized and a function that uses it was called
	(ex Record), the RGB LED will blink Yellow to inform the user.

# To use the serial commands, the command must be sent with the End Of Line
	character (ex: "auto_config#").
	NOTE: the commands and the EOL character are defined in the Variables
		section of the code

# The baudrate used by the XBee Master Shield to communicate with the Slave XBee
	and the computer are defined in the <XBee_API.h> library. Since the XBee
	can have a different value from the one defined, one must first set the
	value to match the one defined. This is done by using the X-CTU software
	or by calling the automatic configuration command through serial.
	TIP: because the Slave and the Master must have the same baudrate, one
		can also use the automatic configuration for the slave and call
		afterwards the OSC or serial command to configure as slave.

# When the XBee is initialized or a HardwareSerial is assigned to it, it begins
	a new communication using its defined baudrate. Therefore one must pay
	attention when using the serial with the XBee.

# Since XBee is initialized with its defined baudrate, there can be a communication
	problem if the XBee's baudrate wasn't set to match the defined value (see
	the baudrate note above).

# Only 2 XBee outuputs are defined in PULSE and RELAY TOGGLE, because only two relays
	are available on the Slave board. Additionally, there are two inputs in the
	Slave module for sensors and already connected to the XBee. However, the code
	to read the values from them is not yet implemented.

# DEVELOPER note
	- currently RELAY, PULSE and RECORD are universal (version independent).
		> RELAY and PULSE are directly associated with the XBee Slave's pins and
		depend only of the OSC message sent. Therefore the OSC message depends
		on the version of the Slave.
		> RECORD can only be used in one way because the XBee Master must be used
		with the Ethernet Shield, leavind only the 
	- SEND can only be used in one


******************************************************************************************




