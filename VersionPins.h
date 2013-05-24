#ifndef RC_XMS_VERSION_PINS_H
#define RC_XMS_VERSION_PINS_H

/*
	RoboCore XBee Master Shield Pins

  Pins of the XBee Master Shield from RoboCore ( http://www.RoboCore.net )

  Released under the Beerware licence
  Written by François
  
  
  Definition of the custom pins for each version of the Master Shield
  
  !!! One must know what to do before changing the values for the pins
  
*/


#define XMS_VERSION_MAIN 1 //current version being used
#define XMS_VERSION_SUB 0 //current version being used


#if (XMS_VERSION_MAIN == 1) && (XMS_VERSION_SUB == 0) // for version 1.0
  #define XMS_SS_HARDWARE  53 // 10 on UNO
  
  #define XMS_SS_ETHERNET  10 // Ethernet Shield
  #define XMS_SS_SD         4 // Ethernet Shield

  #define XMS_XBEE_SERIAL  &Serial1
  #define XMS_IR_RECEIVER  48
  #define XMS_LED_RED      43
  #define XMS_LED_BLUE     45
#endif


#endif //RC_XMS_VERSION_PINS_H




