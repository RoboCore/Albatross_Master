#ifndef RC_AS_CONFIGURATION_H
#define RC_AS_CONFIGURATION_H

/*
	RoboCore Albatross Slave Configuration
                     (v1.0 - 01/08/2013)

  Pin configuration of the slave modules from Albatross ( http://www.RoboCore.net )

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
  
  The Albatross slave types.
  If a slave can be configured with ConfigureSlavePins(), it must also
    exist in SlaveTypeExists().
  
*/


// **************************************************************************
// ********************************** HASH **********************************

#define RCA_RELAY_1_0  33095
#define RCA_RFID_1_0   25156


// **************************************************************************

// Configure the slave according to its type
byte ConfigureSlavePins(XBeeMaster *xbee, word slave_type){
  XBeePin pins[9]; // D0 - D8
  byte num_pins = 0;
  int res = 0;

  //assigne the pins
  switch(slave_type){
    
#if defined(RCA_RELAY_1_0)
    case RCA_RELAY_1_0:
      pins[0].pin = D0;
      pins[0].value = XBEE_PIN_DO_LOW;
      pins[1].pin = D1;
      pins[1].value = XBEE_PIN_DO_LOW;
      num_pins = 2; //update
      break;
#endif

#if defined(RCA_RFID_1_0)
    case RCA_RFID_1_0:
    // TO DO **************
    //  will have a relay?
    //    > if not, res=-29 do that it becomes 1 after ConfigurePins()
//      pins[0].pin = D0;
//      pins[0].value = XBEE_PIN_DO_LOW;
//      num_pins = 1; //update
      break;
#endif
  
  }

  //configure the pins
  res += xbee->ConfigurePins(pins, num_pins);
  
  return res;
}

// **************************************************************************

// Check if a slave type exists
boolean SlaveTypeExists(word slave_type){
  boolean exists = false;
  
  switch(slave_type){
#if defined(RCA_RELAY_1_0)
    case RCA_RELAY_1_0:  exists = true; break;
#endif
#if defined(RCA_RFID_1_0)
    case RCA_RFID_1_0:   exists = true; break;
#endif
  }
  
  return exists;
}

// **************************************************************************

#endif // RC_AS_CONFIGURATION_H




