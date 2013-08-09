#ifndef RC_AS_ACCESS_CONTROL_H
#define RC_AS_ACCESS_CONTROL_H

/*
	RoboCore Albatross Access Control
                     (v1.0 - 01/08/2013)

  The function to process incoming data from Slave
  modules of the Albatross ( http://www.RoboCore.net )
  that might be used for access.

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
  
*/

/*
    **********************
    ***** Operation ******  REMOVE THIS COMMENT AFTER IMPLEMENTED
    **********************
    
    1) Slave configured as RCA_RFID_1_0
    2) wait for OSC message of DATA
        if OSC
          3) add >> open or create the file
          3) remove >> remove single or all entries
        if DATA
          3) pulse the relay if necessary
*/


/*
    ***************************
    ***** File Structure ******
    ***************************
    
    <serial_number>.csv
    #<num_entries>
    <RFID>;<destination_address>;<relay_number>
        NOTE: if <destination_address> == 't', it
              means that destination = source
    
    ex:
    0013A200409FAA25.csv
    #3
    1155AAFF;0013A200409FAA1A;1;0013A200409FAA1A;0
    2266BBCC;t;1
    10CAF3A9;0013A200409FAA1A;0;t;1
*/




#define AS_AC_RECEIVED_TEST //uncomment to see the data received

#define AS_SACD_STATE_EXECUTE     0
#define AS_SACD_STATE_ADD         1
#define AS_SACD_STATE_REM_SINGLE 10
#define AS_SACD_STATE_REM_ALL    11

struct SACD{
  byte state; // 0 - execute | 1 - add | 10 - remove SINGLE | 11 - remove ALL
  byte relay; // relay number or AS_SACD_NO_DATA
  ByteArray serial_number;
} AccessControlData;

// **************************************************************************

// Process incoming data for access
//  (returns 0 if nothing happened, 1 if executed RFID function)
byte AccessControl(char *address_64bit, ByteArray *data){
  byte res = 0;
  
  //check if valid
  if((address_64bit == NULL) | (data == NULL) || (data->length == 0))
    return res;

#ifdef AS_AC_RECEIVED_TEST
  Serial.println("\n>> Received Test");
  Serial.println(address_64bit);
  DisplayByteArray(&Serial, data, true);
#endif

  //NOTE: the ID in data might have non-printable characters
  //      at the end, like CR (carriage return - 13)
  //      and/or LF (line feed - 10)

  switch(AccessControlData.state){
    case AS_SACD_STATE_EXECUTE:
      // 1) open file named <address_64bit>.csv (might not have one)
      // 2) find the ID (available in <data>)
      //  if found, do for ALL destinations:
      //    3) parse the destination + relay
      //    4) pulse the relay
      //    5) res = 1
      break;
    
    case AS_SACD_STATE_ADD:
      // 1) open/create file named <address_64bit>.csv
      // 2) find the ID (available in <data>)
      //  if found:
      //    3) combine & add destination + relay
      //  else
      //    3.1) add new line with destination + relay combined
      //    3.2) update the number of entries
      // 4) return state to AS_SACD_STATE_EXECUTE
      // 5) res = 1
      break;
    
    case AS_SACD_STATE_REM_SINGLE:
      // 1) open file named <address_64bit>.csv
      // 2) find the ID (available in <data>)
      //  if found:
      //    2.1) find & remove the destination
      //    2.2) update the number of entries IF NECESSARY
      // 3) return state to AS_SACD_STATE_EXECUTE
      // 4) res = 1
      break;
    
    case AS_SACD_STATE_REM_ALL:
      // 1) open file named <address_64bit>.csv
      // 2) find & remove the ID (available in <data>)
      // 3) return state to AS_SACD_STATE_EXECUTE
      // 4) res = 1
      break;
  }
  
  return res;
}

// **************************************************************************

#endif // RC_AS_ACCESS_CONTROL_H




