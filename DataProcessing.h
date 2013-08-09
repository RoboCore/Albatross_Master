#ifndef RC_AS_DATA_PROCESSING_H
#define RC_AS_DATA_PROCESSING_H

/*
	RoboCore Albatross Data Processing
                     (v1.0 - 30/07/2013)

  The function to process incoming data from Slave
  modules of the Albatross ( http://www.RoboCore.net )

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

#define AS_DP_RECEIVED_TEST //uncomment to see the data received


// Process incoming data (user defined)
void DataProcessing(char *address_64bit, ByteArray *data){
  //check if valid
  if((address_64bit == NULL) | (data == NULL) || (data->length == 0))
    return;

#ifdef AS_DP_RECEIVED_TEST
  Serial.println("\n>> Received Test");
  Serial.println(address_64bit);
  DisplayByteArray(&Serial, data, true);
#endif
  
  //--- change from here ---
  
  
  //--------- end ----------
}


#endif // RC_AS_DATA_PROCESSING_H




