#ifndef RC_AM_MODULE_HASH_H
#define RC_AM_MODULE_HASH_H

/*
	RoboCore Albatross Master Module Hash
                     (v1.0 - 01/08/2013)

  Hash function for the Master and Slave of the Albatross ( http://www.RoboCore.net )

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


// Get the hash for the module
unsigned long ModuleHash(char *module){
  int length = StrLength(module);
  
  //check for valid name
  if(length == 0)
    return 0;
  
  unsigned long hash = 0;
  word square;
  for(int i=0 ; i < length ; i++){
    square = (i+1) * (i+1);
    hash += ((byte)module[i] - 32) * square;
  }
  
  return (hash & 0xFFFFFFFF);
}


#endif // RC_AM_MODULE_HASH_H




