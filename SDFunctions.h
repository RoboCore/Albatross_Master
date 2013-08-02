#ifndef RC_AM_SD_FUNCTIONS_H
#define RC_AM_SD_FUNCTIONS_H

/*
	RoboCore Albatross Master SD Functions
                     (v1.0 - 01/08/2013)

  Functions to handle SD data of the Albatross Master ( http://www.RoboCore.net )

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

#define FILE_NAME_SIZE 30
#define STR_SIZE_EXTENSION 4 //size of the extension string (ex: ".rbc")

typedef struct {
  char *folder;
  byte folder_size;
  char *extension;
} FolderData;

char file_name[FILE_NAME_SIZE];

// **************************************************************************

// Build file name for SD
boolean BuildFileName(FolderData *folder_data, char *fname){
  boolean res = false;
  
  // --- set folder ---
  for(int i=0 ; i < folder_data->folder_size ; i++)
    file_name[i] = folder_data->folder[i];
  
  // --- set name ---
  int length = StrLength(fname);
  
  if((length > 0) && (length < (FILE_NAME_SIZE - folder_data->folder_size - STR_SIZE_EXTENSION - 1))){
    //insert the name into the buffer and add the extension
    file_name[folder_data->folder_size + length + STR_SIZE_EXTENSION] = '\0'; //NULL terminated string
    for(int i=0 ; i < length ; i++)
      file_name[i + folder_data->folder_size] = fname[i];
    for(int i=0 ; i < STR_SIZE_EXTENSION ; i++)
      file_name[i + folder_data->folder_size + length] = folder_data->extension[i];
    
    res = true;
  }
  
  return res;
}

// **************************************************************************

#endif // RC_AM_SD_FUNCTIONS_H




