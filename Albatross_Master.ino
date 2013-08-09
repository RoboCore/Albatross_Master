
/*
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
  
*/


#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <EEPROM.h>

#include <ArdOSC.h> // RoboCore's version v1.0
#include <IRremote.h> //with some small modifications

#include <Memory.h> //v1.3
#include <String_Functions.h> //v1.3
#include <Hex_Strings.h> //v1.2
#include <XBee_API.h> //v1.4

#include "SDFunctions.h"
#include "VersionPins.h"
#include "ModuleHash.h"
#include "ConfigureSlavePins.h"
#include "AccessControl.h"
#include "DataProcessing.h"

// DELAY MACRO *************************************************************************

#define RCdelay(x) ({ \
  unsigned long st = millis(); \
  while(millis() < (st + x)) { /* do nothing */ } \
})

// TIMER 1 MACROS **********************************************************************

//  TCCR1B = 0x00; .............. Disbale Timer1 while we set it up
//  TCNT1  = TIMER1_TCNT1; ...... Reset Timer Count to 49536 out of 65536
//  TIFR1  = 0x00; .............. Timer1 INT Flag Reg: Clear Timer Overflow Flag
//  TIMSK1 = 0x01; .............. Timer1 INT Reg: Timer1 Overflow Interrupt Enable
//  TCCR1A = 0x00; .............. Timer1 Control Reg A: Normal port operation, Wave Gen Mode normal
//  TCCR1B = 0x01; .............. Timer1 Control Reg B: Timer Prescaler set to 1 (an Timer ON)


#define TIMER1_TCNT1 49536 //timer count (49536 of 65536)

//configure Timer 1 (16 bits) to 1 ms
//  (Arduino > 16 MHz and Prescaler de 1)
#define TIMER1_CONFIGURE() ({ \
  TCCR1B = 0x00; \
  TCNT1  = TIMER1_TCNT1; \
  TIFR1  = 0x00; \
  TIMSK1 = 0x01; \
  TCCR1A = 0x00; \
})

#define TIMER1_DISABLE (TCCR1B = 0x00)

#define TIMER1_ENABLE ({ \
  TCNT1  = TIMER1_TCNT1; \
  TIFR1  = 0x00; \
  TCCR1B = 0x01; \
})

#define TIMER1_RESET ({ \
  TCNT1  = TIMER1_TCNT1; \
  TIFR1  = 0x00; \
})


// VARIABLES ***************************************************************************

#define SS_HARDWARE AMS_SS_HARDWARE //10 on UNO

// ----------------------
// ETHERNET
const int SSpin_Ethernet = AMS_SS_ETHERNET; //of Ethernet Shield

byte myMAC[] = { 0xA1, 0xBA, 0x78, 0x05, 0x50, 0x00 };
byte myIP[]  = { 192, 168, 0, 99 };
uint16_t serverPort  = 4444;
uint16_t clientPort = 1111;

// ----------------------
// SD
const int SSpin_SD = AMS_SS_SD; //of Ethernet Shield
SDClass mySD; //cannot directly use 'SD' because of XBee's AT command **** MODIFY

FolderData IRfolder = { "IR/" , 3 , ".rbc" };
FolderData RFIDfolder = { "Access/" , 7 , ".csv" };

extern char file_name[];

boolean SDinitialized; //false when couldn't initialize the SD

// ----------------------
// OSC
OSCServer server;
OSCClient client;

// ----------------------
// IR
// Sender Pin if by default pin 46 on Mega 2560 (Timer 5)
const int pinIRreceiver = AMS_IR_RECEIVER;

IRrecv IRreceiver(pinIRreceiver);
IRsend IRsender;

#define RECORDING_TIMEOUT 5000 //timeout for the recording
boolean SaveIR; //TRUE when waiting for the command name
unsigned int IRlength; //length of the IR command
unsigned int IRbuffer[RAWBUF]; //values of the IR command (already converted)

// ----------------------
// Serial commands
#define EOL_CHAR '#'
#define BUFFER_SIZE 30
char buffer[BUFFER_SIZE]; //buffer to read from Serial
const char Version[] = "version";
const char InvalidCommand[] = "Invalid command!";

// XBee Network
const char AutoConfigure[] = "auto_config";
const char ConfigureSlave[] = "config_slave";
const char Restore[] = "restore";
const char XBeeConfig[] = "xconfig";
const char XNetwork_ID[] = "id";
const char XNetwork_Channel[] = "channel";

// Local Network
const char IPconfig[] = "ipconfig";
const char LNetwork_mac[] = "mac";
const char LNetwork_ip[] = "ip";
const char LNetwork_serverPort[] = "server_port";
const char LNetwork_clientPort[] = "client_port";

// ----------------------
// EEPROM

#define EEPROM_NETWORK_ADDRESS_START 100 // CAREFUL not to collide with other EEPROM structures
#define EEPROM_NETWORK_READ 13

#define EEPROM_XBEE_ADDRESS_START 150 // CAREFUL not to collide with other EEPROM structures
#define EEPROM_XBEE_READ 13

/*
  Structures
  
  EEPROM_NETWORK_ADDRESS_START = EEPROM_NETWORK_READ
    + [1-6]   = myMAC
    + [7-10]  = myIP
    + [11-12] = serverPort
    + [13-14] = clientPort
    
  EEPROM_XBEE_ADDRESS_START = EEPROM_XBEE_READ
    + [1]   = Channel
    + [2-3] = ID
*/

// ----------------------
// Other
XBeeMaster XBee(AMS_XBEE_SERIAL);
const int pinLEDRed = AMS_LED_RED;
const int pinLEDBlue = AMS_LED_BLUE;
//const int pinLEDGreen = 0; // for SMD version
//const int pinERROR = 0; // for SMD version - preferably red

#define DEFAULT_PULSE_TIME 500 // in [ms]
#define MIN_PULSE_TIME 100 // in [ms]
#define MAX_PULSE_TIME 1000 // in [ms]

byte LEDstate = LOW;
int LEDpin = -1; //(-1) if none or the pins assigned to the LED
int LEDtime; //desired time (in ms)
int LEDms; //elapsed time (DO NOT CHANGE)

#define pinRelayToggle pinLEDRed //same for Toggle and Pulse
#define pinConfigure pinLEDRed
#define pinIRRecord pinLEDBlue
#define pinIRSend pinLEDBlue
#define pinTest pinLEDRed

extern struct SACD AccessControlData;


//**************************************************************************************
//******************************* Arduino SPECIFIC *************************************
//**************************************************************************************

// SETUP *******************************************************************************

void setup(){
  // during setup the RGB LED is White (Green + Blue + Red) and turns to Green
  //   when complete.
  // IF the SD card was not initialized, the LED will blink Yellow (Green + Red)
  //    for about 3 seconds, time to let the user know an error has ocurred.
  
  // --- configure Timer 1 ---
  TIMER1_CONFIGURE(); //configure Timer1
  
  // --- initialize Pointer List ---
  PointerList::Initialize();
  
  // --- configure other ---
  pinMode(pinLEDRed, OUTPUT);
  pinMode(pinLEDBlue, OUTPUT);
//  pinMode(pinLEDGreen, OUTPUT); // for SMD version *** MODIFY
  digitalWrite(pinLEDRed, HIGH);
  digitalWrite(pinLEDBlue, HIGH);
//  digitalWrite(pinLEDGreen, LOW); // for SMD version *** MODIFY
  
  // --- set serial ---
  Serial.begin(9600);
  
  // --- set XBee ---
  XBee.Initialize(&Serial); //initialize XBee with Serial for debuging purpose
  ReadXBeeConfiguration();
  
  // --- configure SS pins ---
  pinMode(SS_HARDWARE, OUTPUT); //must be output for the SD library to work
  
  pinMode(SSpin_Ethernet, OUTPUT);
  pinMode(SSpin_SD, OUTPUT);
  digitalWrite(SSpin_Ethernet, HIGH); //disable Ethernet
  digitalWrite(SSpin_SD, HIGH); //disable SD
  
  // --- configure SD ---
  EnableSD();
  if(!mySD.begin(SSpin_SD)){
    Serial.println("ERROR: SD not initialized");
    SDinitialized = false;
//    digitalWrite(pinERROR, HIGH); // for SMD version *** MODIFY
    //blink Blue LED
    BlinkEnable(pinLEDBlue, 100);
    RCdelay(3000); // wait a bit
    BlinkStop();
    digitalWrite(pinLEDBlue, HIGH); //return to setup LEDs
  } else {
    Serial.println("SD initialized");
    SDinitialized = true;
  }

  //create folders if doesn't exist
  if(SDinitialized){
    //IR folder
    if(!mySD.exists(IRfolder.folder)){
      mySD.mkdir(IRfolder.folder);
      Serial.println("IR/ created");
    }
    
    //Access folder
    if(!mySD.exists(RFIDfolder.folder)){
      mySD.mkdir(RFIDfolder.folder);
      Serial.println("Access/ created");
    }
  }
  
  //initialize the access control data
  AccessControlData.state = AS_SACD_STATE_EXECUTE;
  AccessControlData.relay = 0;
  InitializeByteArray(&AccessControlData.serial_number);
  
  // --- configure Ethernet ---
  ReadNetworkConfiguration();
  EnableEthernet();
  Ethernet.begin(myMAC, myIP); 
  server.begin(serverPort);
  
  //set callback function & oscaddress
  server.addCallback("/RoboCore/test", &TEST); //RoboCore callback to verify calls - for testing
  server.addCallback("/RoboCore/configure/^", &Configure); //configure master|slave >> currently not very useful
  server.addCallback("/RoboCore/relay/^", &RelayToggle); //set/unset the relay
  server.addCallback("/RoboCore/pulse/^", &RelayPulse); //send o pulse to the relay
  server.addCallback("/RoboCore/IR/record", &IRRecord); // record the IR command
  server.addCallback("/RoboCore/IR/send/^", &IRSend); //send/save the IR command
  server.addCallback("/RoboCore/rfid/add/^", &RFIDAdd); //add an rfid to the access list
  server.addCallback("/RoboCore/rfid/remove/^", &RFIDRemove); //remove an rfid from the access list
  
  // --- configure IR ---
  SaveIR = false;
  IRlength = 0;

//  digitalWrite(pinON, HIGH); // *** MODIFY
  Serial.println("--- RoboCore Albatross Master v1.0 ---");
  
  // --- reset LEDs ---
  digitalWrite(pinLEDRed, LOW);
  digitalWrite(pinLEDBlue, LOW);
}

// LOOP ********************************************************************************

void loop(){
  //check for OSC message
  if(server.availableCheck(CALL_SPECIFIC_CALLBACK, 1) > 0){ //don't call general callback + flush
    //do nothing, used only to receive OSC messages
  }
  
  //--------------------------------
  
  //check for serial message
  if(Serial.available()){
    //read command as string
    int length = ReadFromSerial(&Serial, buffer, BUFFER_SIZE, EOL_CHAR);
    char temp[length + 1];
    temp[length] = '\0';
    for(byte i=0 ; i < length ; i++)
      temp[i] = buffer[i];
    
    //automatic configuration of the XBee
    if((length == StrLength((char*)AutoConfigure)) && (StrCompare(temp, (char*)AutoConfigure, (byte)CASE_INSENSITIVE) == StrLength((char*)AutoConfigure))){ //must be exact match
      Serial.println();
      Serial.println("# Auto Configuration as Master");
      //try common values
      Serial.println("9600");
      byte res = XBee.ConfigureAsMaster(9600);
      if(res != 1){
        Serial.println("19200");
        res = XBee.ConfigureAsMaster(19200);
      }
      
      if(res != 1){
        Serial.println("1200");
        res = XBee.ConfigureAsMaster(1200);
      }
      if(res != 1){
        Serial.println("2400");
        res = XBee.ConfigureAsMaster(2400);
      }
      if(res != 1){
        Serial.println("4800");
        res = XBee.ConfigureAsMaster(4800);
      }
      if(res != 1){
        Serial.println("38400");
        res = XBee.ConfigureAsMaster(38400);
      }
      if(res != 1){
        Serial.println("57600");
        res = XBee.ConfigureAsMaster(57600);
      }
      if(res != 1){
        Serial.println("115200");
        res = XBee.ConfigureAsMaster(115200);
      }
      
      //display result
      if(res != 1){
        Serial.println("ERROR on configuration");
      } else {
        Serial.println("done!");
      }
    }
    //configure as slave
    else if(StrParserQty(temp, ':') == 2){
      //get the data sent
      char* config = StrParser(temp, ':', 1);
      char* slave = StrParser(temp, ':', 2);
      
      if((StrLength(config) == StrLength((char*)ConfigureSlave)) && (StrCompare(config, (char*)ConfigureSlave, (byte)CASE_INSENSITIVE) == StrLength((char*)ConfigureSlave))){ //must be exact match
        unsigned long slave_type = ModuleHash(slave); //get the code for the slave
        
        //configure if the type exists
        if(SlaveTypeExists(slave_type)){
          Serial.println();
          Serial.println("# Configure as Slave");
          byte res = XBee.ConfigureAsSlave(XBee.GetXBeebaudrate()); //presumed baudrate already set
          RCdelay(1000); //time for the XBee to "recover" from previous configuration
          if(res == 1){
            Serial.println();
            res += ConfigureSlavePins(&XBee, slave_type); //configure the pins
          }
          
          //display result
          if(res != 2){
            Serial.println("ERROR on configuration");
          } else {
            Serial.println("done!");
          }
        }
      } else {
        Serial.println(InvalidCommand);
      }
      
      Mfree(config); //free the string allocated by StrParser()
      Mfree(slave); //free the string allocated by StrParser()
    }
    //restore values
    else if((length == StrLength((char*)Restore)) && (StrCompare(temp, (char*)Restore, (byte)CASE_INSENSITIVE) == StrLength((char*)Restore))){ //must be exact match
      Serial.println();
      Serial.println("# Restore");
      byte res = XBee.Restore();
      
      //display result
      if(res != 1){
        Serial.println("ERROR on configuration");
      } else {
        Serial.println("Factory settings restored!");
      }
    }
    //display version
    else if((length == StrLength((char*)Version)) && (StrCompare(temp, (char*)Version, (byte)CASE_INSENSITIVE) == StrLength((char*)Version))){ //must be exact match
      Serial.println();
      Serial.print("# Version: ");
      Serial.print(AMS_VERSION_MAIN);
      Serial.print(".");
      Serial.println(AMS_VERSION_SUB);
    }
    //display local network configuration
    else if((length == StrLength((char*)IPconfig)) && (StrCompare(temp, (char*)IPconfig, (byte)CASE_INSENSITIVE) == StrLength((char*)IPconfig))){ //must be exact match
      Serial.println();
      //display the MAC
      Serial.print(LNetwork_mac);
      Serial.print('=');
      for(int i=0 ; i < 5 ; i++){
        Serial.print(myMAC[i], HEX);
        Serial.print('-');
      }
      Serial.println(myMAC[5], HEX);
      //display the IP
      Serial.print(LNetwork_ip);
      Serial.print('=');
      for(int i=0 ; i < 3 ; i++){
        Serial.print(myIP[i]);
        Serial.print('.');
      }
      Serial.println(myIP[3]);
      //display the server port
      Serial.print(LNetwork_serverPort);
      Serial.print('=');
      Serial.println(serverPort);
      //display the client port
      Serial.print(LNetwork_clientPort);
      Serial.print('=');
      Serial.println(clientPort);
    }
    //display XBee network configuration
    else if((length == StrLength((char*)XBeeConfig)) && (StrCompare(temp, (char*)XBeeConfig, (byte)CASE_INSENSITIVE) == StrLength((char*)XBeeConfig))){ //must be exact match
      Serial.println();
      //display the ID
      Serial.print(XNetwork_ID);
      Serial.print('=');
      Serial.println(XBee.GetNetworkID(), HEX);
      //display the Channel
      Serial.print(XNetwork_Channel);
      Serial.print('=');
      Serial.println(XBee.GetNetworkChannel(), HEX);
    }
    //configure the network (local or XBee)
    else if(StrParserQty(temp, '=') > 1){ //must have at least 2 parts
      char *cmd = StrParser(temp, '=', 1); //get the command
      int cmd_length = StrLength(cmd);
      
      //check if XBee Network  ID
      if((cmd_length == StrLength((char*)XNetwork_ID)) && (StrCompare(cmd, (char*)XNetwork_ID, (byte)CASE_INSENSITIVE) == StrLength((char*)XNetwork_ID))){
        char *value = StrParser(temp, '=', 2); //get the value
        //store in a byte array
        ByteArray bvalue;
        InitializeByteArray(&bvalue);
        HexStringToByteArray(value, &bvalue);
        //convert the value
        word id = 0;
        for(int j=0 ; j < bvalue.length ; j++){
          id = (id * 256) + bvalue.ptr[j];
        }
        //configure the ID
        if(XBee.SetNetworkID(id)){
          //write to the EEPROM
          int current_address = EEPROM_XBEE_ADDRESS_START + 2; //the current address
          EEPROM.write(current_address, ((XBee.GetNetworkID() & 0xFF00) >> 8)); //MSB
          current_address++;
          EEPROM.write(current_address, (XBee.GetNetworkID() & 0xFF)); //LSB
        }
        //free the variables
        FreeByteArray(&bvalue);
        Mfree(value); //free the string allocated by StrParser()
      }
      //check if XBee Network Channel
      else if((cmd_length == StrLength((char*)XNetwork_Channel)) && (StrCompare(cmd, (char*)XNetwork_Channel, (byte)CASE_INSENSITIVE) == StrLength((char*)XNetwork_Channel))){
        char *value = StrParser(temp, '=', 2); //get the value
        //store in a byte array
        ByteArray bvalue;
        InitializeByteArray(&bvalue);
        HexStringToByteArray(value, &bvalue);
        //configure the Channel
        if(XBee.SetNetworkChannel(bvalue.ptr[0])){
          //write to the EEPROM
          int current_address = EEPROM_XBEE_ADDRESS_START + 1; //the current address
          EEPROM.write(current_address, XBee.GetNetworkChannel());
        }
        //free the variables
        FreeByteArray(&bvalue);
        Mfree(value); //free the string allocated by StrParser()
      }
      //check if Local Network Server Port
      else if(((cmd_length == StrLength((char*)LNetwork_serverPort)) && StrCompare(cmd, (char*)LNetwork_serverPort, (byte)CASE_INSENSITIVE) == StrLength((char*)LNetwork_serverPort))){
        char *value = StrParser(temp, '=', 2); //get the value
        //store in a byte array
        ByteArray bvalue;
        InitializeByteArray(&bvalue);
        StringToByteArray(value, &bvalue);
        //convert the value
        uint16_t port = 0;
        for(int j=0 ; j < bvalue.length ; j++){
          port = (port * 10) + bvalue.ptr[j] - 48; // '0' = 48
        }
        //set the port
        SetNetworkPort(true, port);
        //free the variables
        FreeByteArray(&bvalue);
        Mfree(value); //free the string allocated by StrParser()
      }
      //check if Local Network Client Port
      else if(((cmd_length == StrLength((char*)LNetwork_clientPort)) && StrCompare(cmd, (char*)LNetwork_clientPort, (byte)CASE_INSENSITIVE) == StrLength((char*)LNetwork_clientPort))){
        char *value = StrParser(temp, '=', 2); //get the value
        //store in a byte array
        ByteArray bvalue;
        InitializeByteArray(&bvalue);
        StringToByteArray(value, &bvalue);
        //convert the value
        uint16_t port = 0;
        for(int j=0 ; j < bvalue.length ; j++){
          port = (port * 10) + bvalue.ptr[j] - 48; // '0' = 48
        }
        //set the port
        SetNetworkPort(false, port);
        //free the variables
        FreeByteArray(&bvalue);
        Mfree(value); //free the string allocated by StrParser()
      }
      //check if Local Network IP
      else if(((cmd_length == StrLength((char*)LNetwork_ip)) && StrCompare(cmd, (char*)LNetwork_ip, (byte)CASE_INSENSITIVE) == StrLength((char*)LNetwork_ip))){
        char *ip = StrParser(temp, '=', 2); //get the ip
        if(StrParserQty(ip, '.') == 4){ //must have 4 fields
          //create byte array to store ip
          ByteArray bip;
          InitializeByteArray(&bip);
          ResizeByteArray(&bip, 4);
          //create temporary byte array to store fields
          ByteArray btemp;
          InitializeByteArray(&btemp);
          //get the values
          for(int j=1 ; j <= 4 ; j++){
            //get the field
            char *field = StrParser(ip, '.', j);
            StringToByteArray(field, &btemp);
            //convert the value
            byte value = 0;
            for(int k=0 ; k < btemp.length ; k++){
              value = (value * 10) + btemp.ptr[k] - 48; // '0' = 48
            }
            FreeByteArray(&btemp); //free the memory
            //store the value
            bip.ptr[j-1] = value;
            Mfree(field); //free the string allocated by StrParser()
          }
          //set the IP
          SetNetworkID(&bip);
          FreeByteArray(&bip);
        }
        Mfree(ip); //free the string allocated by StrParser()
      }
      //check if Local Network MAC
      else if(((cmd_length == StrLength((char*)LNetwork_mac)) && StrCompare(cmd, (char*)LNetwork_mac, (byte)CASE_INSENSITIVE) == StrLength((char*)LNetwork_mac))){
        char *mac = StrParser(temp, '=', 2); //get the ip
        if(StrParserQty(mac, '-') == 6){ //must have 6 fields
          //create byte array to store mac
          ByteArray bmac;
          InitializeByteArray(&bmac);
          ResizeByteArray(&bmac, 6);
          //get the values
          for(int j=1 ; j <= 6 ; j++){
            char *field = StrParser(mac, '-', j); //get the field
            bmac.ptr[j-1] = HexCharToByte(field); //convert and store the value
            Mfree(field); //free the string allocated by StrParser()
          }
          //set the MAC
          SetNetworkID(&bmac);
          FreeByteArray(&bmac);
        }
        Mfree(mac); //free the string allocated by StrParser()
      }
      //invalid command
      else{
        Serial.println(InvalidCommand);
      }
      
      Mfree(cmd); //free the string allocated by StrParser()
    }
    //invalid command
    else{
      Serial.println(InvalidCommand);
    }

  } //end of Serial.available()
  
  //--------------------------------
  
  //check for XBee message
  char* str = NULL;
  int res = XBee.Listen(&str, false, 20, 100); //use 20 ms timeout
  if(res == 1){
    Serial.println(); //for debugging
    AvailableMemory(&Serial, true); //for debugging
  
    //parse source 64-bit address + data **** NOT YET IMPLEMENTED ??? which frame identifier will be received?
    char* temp = "##";
    temp[0] = str[0];
    temp[1] = str[1];
    if(HexCharToByte(temp) == API_RX_64_BIT){
      int length = StrLength(str);
      
      //parse address
      char address[17];
      address[16] = '\0';
      for(int i=0 ; i < 16 ; i++){
        address[i] = str[i+2]; // (+2) because of the frame identifer
      }
      
      //parse data
      int data_length = length - 21; // (-2) for frame identifier & (-16) for address & (-2) for RSSI & (-2) for options & (+1) for '\0'
      char data[data_length];
      data[data_length - 1] = '\0';
      for(int i=0 ; i < data_length ; i++){
        data[i] = str[i+22];
      }
      
      //store data in a Byte Array
      ByteArray bdata;
      InitializeByteArray(&bdata);
      HexStringToByteArray(data, &bdata);
      
      if(AccessControl(address, &bdata) == 0){ //call the access control function
        DataProcessing(address, &bdata); //call the processing function
      }
      FreeByteArray(&bdata); //free the used memory
    }
  }
  Mfree(str); //free the string allocated by Listen()
  
}

//**************************************************************************************
//********************************* OSC SPECIFIC ***************************************
//**************************************************************************************

// CONFIGURE ***************************************************************************

// Configure XBee as master/slave
// (returns -1 if invalid input or the result of the configuration)
void Configure(OSCMessage *_mes){
  Serial.println(); //for debugging
  AvailableMemory(&Serial, true); //for debugging
  
  BlinkEnable(pinConfigure, 500); //blink LED
  Serial.println("CONFIGURE");
  int res = -1;
  
  int qty = StrParserQty(_mes->getOSCAddress(), '/');
  if(qty >= 3){                                             // "/RoboCore/configure/<type>"
    char* type = StrParser(_mes->getOSCAddress(), '/', 3);
    long baudrate = XBee.GetXBeebaudrate();
    //check if baudrate is given
    if(qty == 4){                                           // "/RoboCore/configure/<type>/<baudrate>"
      char* temp = StrParser(_mes->getOSCAddress(), '/', 4);
      baudrate = atol(temp);
      //check if valid value
      if((baudrate % 1200) != 0)
        baudrate = XBee.GetXBeebaudrate();
      Mfree(temp); //free the string allocated by StrParser()
    }
    //configure
    if(StrCompare("master", type) == 6){
      Serial.println("\tmaster");
      res = XBee.ConfigureAsMaster(baudrate);
    } else if(StrCompare("slave", type) == 5){
      Serial.println("\tslave");
      res = XBee.ConfigureAsSlave(baudrate);
      // NOT YET IMPLEMENTED, but should configure the slave by its type
    } else {
      Serial.println("\tother");
    }
    Mfree(type); //free the string allocated by StrParser()
    
    //display result
    if(res != 1){
      Serial.println("ERROR on configuration");
    } else {
      Serial.println("done!");
    }
  } else {
    Serial.println("\tINVALID command");
  }
  
  //send response
  OSCMessage resMes;
  resMes.setAddress(_mes->getIpAddress(), clientPort);
  resMes.beginMessage("/RoboCore/configure");
  resMes.addArgInt32(res);
  client.send(&resMes);
  
  BlinkStop(); //stop blink
  digitalWrite(pinConfigure, LOW);
}


// TOGGLE RELAY ************************************************************************

void RelayToggle(OSCMessage *_mes){
//NOTE: could call MReset() at the end of the functions instead of calling Mfree() multiple times

//IMPORTANT: avoid processing (ex: finding the relay) when creating the message to avoid a peak in memory usage

  digitalWrite(pinRelayToggle, HIGH); //turn on
  Serial.println(); //for debugging
  AvailableMemory(&Serial, true); //for debugging
  
  Serial.println("RELAY");
  int res = -1;
  
  int arg_qty = StrParserQty(_mes->getOSCAddress(), '/');
  
  if(arg_qty == 4){ // "/RoboCore/relay/<serial_number>/<relay_number>" + <relay_state>
    if(_mes->getArgsNum() == 1){ //must have an argument
      int state = 0; //default
      
      switch(_mes->getArgTypeTag(0)){
        case kTagInt32:
          state = _mes->getArgInt32(0); //get the value
          break;
        case kTagFloat:
          state = (int)_mes->getArgFloat(0); //get the value and convert
          break;
        case kTagString:{ //use brackets otherwise causes interference with switch statement and memory problem
          int size = _mes->getArgStringSize(0);
          char arg[size];
          _mes->getArgString(0, arg);
          if(atoi(arg)) //if valid string and greater than 0
            state = 1;
          break;
        }
        default: //INVALID argument type
          state = -1;
          break;
      }
      
      if(state != -1){
        //--- select relay to toggle
        char* srelay = StrParser(_mes->getOSCAddress(), '/', 4);
        int relay = atoi(srelay);
        char* totoggle = NULL; //NOT to be freed, because it WASN'T allocated
        switch(relay){
          case 1:
            totoggle = D1;
            break;
          
          case 2:
            totoggle = D2;
            break;
          
          case 3:
            totoggle = D3;
            break;
          
          case 4:
            totoggle = D4;
            break;
          
          case 5:
            totoggle = D5;
            break;
          
          default:
            totoggle = D0;
            break;
        }
        Mfree(srelay); //free string
        //--- end of selection
        //get receiver XBee
        char* receiver = StrParser(_mes->getOSCAddress(), '/', 3); //get the address
        if(StrLength(receiver) == 16){ //8 bytes in HEX format >> 16 characters
          //begin transmission
          ByteArray barray;
          InitializeByteArray(&barray);
          //--- create message
          char* tosend = NULL;
          if(state > 0)
            tosend = ByteToHexChar(XBEE_PIN_DO_HIGH); //ON
          else
            tosend = ByteToHexChar(XBEE_PIN_DO_LOW);  //OFF
          XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, totoggle, tosend);
          Mfree(tosend); //free string
          //--- end of creation
          XBee.CreateFrame(&barray); //already free Byte Array
          XBee.Send();
          char* str = NULL;
          XBee.Listen(&str, false);
          
          res = XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str);
          Mfree(str); //free the string allocated by Listen()
          
          res += SendATWR(&receiver); //send the WR command
        }
        Mfree(receiver); //free the string allocated by StrParser()
      }
    }
  } else if(arg_qty == 5){ // "/RoboCore/relay/<serial_number>/<relay_number>/<relay_state>"
    //--- select relay to toggle
    char* srelay = StrParser(_mes->getOSCAddress(), '/', 4);
    int relay = atoi(srelay);
    char* totoggle = NULL; //NOT to be freed, because it WASN'T allocated
    switch(relay){
      case 1:
        totoggle = D1;
        break;
          
      case 2:
        totoggle = D2;
        break;
      
      case 3:
        totoggle = D3;
        break;
      
      case 4:
        totoggle = D4;
        break;
      
      case 5:
        totoggle = D5;
        break;
        
      default:
        totoggle = D0;
        break;
    }
    Mfree(srelay); //free string
    //--- end of selection
    char* svalue = StrParser(_mes->getOSCAddress(), '/', 5); //get the value
    int value = atoi(svalue);
    Mfree(svalue); //free the string allocated by StrParser()
    //get receiver XBee
    char* receiver = StrParser(_mes->getOSCAddress(), '/', 3); //get the address
    if(StrLength(receiver) == 16){ //8 bytes in HEX format >> 16 characters
      //begin transmission
      ByteArray barray;
      InitializeByteArray(&barray);
      //--- create message
      char* tosend = NULL;
      if(value > 0)
        tosend = ByteToHexChar(XBEE_PIN_DO_HIGH); //ON
      else
        tosend = ByteToHexChar(XBEE_PIN_DO_LOW);  //OFF
      XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, totoggle, tosend);
      Mfree(tosend); //free string
      //--- end of creation
      XBee.CreateFrame(&barray); //already free Byte Array
      XBee.Send();
      char* str = NULL;
      XBee.Listen(&str, false);

      res = XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str);
      Mfree(str); //free the string allocated by Listen()

      res += SendATWR(&receiver); //send the WR command
    }
    Mfree(receiver); //free the string allocated by StrParser()
  }
  
  //send response
  OSCMessage resMes;
  resMes.setAddress(_mes->getIpAddress(), clientPort);
  resMes.beginMessage("/RoboCore/relay");
  resMes.addArgInt32(res);
  client.send(&resMes);
  
  digitalWrite(pinRelayToggle, LOW); //turn off
}

// PULSE RELAY ************************************************************************

void RelayPulse(OSCMessage *_mes){
//NOTE: could call MReset() at the end of the functions instead of calling Mfree() multiple times

//IMPORTANT: avoid processing (ex: finding the relay) when creating the message to avoid a peak in memory usage

  digitalWrite(pinRelayToggle, HIGH); //turn on
  Serial.println(); //for debugging
  AvailableMemory(&Serial, true); //for debugging
  
  Serial.println("PULSE");
  int res = -1;
  
  int arg_qty = StrParserQty(_mes->getOSCAddress(), '/');
  
  if(arg_qty == 4){ // "/RoboCore/pulse/<serial_number>/<relay_number>"
    int pulse_time = DEFAULT_PULSE_TIME; //default

    if(_mes->getArgsNum() == 1){ //can have an argument
      switch(_mes->getArgTypeTag(0)){
        case kTagInt32:
          pulse_time = _mes->getArgInt32(0); //get the value
          break;
        case kTagFloat:
          pulse_time = (int)_mes->getArgFloat(0); //get the value and convert
          break;
        case kTagString:{ //use brackets otherwise causes interference with switch statement and memory problem
          int size = _mes->getArgStringSize(0);
          char arg[size];
          _mes->getArgString(0, arg);
          pulse_time = atoi(arg); //convert value
          break;
        }
        //else has default value
      }
    }
    
    //validate time
    if(pulse_time < MIN_PULSE_TIME)
      pulse_time = MIN_PULSE_TIME;
    if(pulse_time > MAX_PULSE_TIME)
      pulse_time = MAX_PULSE_TIME;
    
    //--- select relay to pulse
    char* srelay = StrParser(_mes->getOSCAddress(), '/', 4);
    int relay = atoi(srelay);
    char* topulse = NULL; //NOT to be freed, because it WASN'T allocated
    switch(relay){
      case 1:
        topulse = D1;
        break;
          
      case 2:
        topulse = D2;
        break;
      
      case 3:
        topulse = D3;
        break;
      
      case 4:
        topulse = D4;
        break;
      
      case 5:
        topulse = D5;
        break;
      
      default:
        topulse = D0;
        break;
    }
    Mfree(srelay); //free string
    //--- end of selection
    //get receiver XBee
    char* receiver = StrParser(_mes->getOSCAddress(), '/', 3); //get the address
    if(StrLength(receiver) == 16){ //8 bytes in HEX format >> 16 characters
      //begin transmission
      ByteArray barray;
      InitializeByteArray(&barray);
      
      //--- create FIRST message
      char* tosend = ByteToHexChar(5); //ON
      XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, topulse, tosend);
      Mfree(tosend); //free string
      //--- end of creation
      XBee.CreateFrame(&barray); //already free Byte Array
      XBee.Send();
      char* str = NULL;
      XBee.Listen(&str, false);
      
      res = XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str);
      Mfree(str); //free the string allocated by Listen()
      
      //wait to end pulse
      long start_time = millis();
      long current_time = start_time;
      while(current_time < (start_time + pulse_time)){
        current_time = millis();
      }
      
      //--- create SECOND message
      tosend = ByteToHexChar(4); //OFF
      XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, topulse, tosend);
      Mfree(tosend); //free string
      //--- end of creation
      XBee.CreateFrame(&barray); //already free Byte Array
      XBee.Send();
      str = NULL;
      XBee.Listen(&str, false);
      
      res += XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str); //add to previous result
      Mfree(str); //free the string allocated by Listen()
    }
    Mfree(receiver); //free the string allocated by StrParser()
  } else if(arg_qty == 5){ // "/RoboCore/pulse/<serial_number>/<relay_number>/<pulse_time>"
    //--- select relay to pulse
    char* srelay = StrParser(_mes->getOSCAddress(), '/', 4);
    int relay = atoi(srelay);
    char* topulse = NULL; //NOT to be freed, because it WASN'T allocated
    switch(relay){
      case 1:
        topulse = D1;
        break;
          
      case 2:
        topulse = D2;
        break;
      
      case 3:
        topulse = D3;
        break;
      
      case 4:
        topulse = D4;
        break;
      
      case 5:
        topulse = D5;
        break;
      
      default:
        topulse = D0;
        break;
    }
    Mfree(srelay); //free string
    //--- end of selection
    char* svalue = StrParser(_mes->getOSCAddress(), '/', 5); //get the pulse time
    int pulse_time = atoi(svalue);
    //validate time
    if(pulse_time < MIN_PULSE_TIME)
      pulse_time = MIN_PULSE_TIME;
    if(pulse_time > MAX_PULSE_TIME)
      pulse_time = MAX_PULSE_TIME;
    Mfree(svalue); //free the string allocated by StrParser()
    //get receiver XBee
    char* receiver = StrParser(_mes->getOSCAddress(), '/', 3); //get the address
    if(StrLength(receiver) == 16){ //8 bytes in HEX format >> 16 characters
      //begin transmission
      ByteArray barray;
      InitializeByteArray(&barray);
      
      //--- create FIRST message
      char* tosend = ByteToHexChar(5); //ON
      XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, topulse, tosend);
      Mfree(tosend); //free string
      //--- end of creation
      XBee.CreateFrame(&barray); //already free Byte Array
      XBee.Send();
      char* str = NULL;
      XBee.Listen(&str, false);
      
      res = XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str);
      Mfree(str); //free the string allocated by Listen()
      
      //wait to end pulse
      long start_time = millis();
      long current_time = start_time;
      while(current_time < (start_time + pulse_time)){
        current_time = millis();
      }
      
      //--- create SECOND message
      tosend = ByteToHexChar(4); //OFF
      XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, topulse, tosend);
      Mfree(tosend); //free string
      //--- end of creation
      XBee.CreateFrame(&barray); //already free Byte Array
      XBee.Send();
      str = NULL;
      XBee.Listen(&str, false);
      
      res += XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str); //add to previous result
      Mfree(str); //free the string allocated by Listen()
    }
    Mfree(receiver); //free the string allocated by StrParser()
  }
  
  //send response
  OSCMessage resMes;
  resMes.setAddress(_mes->getIpAddress(), clientPort);
  resMes.beginMessage("/RoboCore/pulse");
  resMes.addArgInt32(res);
  client.send(&resMes);
  
  digitalWrite(pinRelayToggle, LOW); //turn off
}


// *************************************************************************************
// IR - RECORD *************************************************************************

// Record the IR command
void IRRecord(OSCMessage *_mes){
  Serial.println(); //for debugging
  AvailableMemory(&Serial, true); //for debugging
  
  //check if SD was initialized
  if(!SDinitialized){
    Serial.println("No SD");
    //blink Yellow
    BlinkEnable(pinLEDRed, 100);
    RCdelay(2000); // wait a bit
    BlinkStop();
    return;
  }
  
  Serial.println("IR RECORD");
  boolean res = false; //default
  boolean gotoend = false; //Arduino DOES NOT allow labels after the goto statement
  SaveIR = false; //reset
  
  //record
  decode_results Results;
  IRreceiver.enableIRIn(); //start de receiver
  Serial.println("Recording...");
  
  //start to blink LED
  BlinkEnable(pinIRRecord, 200);
  
  boolean leave = false;
  unsigned long start_time = millis();
  unsigned long current_time;
  while(!leave){
    current_time = millis();
    
    //check for timeout
    if((current_time - start_time) >= RECORDING_TIMEOUT)
      leave = true;
    
    //check for valid input
    if(IRreceiver.decode(&Results)){
      leave = true;
      SaveIR = true; //set
      res = true; //set
    }
  }
  
  //stop blink
  BlinkStop();
  
  IRreceiver.resume(); //must be called after enable
  
  //check whether to save or not
  if(!SaveIR){
    digitalWrite(pinIRRecord, LOW); //make sure it is off
    Serial.println("\tTIMEOUT");
    gotoend = true;
  }
  
  if(!gotoend){ //gotoend 2
    Serial.println("Converting");
    
    //convert date
      // To store raw codes:
      // Drop first value (gap)
      // Convert from ticks to microseconds
      // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    IRlength = Results.rawlen - 1;
    for(int i=1 ; i < Results.rawlen ; i++){
      if (i % 2) {
        // Mark
        IRbuffer[i-1] = Results.rawbuf[i]*USECPERTICK - MARK_EXCESS; //IRbuffers starts at 0 and rawbuf at 1
        Serial.print(" m");
        Serial.print(IRbuffer[i-1], DEC);
      } 
      else {
        // Space
        IRbuffer[i-1] = Results.rawbuf[i]*USECPERTICK + MARK_EXCESS; //IRbuffers starts at 0 and rawbuf at 1
        Serial.print(" s");
        Serial.print(IRbuffer[i-1], DEC);
      }
    }
    Serial.println();
    
    digitalWrite(pinIRRecord, HIGH); //turn LED on
    res = true; //successful
  } //gotoend 2
  
  //send response
  OSCMessage resMes;
  resMes.setAddress(_mes->getIpAddress(), clientPort);
  resMes.beginMessage("/RoboCore/IR/record");
  resMes.addArgInt32(res);
  client.send(&resMes);
}


// IR - SEND ***************************************************************************

// Send/Save the IR command
void IRSend(OSCMessage *_mes){
  Serial.println(); //for debugging
  AvailableMemory(&Serial, true); //for debugging
  
  //check if SD was initialized
  if(!SDinitialized){
    Serial.println("No SD");
    //blink Yellow
    BlinkEnable(pinLEDRed, 100);
    RCdelay(2000); // wait a bit
    BlinkStop();
    return;
  }
  
  if(SaveIR)
    Serial.println("IR SAVE");
  else
    Serial.println("IR SEND");
  boolean res = false; //default
  boolean gotoend = false; //Arduino DOES NOT allow labels after the goto statement
  
  
  
  //check for valid name
  char *fname = NULL;
  if(StrParserQty(_mes->getOSCAddress(), '/') >= 4){
    fname = StrParser(_mes->getOSCAddress(), '/', 4); //get the name
  }
  if(fname != NULL){
    if(!BuildFileName(&IRfolder, fname)){ //_mes
      Serial.println("Invalid file name!");
      gotoend = true;
    }
  }
  Mfree(fname); //free the string allocated by StrParser()
  
  if(!gotoend){ //gotoend 1
    if(SaveIR){ // ---------------------------------------- Save to SD
      EnableSD(); //enable the SD
      
      //remove file if exists
      if(mySD.exists(file_name)){
        mySD.remove(file_name);
        Serial.println("Removed file");
      }
      
      //write to file
      File myfile = mySD.open(file_name, FILE_WRITE);
      writeUINT(&myfile, IRlength);
      myfile.write('|');
      for(int i=0 ; i < IRlength ; i++){
        writeUINT(&myfile, IRbuffer[i]);
      }
      myfile.close();
    
      Serial.print("\tSaved as ");
      Serial.print(file_name);
      
      SaveIR = false; //reset
      digitalWrite(pinIRRecord, LOW); //turn LED off
      res = true; //successful
    } else { // ---------------------------------------- Send from SD
      EnableSD(); //enable the SD
    
      //check if file exists
      if(!mySD.exists(file_name)){
        Serial.println("File DOES NOT exist");
        gotoend = true;
      }
      
      if(!gotoend){ //gotoend 2
        //parse number of times to repeat
        int repeat = 1; //by default send just one time
        if(StrParserQty(_mes->getOSCAddress(), '/') >= 5){
          char *value = StrParser(_mes->getOSCAddress(), '/', 5);
          repeat = atoi(value);
          if((repeat <= 0) || (repeat > 30))
            repeat = 1;
          Mfree(value); //free the string allocated by StrParser()
        }
        
        //read from file
        File myfile = mySD.open(file_name, FILE_READ);
        IRlength = 0; //reset
        if(myfile.available() >= 2){
          IRlength = readUINT(&myfile); //read length
          myfile.read(); // '|' character
        }
        //check if valid length
        if(IRlength <= 0){
          myfile.close();
          Serial.println("INVALID length");
          gotoend = true;
        }
        
        if(!gotoend){ //gotoend 3
          //store in buffer
          unsigned int i = 0;
          while (myfile.available()) {
            IRbuffer[i] = readUINT(&myfile);
            i++;
          }
          myfile.close();
          
          //start to blink LED
          BlinkEnable(pinIRSend, 10);
        
          // Assume 38 KHz
          for(int i=0 ; i < repeat ; i++){
            IRsender.sendRaw(IRbuffer, IRlength, 38);
            delayMicroseconds(20);
          }
          
          //stop blink
          BlinkStop();
          digitalWrite(pinIRSend, LOW);
          
          res = true; //successful
        } //gotoend 3
      } //gotoend 2
    }
  } //gotoend 1
  
//END of function
  EnableEthernet(); //re-enable Ethernet
  
  OSCMessage resMes;
  resMes.setAddress(_mes->getIpAddress(), clientPort);
  resMes.beginMessage("/RoboCore/IR/send");
  resMes.addArgInt32(res);
  client.send(&resMes);
}


// *************************************************************************************
// TEST ********************************************************************************

// Test function to check OSC
void TEST(OSCMessage *_mes){
  Serial.println(); //for debugging
  AvailableMemory(&Serial, true); //for debugging
  
  digitalWrite(pinTest, HIGH); //turn on
  
  //print client IP address
  byte* address = _mes->getIpAddress();
  for(int i=0 ; i < 4 ; i++){
    Serial.print(address[i]);
    if(i < 3)
      Serial.print(".");
  }
  Serial.print(" : ");
  Serial.println(_mes->getOSCAddress());
  
  //print SD files
  if(SDinitialized){
    Serial.println("\nSD");
    File root = mySD.open("/");
    if(root)
      printDirectory(root, 1);
    root.close();
    Serial.println("\tdone!");
  } else {
    //blink Blue LED
    BlinkEnable(pinLEDBlue, 100);
    RCdelay(2000); // wait a bit
    BlinkStop();
  }
  
  //send response
  OSCMessage resMes;
  resMes.setAddress(_mes->getIpAddress(), clientPort);
  resMes.beginMessage( "/RoboCore");
  resMes.addArgInt32(1);
  client.send(&resMes);
  
  digitalWrite(pinTest, LOW); //turn off
}


// *************************************************************************************
// RFID - ADD **************************************************************************

// Test function to check OSC
void RFIDAdd(OSCMessage *_mes){
   // "/RoboCore/rfig/add/<relay_number>"
   // "/RoboCore/rfig/add/<relay_number>/<serial_number>/"
   
   // add the <relay_number> to AccessControlData
   // add the <serial_number> to AccessControlData
   // change the state of AccessControlData to 1
}

// RFID - REMOVE ***********************************************************************

// Test function to check OSC
void RFIDRemove(OSCMessage *_mes){
   // "/RoboCore/rfig/remove/all"
   // "/RoboCore/rfig/remove/<relay_number>"
   // "/RoboCore/rfig/remove/<relay_number>/<serial_number>/"
   
   // add the <relay_number> to AccessControlData
   // add the <serial_number> to AccessControlData
   // change the state of AccessControlData to 10 or 11
}


//**************************************************************************************
//*********************************** Auxiliary ****************************************
//**************************************************************************************

// SD - Write & Read UINT **************************************************************

// Write UINT to file
size_t writeUINT(File* file_ptr, unsigned int value){
  size_t res = 0;
  res += file_ptr->write((value & 0xFF00) >> 8);
  res += file_ptr->write(value & 0x00FF);
  return res;
}

// Read UINT from file
unsigned int readUINT(File* file_ptr){
  unsigned int res = 0;
  res |= (file_ptr->read() << 8);
  res |= file_ptr->read();
  return res;
}

// SD - Print Directory ****************************************************************

// Print directory
//  (CAUTION >> recursive functions, so can use a lot of memory)
void printDirectory(File dir, int numTabs) {
  dir.rewindDirectory(); //reset
  
  while(true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    
    for (uint8_t i=0; i<numTabs; i++) {
      Serial.print('\t');
    }
    
    Serial.print(entry.name());
    
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs+1);
    } 
    else {
      Serial.println();
    }
    
    // Make sure to close everything that we open when we are done with it
    entry.close();
  }
}

// ENABLE ******************************************************************************

// Enable Ethernet
void EnableEthernet(void){
  digitalWrite(SSpin_SD, HIGH);
  digitalWrite(SSpin_Ethernet, LOW);
}

// Enable SD
void EnableSD(void){
  digitalWrite(SSpin_Ethernet, HIGH);
  digitalWrite(SSpin_SD, LOW);
}

// TIMER 1 *****************************************************************************

//Timer1 Overflow
ISR(TIMER1_OVF_vect)
{
  //check for invalid pin
  if(LEDpin == -1)
    TIMER1_DISABLE;
  
  LEDms++; // +1 ms
  if(LEDms > LEDtime){
    digitalWrite(LEDpin, LEDstate);
    //toggle state
    if(LEDstate == LOW)
      LEDstate = HIGH;
    else
      LEDstate = LOW;
    LEDms = 1; //reset
  }
  
  TIMER1_RESET; //reset the timer
}


// BLINK *******************************************************************************

// Start to blink LED
void BlinkEnable(int pin, int time){
  LEDpin = pin;
  LEDtime = time;
  TIMER1_ENABLE;
}
          
// Stop LED blink
void BlinkStop(){
  LEDpin = -1;
  TIMER1_DISABLE;
}


//**************************************************************************************
//**************************************************************************************

// Send the remote AT command WR
//  (returns the result of ResponseStatus())
byte SendATWR(char **receiver){
  //begin transmission
  ByteArray barray;
  InitializeByteArray(&barray);
  //--- create message
  XBeeMessages::CreateRemoteATRequest(&barray, *receiver, "0000", USE_64_BIT_ADDRESS, WR, NULL);
  //--- end of creation
  XBee.CreateFrame(&barray); //already free Byte Array
  XBee.Send();
  RCdelay(100); //insert a pause because WR command takes some time to answer
  char* str = NULL;
  byte res = XBee.Listen(&str, false);
  
  res = XBeeMessages::ResponseStatus(API_REMOTE_AT_COMMAND_REQUEST, str);
  Mfree(str); //free the string allocated by Listen()
  
  return res;
}

//**************************************************************************************
//**************************************************************************************

// XBee - Read Configuration ***********************************************************

// Read the XBee network configuration from the EEPROM
void ReadXBeeConfiguration(void){
  //check if first time reading - store default values
  if(EEPROM.read(EEPROM_XBEE_ADDRESS_START) != EEPROM_XBEE_READ){
    EEPROM.write(EEPROM_XBEE_ADDRESS_START, EEPROM_XBEE_READ); //write to the read address
    int current_address = EEPROM_XBEE_ADDRESS_START + 1; //the current address
    
    //write the Channel
    EEPROM.write(current_address, XBee.GetNetworkChannel());
    current_address++;
    //write the ID
    EEPROM.write(current_address, ((XBee.GetNetworkID() & 0xFF00) >> 8)); //MSB
    current_address++;
    EEPROM.write(current_address, (XBee.GetNetworkID() & 0xFF)); //LSB
    current_address++;
  }
  // the configuration was already set
  else {
    int current_address = EEPROM_XBEE_ADDRESS_START + 1; //the current address
    
    //read & set the Channel
    byte channel = EEPROM.read(current_address);
    current_address++;
    XBee.SetNetworkChannel(channel);
    //read & set the ID
    word id = (EEPROM.read(current_address) << 8); // MSB
    current_address++;
    id |= EEPROM.read(current_address); // LSB
    current_address++;
    XBee.SetNetworkID(id);
  }
}

// Network - Read Configuration ********************************************************

// Read the local network configuration from the EEPROM
void ReadNetworkConfiguration(void){
  //check if first time reading - store default values
  if(EEPROM.read(EEPROM_NETWORK_ADDRESS_START) != EEPROM_NETWORK_READ){
    EEPROM.write(EEPROM_NETWORK_ADDRESS_START, EEPROM_NETWORK_READ); //write to the read address
    int current_address = EEPROM_NETWORK_ADDRESS_START + 1; //the current address
    
    //write the MAC
    for(int i=0 ; i < 6 ; i++){
      EEPROM.write(current_address, myMAC[i]);
      current_address++;
    }
    //write the IP
    for(int i=0 ; i < 4 ; i++){
      EEPROM.write(current_address, myIP[i]);
      current_address++;
    }
    //write the Server Port
    EEPROM.write(current_address, ((serverPort & 0xFF00) >> 8)); // MSB
    current_address++;
    EEPROM.write(current_address, (serverPort & 0xFF)); // LSB
    current_address++;
    //write the Client Port
    EEPROM.write(current_address, ((clientPort & 0xFF00) >> 8)); // MSB
    current_address++;
    EEPROM.write(current_address, (clientPort & 0xFF)); // LSB
    current_address++;
  }
  // the configuration was already set
  else {
    int current_address = EEPROM_NETWORK_ADDRESS_START + 1; //the current address
    
    //read the MAC
    for(int i=0 ; i < 6 ; i++){
      myMAC[i] = EEPROM.read(current_address);
      current_address++;
    }
    //read the IP
    for(int i=0 ; i < 4 ; i++){
      myIP[i] = EEPROM.read(current_address);
      current_address++;
    }
    //read the Server Port
    serverPort = (EEPROM.read(current_address) << 8); // MSB
    current_address++;
    serverPort |= EEPROM.read(current_address); // LSB
    current_address++;
    //read the Client Port
    clientPort = (EEPROM.read(current_address) << 8); // MSB
    current_address++;
    clientPort |= EEPROM.read(current_address); // LSB
    current_address++;
  }
}

// Network - Set Port ******************************************************************

// Set the server or client port
//  (returns FALSE if port out of range)
boolean SetNetworkPort(boolean server, uint16_t port){
  boolean res = false;
  
  //check value
  if((port <= 9999) && (port > 0)){
    //write to the EEPROM
    if(server){
      EEPROM.write((EEPROM_NETWORK_ADDRESS_START + 1 + 10), ((port & 0xFF00) >> 8)); // MSB
      EEPROM.write((EEPROM_NETWORK_ADDRESS_START + 1 + 11), (port & 0xFF)); // LSB
    } else {
      EEPROM.write((EEPROM_NETWORK_ADDRESS_START + 1 + 12), ((port & 0xFF00) >> 8)); // MSB
      EEPROM.write((EEPROM_NETWORK_ADDRESS_START + 1 + 13), (port & 0xFF)); // LSB
    }
    res = true;
  }
}

// Network - Set MAC or IP *************************************************************

// Set the local network ID (MAC or IP)
//  (returns TRUE if successful)
boolean SetNetworkID(ByteArray *barray){
  boolean res = false;
  int current_address = -1;
  //check if MAC
  if(barray->length == 6){
    current_address = EEPROM_NETWORK_ADDRESS_START + 1; //the current address
  }
  //check if IP
  else if(barray->length == 4){
    current_address = EEPROM_NETWORK_ADDRESS_START + 1 + 6; //the current address
  }
  
  //write the values
  if(current_address != -1){
    for(int i=0 ; i < barray->length ; i++){
      EEPROM.write(current_address, barray->ptr[i]);
      current_address++;
    }
    res = true; //update
  }
  
  return res;
}

//**************************************************************************************









