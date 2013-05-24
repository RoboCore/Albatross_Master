
//--------------------------------------------------------- MODIFY

/*
	RoboCore XBee Master Shield
		(v1.0 - 24/05/2013)

  Program to use with the XBee Master Shield from RoboCore ( http://www.RoboCore.net )
    (for Arduino 1.0.1 and later)

  Released under the Beerware licence
  Written by François
  
  
  This program is supported by (master version + slaves versions):
    - XBee Master Shield v1.0
          + XBee Slave v1.0
  
  The program is used to execute commands via OSC messages sent in the local network.
  The included commands are:
    - toggle on/off an IO port in a XBee Slave;
    - pulse an IO port in a XBee Slave;
    - configure a XBee as Master or Slave;
    - record an IR command;
    - send an IR command;
    - perform a test;
  The Serial is used to display the status of the program but IS NOT necessary for the
    program to run. One can also use the Serial to execute special commands:
    - auto_config >> automatically configure the XBee as Master by scanning through
                      the possible baudrate values;
    - config_slave >> configure the XBee as slave (presumed the correct baudrate was
                      already set);
    - restore >> restore the XBee's parameter to their factory settings;
    - version >> return the version of the current program. ************************************************* TEST
  
  
  MUST CONFIGURE variables (before uploading to the Arduino), according to the user's network:
    - myMac[];
    - myIp[];
    - serverPort;
    - clientPort.
  
*/

/*

                                                              ADICIONAR ReadFromSerial() de <String_Functions.h> ************************************************* OK
                                                              
                                                              ************* verificar compatibilidade de versão de mestres e escravos
                                                                        no momento RELAY, PULSE e RECORD são universais, pois estão diretamente associados com o pino do XBee escravo (deve ser configurado pelo usuário) e só há um modo de gravação de IR
                                                                        CONFIGURE atualmente só funciona para a Sërie 1 do XBee (no futuro talvez seja alterado para suporte às duas séries)
                                                                        SEND atualmente está no Master, portanto não há problema de versão
                                                               **** criar .h com pinos + infos importantes das versõe do Master (e Slave?)
                                                              
                                                              create RELEASE version without MODIFY + TESTE + outros
                                                              mudar MODIFY + TESTE + outros ***********************************

*/

#include "VersionPins.h"

#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>

#include <ArdOSC.h> // RoboCore's version v1.0
#include <IRremote.h> //with some small modifications

#include <Memory.h> //v1.3
#include <String_Functions.h> //v1.3
#include <Hex_Strings.h> //v1.2
#include <XBee_API.h> //v1.2


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

#define SS_HARDWARE XMS_SS_HARDWARE //10 on UNO

// ----------------------
// ETHERNET
const int SSpin_Ethernet = XMS_SS_ETHERNET; //of Ethernet Shield

byte myMac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x9B, 0x36 };
byte myIp[]  = { 192, 168, 0, 31 };
uint16_t serverPort  = 4444;
uint16_t clientPort = 1111;

// ----------------------
// SD
const int SSpin_SD = XMS_SS_SD; //of Ethernet Shield
SDClass mySD; //cannot directly use 'SD' because of XBee's AT command **** MODIFY

#define STR_SIZE_FOLDER 3 //size of the string stored in 'kfolder'
#define STR_SIZE_EXTENSION 4 //size of the string stored in 'kextension'
const char kfolder[] = "IR/"; //NOT to forget to add the '/' at the end
const char kextension[] = ".rbc"; //NOT to forget to add the '.' at the beginning

#define FILE_NAME_SIZE 30
char file_name[FILE_NAME_SIZE];

boolean SDinitialized; //false when couldn't initialize the SD


// ----------------------
// OSC
OSCServer server;
OSCClient client;

// ----------------------
// IR
// Sender Pin if by default pin 46 on Mega 2560 (Timer 5)
const int pinIRreceiver = XMS_IR_RECEIVER;

IRrecv IRreceiver(pinIRreceiver);
IRsend IRsender;

#define RECORDING_TIMEOUT 5000 //timeout for the recording
boolean SaveIR; //TRUE when waiting for the command name
unsigned int IRlength; //length of the IR command
unsigned int IRbuffer[RAWBUF]; //values of the IR command (already converted)

// ----------------------
// Other
XBeeMaster XBee(XMS_XBEE_SERIAL);
const int pinLEDRed = XMS_LED_RED;
const int pinLEDBlue = XMS_LED_BLUE;
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

#define EOL_CHAR '#'
#define BUFFER_SIZE 30
char buffer[BUFFER_SIZE]; //buffer to read from Serial
const char AutoConfigure[] = "auto_config";
const char ConfigureSlave[] = "config_slave";
const char Restore[] = "restore";
const char Version[] = "version"; //************************************************************************************ TESTE


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
  
  // --- set folder ---
  for(int i=0 ; i < STR_SIZE_FOLDER ; i++)
    file_name[i] = kfolder[i];
  
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
    delay(3000); // wait a bit
    BlinkStop();
    digitalWrite(pinLEDBlue, HIGH); //return to setup LEDs
  } else {
    Serial.println("SD initialized");
    SDinitialized = true;
  }

  //create folder if doesn't exist
  if(SDinitialized){
    if(!mySD.exists((char*)kfolder)){
      mySD.mkdir((char*)kfolder);
      Serial.println("IR/ created");
    }
  }
  
  // --- configure Ethernet ---
  EnableEthernet();
  Ethernet.begin(myMac, myIp); 
  server.begin(serverPort);
  
  //set callback function & oscaddress
  server.addCallback("/RoboCore/test", &TEST); //RoboCore callback to verify calls - for testing
  server.addCallback("/RoboCore/configure/^", &Configure); //configure master|slave
  server.addCallback("/RoboCore/relay/^", &RelayToggle); //set/unset the relay
  server.addCallback("/RoboCore/pulse/^", &RelayPulse); //send o pulse to the relay
  server.addCallback("/RoboCore/IR/record", &IRRecord); // record the IR command
  server.addCallback("/RoboCore/IR/send/^", &IRSend); //send/save the IR command
  
  // --- configure IR ---
  SaveIR = false;
  IRlength = 0;

//  digitalWrite(pinON, HIGH); // *** MODIFY
  Serial.println("--- RoboCore XBee Master v1.0 ---"); //MODIFY
  
  // --- reset LEDs ---
  digitalWrite(pinLEDRed, LOW);
  digitalWrite(pinLEDBlue, LOW);
}

// LOOP ********************************************************************************

void loop(){
  //check for OSC message
  if(server.availableCheck(CALL_SPECIFIC_CALLBACK, 1) > 0){ //don't call general callback + flush
    //do nothing, used only to receive messages
  }
  
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
    else if((length == StrLength((char*)ConfigureSlave)) && (StrCompare(temp, (char*)ConfigureSlave, (byte)CASE_INSENSITIVE) == StrLength((char*)ConfigureSlave))){ //must be exact match
      Serial.println();
      Serial.println("# Configure as Slave");
      byte res = XBee.ConfigureAsSlave(XBee.GetXBeebaudrate()); //presumed baudrate already set
      
      //display result
      if(res != 1){
        Serial.println("ERROR on configuration");
      } else {
        Serial.println("done!");
      }
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
      Serial.print(XMS_VERSION_MAIN);
      Serial.print(".");
      Serial.println(XMS_VERSION_SUB);
    }
    //invalid command
    else{
      Serial.println("Invalid command");
    }

  } //end of Serial.available()
  
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


// TOGGLE RELAY ************************************************************************ "0013A20040791ABB"

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
            tosend = ByteToHexChar(5); //ON
          else
            tosend = ByteToHexChar(4); //OFF
          XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, totoggle, tosend);
          Mfree(tosend); //free string
          //--- end of creation
          XBee.CreateFrame(&barray); //already free Byte Array
          XBee.Send();
          char* str = NULL;
          XBee.Listen(&str, false);
          
          res = XBeeMessages::ResponseOK(API_REMOTE_AT_COMMAND_REQUEST, str);
          Mfree(str); //free the string allocated by Listen()
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
        tosend = ByteToHexChar(5); //ON
      else
        tosend = ByteToHexChar(4); //OFF
      XBeeMessages::CreateRemoteATRequest(&barray, receiver, "0000", USE_64_BIT_ADDRESS, totoggle, tosend);
      Mfree(tosend); //free string
      //--- end of creation
      XBee.CreateFrame(&barray); //already free Byte Array
      XBee.Send();
      char* str = NULL;
      XBee.Listen(&str, false);
      
      res = XBeeMessages::ResponseOK(API_REMOTE_AT_COMMAND_REQUEST, str);
      Mfree(str); //free the string allocated by Listen()
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

// PULSE RELAY ************************************************************************ "0013A20040791ABB"

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
      
      res = XBeeMessages::ResponseOK(API_REMOTE_AT_COMMAND_REQUEST, str);
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
      
      res += XBeeMessages::ResponseOK(API_REMOTE_AT_COMMAND_REQUEST, str); //add to previous result
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
      
      res = XBeeMessages::ResponseOK(API_REMOTE_AT_COMMAND_REQUEST, str);
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
      
      res += XBeeMessages::ResponseOK(API_REMOTE_AT_COMMAND_REQUEST, str); //add to previous result
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
    delay(2000); // wait a bit
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
    delay(2000); // wait a bit
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
  if(!BuildFileName(_mes)){
    Serial.println("Invalid file name!");
    gotoend = true;
  }
  
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
          IRsender.sendRaw(IRbuffer, IRlength, 38);
          
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
    delay(2000); // wait a bit
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

// BUILD FILE NAME ********************************************************************* MODIFY

// Build file name for SD
boolean BuildFileName(OSCMessage *_mes){
  boolean res = false;
  
  if(StrParserQty(_mes->getOSCAddress(), '/') >= 4){
    char* fname = StrParser(_mes->getOSCAddress(), '/', 4); //get the name
    int length = StrLength(fname);
    
    if((length > 0) && (length < (FILE_NAME_SIZE - STR_SIZE_FOLDER - STR_SIZE_EXTENSION - 1))){
      //insert the name into the buffer and add the extension
      file_name[STR_SIZE_FOLDER + length + STR_SIZE_EXTENSION] = '\0'; //NULL terminated string
      for(int i=0 ; i < length ; i++)
        file_name[i + STR_SIZE_FOLDER] = fname[i];
      for(int i=0 ; i < STR_SIZE_EXTENSION ; i++)
        file_name[i + STR_SIZE_FOLDER + length] = kextension[i];
      
      res = true;
    }
    Mfree(fname); //free the string allocated by StrParser()
  }
  
  return res;
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

//// Read incoming data from serial until End Of Line character
//int ReadFromSerial(char eol){
//  int count = 0;
//  char c = eol - 1;
//  while(c != eol){
//    if(Serial.available()){
//      c = Serial.read();
//      buffer[count] = c;
//      count++;
//    }
//  }
//
//  return (count - 1); //because eol is included
//}

//int length = ReadFromSerial(EOL_CHAR);
//char file_name[length + 1];
//file_name[length] = '\0';
//for(byte i=0 ; i < length ; i++)
//file_name[i] = buffer[i];

//**************************************************************************************









