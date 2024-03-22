// Defining a device "type" name and version allows us to check from
// the Julia side if we connected to correct device (and have a compatible version)
#define ARDUINO_TYPE "EXAMPLE"
#define VERSION "1" // Version numbers should be increased with changes to the communication interface

// To allow us to include lib-files from another directoy we have to give a full path
// The following macros allow us to define the path once and reuse it
#define LIB_HOME /home/.../MPIMeasurementDevices/Arduino/lib/ // Full path to lib folder without quotes: /home/user/../MPIMeasurementDevices/Arduino/lib/
#define IDENT(x) x
#define STR(a) STR_(a)
#define STR_(a) #a
#define INCLUDE_LIB(lib) STR(IDENT(LIB_HOME)IDENT(lib))

// This include is essentially "$LIB_HOME/communication.h". Note that neither LIB_HOME nor the file name use quotation marks of any kind
#include INCLUDE_LIB(communication.h) 


// Setup
#define INPUT_BUFFER_SIZE 256
int getVersion(char*);
commandCallback_t cmdHandler[] = {
  {"VERSION", getVersion},
  //{"FOO", setFoo} //for new function: Add command declaration and then command definition (function body)
};
SerialHandler serialHandler = SerialHandler(INPUT_BUFFER_SIZE, cmdHandler, sizeof(cmdHandler)/sizeof(*cmdHandler));

// Functions
int getVersion(char *cmd) {
  Serial.print(ARDUINO_TYPE);
  Serial.print(":");
  Serial.print(VERSION);
  Serial.flush(); 
}


// Arduino Loops
void setup()
{
  Serial.begin(9600);
}

void loop()
{
  serialHandler.read();
}
