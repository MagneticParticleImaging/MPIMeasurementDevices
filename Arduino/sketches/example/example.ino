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
// Declare function for callbacks
int getVersion(char*);
void getChain(char*);
void setChain(char*);
void getEntry(char*);
void getHelp(char*);
// Prepare callback array
commandCallback_t cmdHandler[] = {
  {"VERSION", getVersion},
  {"GET:LED:CHAIN", getChain},
  {"GET:LED:ENTRY:", getEntry},
  {"SET:LED:CHAIN:", setChain},
  {"HELP", getHelp},
  //{"FOO", setFoo} //for new function: Add command declaration and then command definition (function body)
};

// Setup up a serial command handler. This struct will read serial input until it finds a valid command ID, it then calls the callback with the full cmd text.
SerialHandler serialHandler = SerialHandler(INPUT_BUFFER_SIZE, cmdHandler, sizeof(cmdHandler)/sizeof(*cmdHandler));

/*
  In this example, a user can enter a list of milliseconds. The built-in LED lights up and off alternately for the duration of each input.
  It begins in the off state.
*/
#define MAX_LED_CHAIN 256
int waits[MAX_LED_CHAIN];
int temp[MAX_LED_CHAIN];
unsigned long previous;
int currentIndex = 0;
int numWaits = 0;
bool currentValue = false;

// Functions
void getHelp(char*) {
  Serial.println("Valid Commands (without quotes):");
  Serial.println("'!GET:LED:CHAIN*' Get all millisecond entries");
  Serial.println("'!GET:LED:ENTRY:X*' Get the x'th entry");
  Serial.println("----------------------------------------------------");
  Serial.println("'!SET:LED:CHAIN:1000,2000,30,...*' Set the millisecond entry with a comma-sepearted list");
  Serial.println("----------------------------------------------------");
  Serial.println("'!VERSION*' Return the uploaded code version");
  Serial.println("'!HELP*' List all commands");
  Serial.println("----------------------------------------------------");
  Serial.println("#");
}


int getVersion(char *cmd) {
  Serial.print(ARDUINO_TYPE);
  Serial.print(":");
  Serial.print(VERSION);
  Serial.flush(); 
}

void getChain(char* cmd) {
  Serial.print("Chain: ");
  for (int i = 0; i < numWaits; i++) {
    Serial.print(waits[i]);
    if (i <  numWaits -1) {
      Serial.print(',');
    }
  }
  Serial.println();
}

void setChain(char * cmd) {
  digitalWrite(LED_BUILTIN, LOW);

  int num = serialHandler.countChars(cmd, ',') + 1; // See how many periods there will be

  if (num >= MAX_LED_CHAIN) {
    Serial.println("Chain is too long");
    return;
  }

  char *base = strrchr(cmd, ':') + 1; // Move to first number

  int result = serialHandler.readNumbers(base, temp, num);
  if (result == 0) {
    updateChain(temp, num);
    Serial.println("Success");
  }
  else {
    Serial.println("Error");
  }
}

void getEntry(char* cmd) {
  char *base = strrchr(cmd, ':') + 1; // Move to first number
  int result = serialHandler.readNumbers(base, temp, 1);
  if (result == 0 && temp[0] < numWaits && temp[0] >= 0) {
    Serial.println(waits[temp[0]]);
  }
  else {
    Serial.println("Error");
  }
}

void iterate() {
  unsigned long current = millis();
  if (current - previous >= waits[currentIndex]) {
    currentValue = !currentValue;
    currentIndex++;
    digitalWrite(LED_BUILTIN, currentValue);
    previous = current;
  }

  if (currentIndex >= numWaits) {
    currentIndex = 0;
  }
}

void updateChain(int *array, int num){
  for (int i = 0; i < num; i++) {
    waits[i] = array[i];
  }
  numWaits = num;
  currentIndex = 0;
  currentValue = false;
}


// Arduino Loops
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, currentValue);

  temp[0] = 1000;
  temp[1] = 500;
  updateChain(temp, 2);
  previous = 0;

  Serial.begin(9600);
}

void loop()
{
  serialHandler.read();
  iterate();
}
