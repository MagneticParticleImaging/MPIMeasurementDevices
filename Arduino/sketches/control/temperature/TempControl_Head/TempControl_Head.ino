#include "TemperatureControl.h"
#include "Arduino.h"
#include <PID_v1.h>
#include <math.h>



//Test Unit1(SensorData[0][0],SensorData[1][0],SensorData[2][0],SensorData[3][0]);
//HeatUnit unit(A0,3,45,80);

#define ARDUINO_TYPE "HEATINGUNIT"
#define VERSION "1"
#define NUM_UNITS_START 1
#define NUM_UNITS 8
#define NUM_UNITS_MAX 8
#define BAUD_RATE 9600
#define pinOpto 10 // müssen an den Konstruktor übergeben werden
#define OptoDelay 50 //[us] Delay of 4N35 = 10us typically + spiel
#define pinErrorLED 11
#define pinErrorSound 40 // extern, top right
#define pinRelayHighVoltage 41 // extern, top left

// User choices
bool EMR_ENABLE = false; // Electromagnetic Relay
bool HEAT_DURING_MEAS = false;
bool displaySerialComm = false;
bool ACK_COMMANDS = false;

// Code variables
bool ENABLE_HEATING = true;    
bool OPTIMIZE_DUTY_CYCLE = false;
bool ERROR = false; //toDo: obsolete
int ErrorCode[NUM_UNITS];
bool OVERTEMP = false;


int configPinIn[] = {A1, A5, A2, A6, A3, A7, A4, A8 };   //Analog Pins ACHTUNG!!!! Thermistor Index und AnalogPin Index stimmen nicht überein!!!
int configSSRPinOut[] = {2, 3, 4, 5, 6, 7, 8, 9 };         //Primärer Digitaler Pin für Steuereung 1
int configEMRPinOut[] = {22, 23, 24, 25, 26, 27, 28, 29 };  //Sekundärer Pin für Relais Steuerung
//float configTSet[] = {57.0, 57.0, 49.0, 49.0, 49.0, 49.0, 45.0, 45.0 };   //Target Temperatures: +1deg from values from jan22
float configTSet[] = {48.0, 48.0, 53.0, 53.0, 48.0, 48.0, 51.0, 51.0 };   //Target Temperatures: with towel from okt22
float configTMax[] = {63.0, 63.0, 68.0, 68.0, 65.0, 65.0, 63.0, 63.0 };   //T Max 

int timeUpdateCycle = 500;  //millis
int cycleSecond = 1000 / timeUpdateCycle;
int cycleFiveSeconds = cycleSecond * 5;

int PidSampleTime = 100; //ms

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long counter = 0;

double Setpoint[NUM_UNITS], TempInput[NUM_UNITS], newDutyCycle[NUM_UNITS];

HeatUnit *units[NUM_UNITS];
PID *pidUnits[NUM_UNITS];




//####################################################################
// Communication
#define INPUT_BUFFER_SIZE 256
char input_buffer[INPUT_BUFFER_SIZE];
unsigned int input_pos = 0;

typedef struct {
  const char *id;
  int (*handler)(char *);
} commandHandler_t;

// Command Declarations
int getAllTemps(char *);
int getAvgTemps(char *);
int getTempSet(char *);
int setTempSet(char *);
int getTempMax(char *);
int setTempMax(char *);
int getControlMode(char *);
int setControlThreshold(char *);
int setControlPWM(char *);
int setDutyCycleMode(char *);
int setToDefault(char *);
int getVersion(char *);
int getCommands(char *);
int setEnableHeating(char *);
int resetOvertemp(char*);
//int setFoo(char*);

commandHandler_t cmdHandler[] = {
  { "GET:ALLTEMPS", getAllTemps },
  { "GET:AVGTEMPS", getAvgTemps },
  { "GET:TSET", getTempSet },
  { "SET:TSET:", setTempSet },
  { "GET:TMAX", getTempMax },
  { "SET:TMAX:", setTempMax },
  { "GET:CONTROL", getControlMode },
  { "SET:C:THRES", setControlThreshold },
  { "SET:C:PWM", setControlPWM },
  { "SET:C:DUTYMODE", setDutyCycleMode },
  { "SET:DEFAULT", setToDefault },
  { "VERSION", getVersion },
  { "GET:COMMANDS", getCommands },
  { "SET:ENABLE_HEATING:", setEnableHeating },
  { "RESET:OVERTEMP", resetOvertemp}
  //{"FOO", setFoo} //for new function: Add command declaration and then command definition (function body)
};

int getCommands(char *) {
  // toDo: Bechreibung
  Serial.println("Valid Commands (without quotes):");
  Serial.println("'!GET:ALLTEMPS*' Get temperatures of all sensors");
  Serial.println("'!GET:TEMPS:<2,5,6>*' Get individual temperatures e.g. from sensors 2, 5 and 6");
  Serial.println("----------------------------------------------------");
  Serial.println("'!GET:MAXT*' Read the alarm-trigger-temperature for all sensors");
  Serial.println("'!SET:MAXT:<65,80, ...,70>*' Set the max-temp (int!) for all sensors");
  Serial.println("----------------------------------------------------");
  Serial.println("'!GET:CONTROL*' Read the alarm-trigger-temperature for all sensors");
  Serial.println("'!SET:C:PWM:*' Switch Control Mode between PID and Threshold Mode");
  Serial.println("from1:Threshold, 2: Hysteresis, 3: DutyCycle, 4: PI Control) for all sensors");
  Serial.println("'SET:ENABLE_HEATING:<1,0>");
  Serial.println("----------------------------------------------------");
  Serial.println("'!VERSION*' Return the uploaded code version");
  Serial.println("'!GET:COMMANDS*' List all commands");
  Serial.println("#");
  
}





//####################################################################
//####################################################################
//####################################################################

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(1000);  //1000ms standard
 
  pinMode(pinErrorLED, OUTPUT);
  digitalWrite(pinErrorLED, LOW); //init high: active low PNP (during setup)
  
  pinMode(pinErrorSound, OUTPUT);
  digitalWrite(pinErrorSound, LOW); //init low: active high, NPN
  
  pinMode(pinRelayHighVoltage, OUTPUT);
  digitalWrite(pinRelayHighVoltage, LOW); //init low: active high, NPN
  
  pinMode(pinOpto, OUTPUT);
  digitalWrite(pinOpto, HIGH); //init low: active low PNP, no measurement

  for (int i = 0; i < NUM_UNITS_MAX; i++ ){//Alle Pins auf HIGH Initialisieren  und Relais schließen
    //initialise leftover output Pins
    pinMode(configSSRPinOut[i], OUTPUT);
    digitalWrite(configSSRPinOut[i], HIGH);
    pinMode(configEMRPinOut[i], OUTPUT);
    digitalWrite(configEMRPinOut[i], HIGH);
    pinMode(configPinIn[i], INPUT);
  }

  float Kp = 1.5;   //std: 1.2 // 1.3 .. 1.5 for SM  // 0.9...1.1 for nice heating curve
  float Ki = 0.4;  // 3.0 too small? may cause this behaviour..
  float Kd = 0.1;  // 1.0 , rather smaller! 0.5? guideline 1/10 of Ki

  for (int i =  0; i < NUM_UNITS; i++) { //Alle Pins auf HIGH Initialisieren  und Relais schließen
    //initialise output Pins
    
    units[i] = new HeatUnit(configPinIn[i], configSSRPinOut[i], configEMRPinOut[i], configTSet[i], configTMax[i], EMR_ENABLE);
    units[i]->temperatureMeasurement();
    TempInput[i] = units[i]->getTemp();
    units[i]->disableOutput(); 

    newDutyCycle[i] = 0.0;
    Setpoint[i] = double(configTSet[i]);
    pidUnits[i] = new PID(&TempInput[i], &newDutyCycle[i], &Setpoint[i], Kp, Ki, Kd, DIRECT);
    
    pidUnits[i]->SetOutputLimits(0, units[i]->getPeriod());
    pidUnits[i]->SetSampleTime(PidSampleTime);
    pidUnits[i]->SetMode(AUTOMATIC);  
    
  }

  digitalWrite(pinRelayHighVoltage, HIGH); //enable, because this will supply 5V as well to all relays...
  
  
  //for smooth startup and moving average, get some real temperatures
  for (int i=0; i<5; i++) {
    delay(30); 
    enableTemperatureMeasurement();
    for (int i = 0; i < NUM_UNITS; i++) {
      units[i]->temperatureMeasurement(); 
    }
    disableTemperatureMeasurement();
  }

  digitalWrite(pinErrorLED, HIGH); //disable after setup
  OVERTEMP = false;
  //Serial.println("Initialised Temperature Control");
}





//####################################################################
// Functions Block Communication
//####################################################################
int getAllTemps(char *) {
  // Befehl : !GET:ALLTEMPS:*#
  float temp = 0.0;

  for (int i = 0; i < NUM_UNITS; i++) {
    temp = units[i]->getTemp();
    Serial.print(temp);
    if (i < NUM_UNITS-1) {
      Serial.print(",");
    }
  }
  Serial.print("#");
}

int getAvgTemps(char *) {
  // Befehl : !GET:AVGTEMPS:*#
  float temp = 0.0;

  for (int i = 0; i < NUM_UNITS; i++) {
    temp = units[i]->getAvgTemp();
    Serial.print(temp);
    if (i < NUM_UNITS-1) {
      Serial.print(",");
    }
  }
  Serial.print("#");
}

int getTempSet(char *) {
  // Befehl : !GET:TSET*#
  float target;
  for (int i = 0; i < NUM_UNITS; i++) {
    target = units[i]->getTargetTemp();  // f(float*)
    Serial.print(target);
    if (i < NUM_UNITS-1) {
      Serial.print(",");
    }  
  }
  Serial.print("#");
}

int getTempMax(char *) {
  // Befehl : !GET:TMAX*#
  float max = 0.0;
  for (int i = 0; i < NUM_UNITS; i++) {
    max = units[i]->getMaxTemp();  // f(float*)
    Serial.println(max);
    if (i < NUM_UNITS-1) {
      Serial.print(",");
    }
  }
  Serial.print("#");
}


int getControlMode(char *) {
  // Befehl : !GET:CONTROL*#
  int mode = 0;
  for (int i = 0; i < NUM_UNITS; i++) {
    mode = units[i]->getControlMode();
    Serial.print("Unit[");
    Serial.print(i);
    Serial.print("] Control Mode: ");
    if (mode == 1) {
      Serial.print("Threshold");
    } 
    else if (mode == 2) {
      Serial.print("PID controled PWM");
    } 
    else if (mode == 3) {  // Default
      Serial.print("Optimized Dutycycle");
    }
    else {
      Serial.print("Unexpected mode");
    }
    Serial.print("#");
  }
}






//####################################################################
int setTempSet(char *cmd) {
  // Befehl : !SET:TSET:<1,2,3,4>*#
  int num = countChars(cmd, ',') + 1;
  if (num == NUM_UNITS) {
    char *base = strchr(cmd, '<') + 1;  // Move to first number
    char *numEnd;
    bool set = true;
    for (int i = 0; i < NUM_UNITS; i++) {
      float target = strtod(base, &numEnd);  //TODO could add more error checking here with errno and end, start
      set &= units[i]->setTargetTemp(target);
      Setpoint[i] = target;
      base = strchr(numEnd, ',') + 1;       // TODO could be more robust by checking for null
    }
    Serial.print(set);
    Serial.print("#");
  } else {
    Serial.print("0#");
  }
  return 0; 
}


//########################################################################
int setTempMax(char *cmd) {
  // !SET:TMAX:<2,3,4,5>*#
  int num = countChars(cmd, ',') + 1;
  if (num == NUM_UNITS) {
    char *base = strchr(cmd, '<') + 1;  // Move to first number
    char *numEnd;
    bool set = true;
    for (int i = 0; i < NUM_UNITS; i++) {
      float max = strtod(base, &numEnd);  //TODO could add more error checking here with errno and end, start
      set &= units[i]->setMaxTemp(max);
      base = strchr(numEnd, ',') + 1;       // TODO could be more robust by checking for null
    }
    Serial.print(set);
    Serial.print("#");
  } else {
    Serial.print("0#");
  }
  return 0;
}







//##########################################################

int setControlThreshold(char *cmd) {
  // !SET:C:THRESHOLD*# 
  bool set = true;
  for (int i = 0; i < NUM_UNITS; i++) {
    set &= units[i]->setControlMode(THRESHOLD);
  }
  Serial.print(set);
  Serial.print("#");
  return 0;
}



int setControlPWM(char *cmd) {
  OPTIMIZE_DUTY_CYCLE = false;
  //!SET:OPTIMIZE_DUTY_CYCLE:<unitnumber, period, dutycycle in percent>*# //
  bool set = true;
  for (int i = 0; i < NUM_UNITS; i++) {
    set &= units[i]->setControlMode(DUTYCYCLE);
  }
  Serial.print(set);
  Serial.print("#");
  return 0;
}


int setDutyCycleMode(char *cmd) {
   // !SET:OPTIMIZE_DUTY_CYCLE:<true>*# // true = 0 default PID, false = 1  Opti
  //set OPTIMIZE_DUTY_CYCLE
  //Serial.println(cmd);
  int num = countChars(cmd, ',') + 1;
  //Serial.println(num);
  bool *nextOptiMode = (bool *)malloc(num * sizeof(bool));

  char *base = strchr(cmd, '<') + 1;  // Move to first number
  char *numEnd;
  for (int i = 0; i < num; i++) {
    nextOptiMode[i] = strtod(base, &numEnd);  //TODO could add more error checking here with errno and end, start
    base = strchr(numEnd, ',') + 1;         // TODO could be more robust by checking for null
    //Serial.println(nextOptiMode[i]);
  }
  if (nextOptiMode[0] == 1) {
    OPTIMIZE_DUTY_CYCLE = true;
    for (int i = 0; i < num; i++) {
      pidUnits[i]->SetMode(MANUAL);
      //Serial.print("Controlmode_DUTY_CYCLE: Optimized");
    }
  } 
  else if (nextOptiMode[0] == 0) {
    OPTIMIZE_DUTY_CYCLE = false;
    for (int i = 0; i < num; i++) {
      pidUnits[i]->SetMode(AUTOMATIC);
      //Serial.print("Controlmode_DUTY_CYCLE: PID");
  
    }
    
  } 
  else {
    //Serial.println("Enter a Number: true = 0 for PID, false = 1 Optimized");
  }
  
  Serial.print("1#");
  free(nextOptiMode);
  return 0;
  
}





int setToDefault(char *) {
  //!SET:DEFAULT*#
  //Serial.println("DEFAULT");
  for (int i = 0; i < NUM_UNITS; i++) {
    units[i]->setMaxTemp(configTMax[i]);
    units[i]->setTargetTemp(configTSet[i]);
    units[i]->setControlMode(DUTYCYCLE);
  }
}


int setEnableHeating(char *cmd) {
  //!SET:ENABLE_HEATING:<true>*# // true = 0, false = 1 !SET:ENABLE_HEATING:<1>*#
  //set ENABLE_HEATING
  //Serial.println(cmd);
  int num = countChars(cmd, ',') + 1;
  //Serial.println(num);
  bool *nextACMode = (bool *)malloc(num * sizeof(bool));

  char *base = strchr(cmd, '<') + 1;  // Move to first number
  char *numEnd;
  for (int i = 0; i < num; i++) {
    nextACMode[i] = strtod(base, &numEnd);  //TODO could add more error checking here with errno and end, start
    base = strchr(numEnd, ',') + 1;         // TODO could be more robust by checking for null
    //Serial.println(nextACMode[i]);
  }
  if (nextACMode[0] == 1) {
    ENABLE_HEATING = true;
  } 
  else if (nextACMode[0] == 0) {
    ENABLE_HEATING = false;
  } 
  else {
    //Serial.println("Enter a Number: true = 0, false = 1");
  }
  //Serial.print("ENABLE_HEATING:");
  //Serial.println(ENABLE_HEATING);
  Serial.print("1#");
  free(nextACMode);
  return 0;
  //
}

int resetOvertemp(char *cmd) {
  OVERTEMP = false;
  Serial.print("1#");
}

int getVersion(char *cmd) {
  Serial.print(ARDUINO_TYPE);
  Serial.print(":");
  Serial.print(VERSION);
  Serial.print("#");
}



void serialCommand() {
  int s = -1;
  int e = -1;
  char beginDelim = '!';
  char *beginCmd;
  char endDelim = '*';
  char *endCmd;
  char *command;
  bool done = false;
  bool unknown = false;

  // determin substring
  if (Serial.available() > 0) {
    done = updateBufferUntilDelim('#');
  }

  if (done) {
    s = -1;
    if ((beginCmd = strchr(input_buffer, beginDelim)) != NULL) {
      s = beginCmd - input_buffer;
    }

    e = -1;
    if ((endCmd = strchr(input_buffer, endDelim)) != NULL) {
      e = endCmd - input_buffer;
    }

    unknown = true;
    //check if valid command
    if (e != -1 && s != -1) {
      command = beginCmd + 1;
      *endCmd = '\0';
      //Command=Command.substring// toDO: # will cause "" to display with rules..
      if (ACK_COMMANDS) {
        //Serial.print("received command: '");
        //Serial.print(command);
        //Serial.println("'");
      }
      

      //check for known commands
      for (int i = 0; i < sizeof(cmdHandler) / sizeof(*cmdHandler); i++) {
        if (strncmp(cmdHandler[i].id, command, strlen(cmdHandler[i].id)) == 0) {
          cmdHandler[i].handler(command);
          unknown = false;
          input_buffer[0] = '\0';  // "Empty" input buffer
          break;
        }
      }
    }

    if (unknown) {
      Serial.println("UNKNOWN");
      if (displaySerialComm) {
        Serial.print("Received input: ");
        Serial.println(input_buffer);
        getCommands(input_buffer);
      } 
      else {
        Serial.println("#");
      }
      Serial.flush();
    }
  }
}

//*****************************************************************
bool updateBufferUntilDelim(char delim) {
  char nextChar;
  while (Serial.available() > 0) {
    nextChar = Serial.read();
    if (nextChar == delim) {
      input_buffer[input_pos] = '\0';
      input_pos = 0;
      return true;
    } 
    else if (nextChar != '\n') {
      input_buffer[input_pos % (INPUT_BUFFER_SIZE - 1)] = nextChar;  // Size - 1 to always leave room for \0
      input_pos++;
    }
  }
  return false;
}


//###### Utility #############################################
int countChars(char *s, char c) {
  return *s == '\0'
           ? 0
           : countChars(s + 1, c) + (*s == c);
}

char *string2char(String command) {
  if (command.length() != 0) {
    char *p = const_cast<char *>(command.c_str());
    return p;
  }
}
//END Communication

//###################################################################
//Versuche Auswertung Serial Port

void print_Data(int i) {
  float Temp = units[i]->getAvgTemp();
  bool Out = units[i]->getOutput();
  float Duty = units[i]->getDutyCycle();
  
  Serial.print(i);Serial.print(",");Serial.print(currentTime);Serial.print(": temp ");Serial.print(TempInput[i]);Serial.print(", output ");Serial.print(Out);Serial.print(",  duty ");Serial.print(Duty);
  Serial.print(", tempDiff ");Serial.print(configTSet[i]-Temp);Serial.print(", dutyNew ");Serial.print(newDutyCycle[i]);Serial.print(", enableAll ");Serial.print(ENABLE_HEATING);
  Serial.println("# ");
}


//#############################
//pid regler --> neuer DutyCycle in Heatunit

int setNewDutyCyclePID(int i){
  //if( isnan(newDutyCycle[i]) ) {
    units[i]->setNewDutyCycle(newDutyCycle[i]/1000); // durch 1000 für Prozent
  //  ERROR = true; // toDO: set LED dauerhaft
  // } else {
  //   units[i]->setNewDutyCycle(0.0);
  // }
  return 0;
}


int getTemperatureInput(int i){
  //TempInput[i] = units[i]->getAvgTemp();
  TempInput[i] = units[i]->getTemp();
  return 0;
}

void enableTemperatureMeasurement() {
  digitalWrite(pinOpto, HIGH); //NPN
  delayMicroseconds(OptoDelay);
}

void disableTemperatureMeasurement() {
  digitalWrite(pinOpto, LOW);
}



//####################################################################
//####################################################################
//####################################################################


void loop() {
  serialCommand();
  currentTime = millis();

  // timeCycle
  if ((currentTime - previousTime > timeUpdateCycle) && ENABLE_HEATING) {
    
    // for (int i = 0; i < NUM_UNITS; i++) {
    //   print_Data(i);
    // }

    enableTemperatureMeasurement();
    for (int i = 0; i < NUM_UNITS; i++) {
      units[i]->temperatureMeasurement(); 
    }
    disableTemperatureMeasurement();
    
    //Serial.println(";");

    if (counter % cycleSecond < 1) {
      //
    }

    if (counter % cycleFiveSeconds < 1) {
      //
    }

    previousTime = currentTime;
    counter++;
    if (counter > 2 ^ 10) {
      counter == 0;
    }
  }



  bool noError = true;
  // run always (loop speed)
  for (int i = 0; i < NUM_UNITS; i++) {
    ErrorCode[i] = units[i]->checkForError();
    noError &= ErrorCode[i] == 0;

    // run PID always, get average temperature
    getTemperatureInput(i);
    pidUnits[i]->Compute();
    setNewDutyCyclePID(i);
    
    if ( (ENABLE_HEATING || HEAT_DURING_MEAS) && ErrorCode[i] == 0 ) {
      units[i]->control(); // function to select output value within class (!important!)
      units[i]->output(); //set this selected value
    }
    else {
      if (ErrorCode[i] == 3) {
        // case 3: Overtemp detected
        digitalWrite(pinRelayHighVoltage, LOW);
        OVERTEMP = true;
      }
      units[i]->disableOutput(); //disable relais of this unit
    }
  }

  // separate security check: No Overtemp? -> enable 150V DC Relay
  bool noOvertemp = true;
  for (int i = 0; i < NUM_UNITS; i++) {
    // over all ErrorCode: When none are 3 then -> turn on
    noOvertemp &= ErrorCode[i] != 3;
  }
  digitalWrite(pinRelayHighVoltage, noOvertemp && !OVERTEMP);


  // Set Error LED
  digitalWrite(pinErrorLED, noError && !OVERTEMP); //active low!
  digitalWrite(pinErrorSound, noError && !OVERTEMP); //active low!


}
