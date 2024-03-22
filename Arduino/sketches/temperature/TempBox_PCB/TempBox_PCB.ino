#define ARDUINO_TYPE "TEMPBOX"
#define VERSION "3" // After every communication protocol change, increase this number and commit with ARDUINO_TYPE:VERSION as message!
#include <Arduino.h>
#include <SPI.h>
#include "MAX31865.h" // https://github.com/olewolf/arduino-max31865

// Include our libs
#define LIB_HOME /home/hackelberg/git/MPIMeasurementDevices/Arduino/lib/
#define IDENT(x) x
#define STR(a) STR_(a)
#define STR_(a) #a
#define INCLUDE_LIB(lib) STR(IDENT(LIB_HOME)IDENT(lib))
#include INCLUDE_LIB(communication.h)


// Timing
#define delaySensors 300    // [ms] Delay with which sensors are read/updated
#define delayResetTemp 10000 // [ms] reset time for RTDs after which faults were detected 
#define delayAlarmOn 2000    // [ms] for SU to notice
unsigned long current = 0;
unsigned long previous = 0;

#define BAUD_RATE 9600

boolean ACK_COMMANDS = false;  // display output of every received command 
boolean PRINT_FAULTS = false;  // display fault type message of sensor
boolean PRINT_ERRORS = false;  // display error type message of sensor: SPAMS SERIAL!
boolean SERIAL_PRINT_UNKNOWN = false; // printing wrong commands and list of all commands
double VAL_INIT_MAX_TEMPS[] = {70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,70.0,60.0,60.0,60.0};// DOUBLE-CHECK LENGTH!!
const unsigned int num_sensors = sizeof(VAL_INIT_MAX_TEMPS)/sizeof(*VAL_INIT_MAX_TEMPS);

// LED PCB to front of box
#define P_LED_Status 2
#define P_LED_Alarm  3

/////////////////////////////////////////////////
// Analog Pins for SeFo Temperature Monitoring //
/////////////////////////////////////////////////
int P_Temp_RightCoil = A1; //on coil core
int P_Temp_LeftCoil  = A2; //on coil core
int P_Temp_Cabinet   = A3;

// Define temperature variables for mapping of analog values
double  Temp_Low_SeFo      = 2.8;
double  Temp_High_SeFo     = 91;
double  Temp_Low_Cabinet   = 3.8;
double  Temp_High_Cabinet  = 86.5;

//////////////////////////////////////////////////////
// CS numbering corresponds to label of temp-ports  //
//////////////////////////////////////////////////////
#define PIN_RTD_CS_1 22 
#define PIN_RTD_CS_2 23 
#define PIN_RTD_CS_3 24 
#define PIN_RTD_CS_4 25 
#define PIN_RTD_CS_5 26
#define PIN_RTD_CS_6 27
#define PIN_RTD_CS_7 28
#define PIN_RTD_CS_8 29
#define PIN_RTD_CS_9 30
#define PIN_RTD_CS_10 31
#define PIN_RTD_CS_11 32
#define PIN_RTD_CS_12 33 
#define PIN_RTD_CS_13 34 
#define PIN_RTD_CS_14 35
#define PIN_RTD_CS_15 36
#define PIN_RTD_CS_16 37 
#define PIN_RTD_CS_17 38 
#define PIN_RTD_CS_18 39

const int rtd_pins[] = {PIN_RTD_CS_1, PIN_RTD_CS_2, PIN_RTD_CS_3, PIN_RTD_CS_4, PIN_RTD_CS_5, PIN_RTD_CS_6, PIN_RTD_CS_7, PIN_RTD_CS_8,
          PIN_RTD_CS_9, PIN_RTD_CS_10, PIN_RTD_CS_11, PIN_RTD_CS_12, PIN_RTD_CS_13, PIN_RTD_CS_14, PIN_RTD_CS_15, PIN_RTD_CS_16, PIN_RTD_CS_17, PIN_RTD_CS_18};
const unsigned int num_rtd_clicks = sizeof(rtd_pins)/sizeof(*rtd_pins);
MAX31865_RTD *rtd_sensors[num_rtd_clicks];

boolean STATE_ALARM[num_sensors];

#define PIN_ALARM 12  // alarm pin. active-high. LOW: all good, will be pulled HIGH as avg temperature exceeds temp_max
                      // this way the other Arduino can use an INTERNAL_PULLUP for init. If disconnected or similar - fail.


#define R_ref 470      //reference resistor on board of MAX31865
#define temp_init -1  //initial temp value

double temp_max[num_sensors];  //define at which temperature the arduino raises alarm
double temp_current[num_sensors];
int temp_status[num_sensors];
int temp_counter[num_sensors];
double Temp;  // initiate once, not in every forloop
int noAlarm;  // detect if all temps were okay

// Preallocate update variables to avoid heap-fragmentation
double update_double[num_sensors];
int update_int[num_sensors];

// Communication
int getAllTemps(char*);
int getTemps(char*);
int getMaxTemp(char*);
int setMaxTemp(char*);
int getVersion(char*);
int getCommands(char*);
int getFaults(char*);
//int setFoo(char*);

commandCallback_t cmdHandler[] = {
  {"GET:ALLTEMPS", getAllTemps},
  {"GET:TEMPS", getTemps},
  {"GET:MAXT", getMaxTemp},
  {"SET:MAXT", setMaxTemp},
  {"VERSION", getVersion},
  {"GET:COMMANDS", getCommands},
  {"GET:FAULTS", getFaults},
  //{"FOO", setFoo} //for new function: Add command declaration and then command definition (function body)
};

#define INPUT_BUFFER_SIZE 256
SerialHandler serialHandler = SerialHandler(INPUT_BUFFER_SIZE, cmdHandler, sizeof(cmdHandler)/sizeof(*cmdHandler));


int getCommands(char*) {
  Serial.println("Valid Commands (without quotes):");
  Serial.println("'!GET:ALLTEMPS*' Get temperatures of all sensors");
  Serial.println("'!GET:TEMPS:<2,5,6>*' Get individual temperatures e.g. from sensors 2, 5 and 6");
  Serial.println("'!GET:FAULTS*' Get all error codes: 501 ... 507. See Code for List.");
  Serial.println("----------------------------------------------------");
  Serial.println("'!GET:MAXT*' Read the alarm-trigger-temperature for all sensors");
  Serial.println("'!SET:MAXT:<65,80, ...,70>*' Set the max-temp (int!) for all sensors");
  Serial.println("----------------------------------------------------");
  Serial.println("'!VERSION*' Return the uploaded code version");
  Serial.println("'!GET:COMMANDS*' List all commands");
  Serial.println("----------------------------------------------------");
  Serial.println("#");
}

// ********************************************************************************************************
void setup() {  // run once
  Serial.begin( BAUD_RATE );
  Serial.setTimeout(1000); //1000ms standard
  
  /* Initialize SPI communication. */
  SPI.begin( );
  SPI.setClockDivider( SPI_CLOCK_DIV16 );
  SPI.setDataMode( SPI_MODE3 );
  
  /* Allow the MAX31865 to warm up. */
  delay( 200 );

  /* configure PINs RTC Click */
  pinMode(P_LED_Alarm, OUTPUT);
  pinMode(P_LED_Status, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);
  for (int i = 0; i < num_rtd_clicks; i++) {
    pinMode(rtd_pins[i], OUTPUT);
  }
  
  /* configure PINs Sefo Analog */
  pinMode(P_Temp_RightCoil, INPUT);
  pinMode(P_Temp_LeftCoil, INPUT);
  pinMode(P_Temp_Cabinet, INPUT);

  /* Configure:
    V_BIAS: enabled
    Auto-conversion: enabled
    1-shot: disabled
    3-wire: enabled
    Fault detection:  automatic delay
    Fault status:  auto-clear
    50 Hz filter: enabled
    Low threshold:  0x0000
    High threshold:  0x7fff
  */
  for (int i = 0; i < num_rtd_clicks; i++) {
    rtd_sensors[i] = new MAX31865_RTD(MAX31865_RTD::RTD_PT100, rtd_pins[i], R_ref);
    rtd_sensors[i]->configure( true, true, false, true, MAX31865_FAULT_DETECTION_NONE, true, true, 0x0000, 0x7fff );
  }

  for (int i=0;i<num_sensors;i++) { 
    temp_max[i] = VAL_INIT_MAX_TEMPS[i];
    temp_current[i] = temp_init; //init for all temps
    temp_status[i] = 0;
    temp_counter[i] = 0;
    STATE_ALARM[i] = false;
  }
  
  digitalWrite(PIN_ALARM, HIGH); //init HIGH (active-high): let it be pulled low when all temps were checked.
  //Serial.println("Initialized.#");
}



//********************************************************************************************************
//   FUNCTIONS   BLOCK
//********************************************************************************************************

void update_temperatures() {  // always updating ALL, if asked for specific ones, then these will be read from temp_current[]
  
  digitalWrite(P_LED_Status, HIGH);
  //digitalWrite(LED_BUILTIN, HIGH);

  for (int i = 0; i < num_rtd_clicks; i++) {
    read_temperatures(rtd_sensors[i], i);
  }

  //Analog SEFO Sensors:
  //I'm not using map() because maps()'s output is an Int and I want full precsion of the temperatures
  //toDo: status auch hier richtig setzten
  temp_current[18] = analogRead(P_Temp_RightCoil)/1024.0*(Temp_High_SeFo-Temp_Low_SeFo)+Temp_Low_SeFo; //map(analogRead(P_Temp_RightCoil), 0,1024,Temp_Low_SeFo,Temp_High_SeFo);
  temp_current[19] = analogRead(P_Temp_LeftCoil)/1024.0*(Temp_High_SeFo-Temp_Low_SeFo)+Temp_Low_SeFo; //map(analogRead(P_Temp_LeftCoil), 0,1024,Temp_Low_SeFo,Temp_High_SeFo);
  temp_current[20] = analogRead(P_Temp_Cabinet)/1024.0*(Temp_High_Cabinet-Temp_Low_Cabinet)+Temp_Low_Cabinet; //map(analogRead(P_Temp_Cabinet), 0,1024,Temp_Low_Cabinet,Temp_High_Cabinet);

  digitalWrite(P_LED_Status, LOW);
  //digitalWrite(LED_BUILTIN, HIGH);
}

//*******************************************
void check_and_set_alarm() {

  for (int i=0;i<num_sensors;i++) { //check all sensors for alarm
    if (temp_current[i] >= temp_max[i] && temp_current[i] < 250 && temp_current[i] > -10) {
      STATE_ALARM[i] = true;
    } else {
      STATE_ALARM[i] = false;
    }

    if (temp_status[i] >= 500 && PRINT_FAULTS) { //delayAlarm so not to spam with every temp update
      print_fault(temp_status[i],i);
    }
  }

  noAlarm = 0;
  for (int i=0;i<num_sensors;i++) {  //set ALARM_Pin if any alarm detected
    if (STATE_ALARM[i]) {
      digitalWrite(PIN_ALARM, HIGH);  // Alarm on. Active High.
      digitalWrite(P_LED_Alarm, HIGH);
      if(PRINT_ERRORS) {
        Serial.print("ALARM:SENSOR:");
        Serial.print(i+1);
        Serial.print(":READING:");
        Serial.print(temp_current[i]);
        Serial.println("#");
      }
      delay(delayAlarmOn); // give some time for Arduino to register
    }
    else {
      noAlarm++;
    }
  }

  if ( noAlarm == num_sensors ) {
    digitalWrite(PIN_ALARM, LOW);  // Alarm off. All Temps clear. grounded internal PULL_UP of other arduino.
    digitalWrite(P_LED_Alarm, LOW);
  } 

}


//*******************************************
void print_fault(int fault, int ndx) {
  
  Serial.print("FAULT:SENSOR:");
  Serial.print(ndx+1);

  switch (fault) {
    case 500: 
      Serial.print(":HIGH_THRESHOLD");
      break;

    case 501:
      Serial.print(":LOW_THRESHOLD");
      break;

    case 502:
      Serial.print(":REFIN");
      break;

    case 503:
      Serial.print(":REFIN_FORCE");
      break;

    case 504:
      Serial.print(":RTDIN_FORCE");
      break;

    case 505:
      Serial.print(":VOLTAGE");
      break;

    case 506:
      Serial.print(":Unrealistic Temperaturerange < -10 or > 250 (hardcoded)");
      break;

    case 507:
      Serial.print(":OTHER");
      break;
      
    case 0:
      Serial.print(":NONE");
      break;
      
    default:
      Serial.print(":UNKOWN...");
      break;
    break;
  }
  Serial.println("#");
}



//*******************************************
void print_temperatures(int *sensors, int sensors_size) {  
  int index = 0;
  Serial.print("TEMPS:");
  for(int i=0;i<sensors_size;i++) {
    index = sensors[i];
    if (index > 0 && index <= num_sensors) {
      Serial.print(temp_current[index-1]);
      if (i < sensors_size-1) {
        Serial.print(",");
      }
    } else {
      Serial.print("index-error");
    }
  }
  Serial.println("#");
}


//*******************************************
void print_maxima(int *sensors, int sensors_size) {
  int index = 0;
  Serial.print("MAXT:GET:");
  for(int i=0;i<sensors_size;i++) {
    index= sensors[i]; 
    if (index > 0 && index <= num_sensors) {
      Serial.print(temp_max[index-1]);
      if (i < sensors_size-1) {
        Serial.print(",");
      }
    } else {
      Serial.print("index-error");
    }
  }
  Serial.println("#");
}


//*******************************************
void set_maxima(double *maxima, int num) {
  double val;
  if (num == num_sensors) { //fill array
    Serial.print("MAXT:SET:");
    for(int i=0;i<num_sensors;i++) {
      val = maxima[i];
      temp_max[i] = val;
      Serial.print(val);
      if (i < num_sensors-1) {
        Serial.print(",");
      }
    }
    Serial.println("#");
  } else {
    Serial.print("Received ");
    Serial.print(num);
    Serial.print(", please submit the expected number of max temperatures: ");
    Serial.print(num_sensors);
    Serial.println("#");
  }
}

int getAllTemps(char *cmd) {
  int sensors[num_sensors];
  for(int i=0;i<num_sensors;i++) {
    sensors[i] = i + 1; // Sensors are counted 1 ... n
  }
  print_temperatures(sensors, num_sensors);
  Serial.flush();     
}

int getTemps(char *cmd) {
  int num = serialHandler.countChars(cmd, ',') + 1;

  char *base = strchr(cmd, '<') + 1; // Move to first number
  int result = serialHandler.readNumbers(base, update_int, num);

  if (result == 0) {
    print_temperatures(update_int, num);
  }
        
  Serial.flush();  
}

int getMaxTemp(char *cmd) {
  int sensors[num_sensors];
  for(int i=0;i<num_sensors;i++) {
    sensors[i] = i + 1; // Sensors are counted 1 ... n
  }
  print_maxima(sensors, num_sensors);
  Serial.flush();     
}

int setMaxTemp(char *cmd) {
  int num = serialHandler.countChars(cmd, ',') + 1;

  char *base = strchr(cmd, '<') + 1; // Move to first number
  int result = serialHandler.readNumbers(base, update_double, num);

  if (result == 0) {
    set_maxima(update_double, num);
  }

  set_maxima(update_double, num);
  Serial.flush();  
}

int getVersion(char *cmd) {
  Serial.print(ARDUINO_TYPE);
  Serial.print(":");
  Serial.print(VERSION);
  Serial.print("#");
  Serial.flush(); 
}


int getFaults(char*) {
    for(int i=0;i<num_sensors;i++) {
    print_fault(temp_status[i],i);
  }
}



//*****************************************************************
void check_counter() {

  for(int i=0;i<num_sensors;i++) {
    if(delaySensors*temp_counter[i] > delayResetTemp ) {
      temp_current[i] = temp_init; // Sensors are counted 1 ... n
      temp_counter[i] = 0;
    }
  }
}


//*****************************************************************
void read_temperatures(MAX31865_RTD *rtd, int index) {
  rtd->read_all( );
  if ( rtd->status( ) == 0 ) {
    double mtemp = rtd->temperature( );    //hinter Temp Position von x Temp[i]
  
    if (mtemp > -10 && mtemp < 250) { //realistic range
      temp_current[index] = mtemp;
      temp_status[index] = 0;
      temp_counter[index] = 0;
    } else {      
      temp_status[index] = 506; //error code für wrong range
      temp_counter[index]++;
    }
  } 
  else {
    if ( rtd->status( ) & MAX31865_FAULT_HIGH_THRESHOLD ) {          
      temp_status[index] = 500;
    } else if ( rtd->status( ) & MAX31865_FAULT_LOW_THRESHOLD ) { 
      temp_status[index] = 501; 
    } else if ( rtd->status( ) & MAX31865_FAULT_REFIN ) {
      temp_status[index] = 502;
    } else if ( rtd->status( ) & MAX31865_FAULT_REFIN_FORCE ) {
      temp_status[index] = 503;
    } else if ( rtd->status( ) & MAX31865_FAULT_RTDIN_FORCE ) {
      temp_status[index] = 504;
    } else if ( rtd->status( ) & MAX31865_FAULT_VOLTAGE ) {
      temp_status[index] = 505;
    } else {
      temp_status[index] = 507; //other
    }
    temp_counter[index]++;
  }

}



///////////////////////////////////////////////////////////////////
//*****************************************************************
///////////////////////////////////////////////////////////////////

void loop() {
  serialHandler.read();
  unsigned long current = millis();
  if (current - previous >= delaySensors) {
    previous = current;
    update_temperatures();  
    check_and_set_alarm();
    check_counter();
  }
}
