
#define ARDUINO_TYPE "SURVBOX"
#define VERSION "3"
#define displaySerialComm 1
#define LIB_HOME /home/hackelberg/git/MPIMeasurementDevices/Arduino/lib/

#define IDENT(x) x
#define STR(a) STR_(a)
#define STR_(a) #a
#define INCLUDE_LIB(lib) STR(IDENT(LIB_HOME)IDENT(lib))
#include INCLUDE_LIB(communication.h)
#include INCLUDE_LIB(rp_watchdog.h)


#include <Adafruit_SleepyDog.h>

/////////////////////////
//// SETUP Variables ////
/////////////////////////

//Reset times
int RP_DownTime = 10000; // [ms] 10 s How long the RPs are switched off during a Reset
int RP_WakeUpTime = 10000000; // [us] time in for the RPs to wake up after restart

// WD: Watchdop of Redpitaya and GUI
boolean WatchDogEnabled = true;  // RedPitaya Watchdog : toDO : testing, turn true again for normal startup
boolean WatchDogGUIEnabled = false; // GUI Watchdog: toDo: implementaion in julia.
long WD_tmax = 50000;  // [us] how long RPs are allowed to not answer
long WD_GUI_tmax = 50000; //[us] how long is the GUI allowed not to answer

//======================================================================================

/////////////////
////VARIABLES////
/////////////////

//Define State Variable inital values
boolean GUI_ENABLEAC = false;      // PERMISSION: GUI sends an "Enable_AC" when a measurement is about to start. Only then the relais should open.
boolean SU_READY = false;          // status: ReadyToMeasure, SU detectes no fails and is ready to rumble
boolean SURVEILLIANCE = true;      // status: SU enabled - active high.
boolean RP_RESET_CB = false;       // status: Resetting RPs - active high (CB: ControlBox)
boolean SU_FAIL_ACK = false;       // status: Acknowlegde fail and restart wohle surveillance unit - active high
boolean PERMISSION = true;         // status: [important!] The scanner will only restart after a fail when Acknowlegde was pressed
boolean LNA_STATUS = false;        // status: of the power suppy for LNAs
boolean DFFieldON = false;         // status: measurement of the actual drive field, when AC signal is sent
boolean SEFO_24V = false;          // status: is the SEFO button pressed? enables 24V and SefoRoboter
boolean OVERTEMP = false;          // fail-status: if single temperature measurement exceeded, active HIGH
boolean AEs_FAIL = false;          // fail-status: any single AE tech fail
boolean GUI_FAIL = false;          // fail-status: If GUI does not react (Gui WatchDog)
boolean ANY_FAIL = false;          // fail-status: sum of all fails
boolean HEATING_ENABLE = true;     // status: External Heating of Copper Capacitors
boolean HEATING_PERMISSION = true; // status: User-Disable function (Serial), otherwise true

//Added for porridge
boolean RSD_Delta = true;



//ACK buffer variables
boolean ACK1_SU = false; //for SU-Acknowledge
boolean ACK2_SU = false;
boolean ACK1_RP = false; //for RP-Reset
boolean ACK2_RP = false;

//LED states
boolean LED_Measurement = false;
boolean LED_DFFieldON = false;
boolean LED_FAIL = false;
boolean LED_RP_ALIVE = false;
boolean LED_SU_FAIL = false;

//Timing
unsigned long t0;
unsigned long t1;
unsigned long tcycle;
unsigned long LCDtime;
int countdownMS;

//WatchDog
//WatchDog
boolean WD_FAIL = true; // init with fail, so watchdog needs to be runing to turn off
boolean WDboot = true;
RedPitayaWatchdog rpWatchdogs [] = {
  RedPitayaWatchdog(3, 2, WD_tmax),
  //RedPitayaWatchdog(4, 3, WD_tmax),
  //RedPitayaWatchdog(7, 6, WD_tmax),
  //RedPitayaWatchdog(9, 10, WD_tmax),
  //RedPitayaWatchdog(15, 14, WD_tmax),
  //RedPitayaWatchdog(18, 17, WD_tmax),
  //RedPitayaWatchdog(21, 20, WD_tmax),
  //RedPitayaWatchdog(22, 13, WD_tmax),
  //RedPitayaWatchdog(25, 24, WD_tmax),
  //RedPitayaWatchdog(28, 27, WD_tmax),
};
int numRPWatchdogs = sizeof(rpWatchdogs)/sizeof(*rpWatchdogs);


//WatchDog GUI
unsigned long WD_GUI_t0;

// Errormessage
int32_t Errormessage = 0x10000;

// Serial ACQ
String SerialACQ = "ACQ#";

//LCD CLEARING
String LCDclear = "                     ";


//======================================================================================
////////////
////PINS////
////////////

const int P_ENABLEAC = 40;        // Relais that blocks AC signal (old: EnableAC)

// Added for Porrdige
const int P_RSD_Delta = 30; //Optokoppler to Deltas

const int P_RP_RSD = A1;           // Red Pitaya Remote RSD, UND Verknüpfung vor Ausgang
const int P_RP_Reset = A0;         // Red Pitaya shutdown supply voltage relais
// const int P_AEs_Disable = 46;      // AETechron's RSD
// const int P_Delta_RSD = 38;        // Delta current sources RSD - Not used now

///master
const int P_Al_RP_M = 2; //#28;
///slave 1
const int P_Al_RP_S1 = 5;
///slave 2
const int P_Al_RP_S2 = 8;
///slave 3
const int P_Al_RP_S3 = 11;
///slave 4
const int P_Al_RP_S4 = 16;
///slave 5
const int P_Al_RP_S5 = 19;
///slave 6
const int P_Al_RP_S6 = 12;
///slave 7
const int P_Al_RP_S7 = 23;
///slave 8
const int P_Al_RP_S8 = 26;
///slave 9 NOT IN USE
const int P_Al_RP_S9 = 29;

/////////////////////
////Communication////
/////////////////////
int getAnalog(char*);
int getDigital(char*);
int setDigital(char*);
int getStatus(char*);
int getStats(char*);
int resetWatchdog(char*);
int disableWatchdog(char*);
int enableWatchdog(char*);
int getWatchdogStatus(char*);
int enableHeating(char*);
int disableHeating(char*);
int resetFailState(char*);
int resetRedPitayas(char*);
int enableSurveillance(char*);
int disableSurveillance(char*);
int getCycleTime(char*);
int resetArduino(char*);
int enableAC(char*);
int disableAC(char*);
int disableGUIWatchdog(char*);
int enableGUIWatchdog(char*);
int debug(char*);
int getCommands(char*);
int getVersion(char*);

commandCallback_t cmdHandler[] = {
  {"GET:ANALOG:", getAnalog},
  {"SET:DIGITAL:", setDigital},
  {"GET:DIGITAL:", getDigital},
  {"GET:STATUS", getStatus},
  {"GET:STATS", getStats},
  {"RESET:WD", resetWatchdog},
  {"GET:STATUS:WD", getWatchdogStatus},
  {"DISABLE:WD", disableWatchdog},
  {"ENABLE:WD", enableWatchdog},
  {"DISABLE:HEATING", disableHeating},
  {"ENABLE:HEATING", enableHeating},
  {"RESET:FAIL", resetFailState},
  {"RESET:RP", resetRedPitayas},
  {"DISABLE:SURVEILLANCE", disableSurveillance},
  {"ENABLE:SURVEILLANCE", enableSurveillance},
  {"GET:CYCLETIME", getCycleTime},
  {"RESET:ARDUINO", resetArduino},
  {"ENABLE:AC", enableAC},
  {"DISABLE:AC", disableAC},
  {"DISABLE:WD:GUI", disableGUIWatchdog},
  {"ENABLE:WD:GUI", enableGUIWatchdog},
  {"Debug", debug},
  {"VERSION", getVersion},
  {"GET:COMMANDS", getCommands},
  //{"FOO", setFoo} //for new function: Add command declaration and then command definition (function body)
};


#define INPUT_BUFFER_SIZE 256
SerialHandler serialHandler = SerialHandler(INPUT_BUFFER_SIZE, cmdHandler, sizeof(cmdHandler)/sizeof(*cmdHandler));

int getCommands(char* cmd){
    //Serial.print(Command);"Debug,WDFAIL_S1-4/M?(without quotes, Structure !COMMAND*#)
    Serial.println("\n");
    Serial.println("Valid Commands (without quotes, Structure: !COMMAND*# (e.g. !GET:STATS*#):"); //toDo Append #...!!!!!!!!!!!!!!!!!!!
    Serial.println("'GET:STATS' display all SU states and fails.");
    Serial.println("'GET:STATUS' Get Errorcodes 8 Bits");
    Serial.println("'GET:CYCLETIME' Time of Arduinocycle in microseconds");
    Serial.println("'GET:STATUS:WD' Get Watchdog Status 3 Bits |WD Enabled?| Master Fail| Slave Fail|");
    Serial.println("'GET:ANALOG:A1' Get Analog value in Integer from A1");
    Serial.println("'GET:DIGITAL:1' Get DIGITAL value from Pin 1");
    Serial.println("'DISABLE:SURVEILLANCE' Disable Surveilliance. CAREFUL. NO SHUTDOWN!");
    Serial.println("'ENABLE:SURVEILLANCE' Enable Surveilliance");
    Serial.println("'ENABLE:WD' Enable RedPyjamas Watchdog");
    Serial.println("'DISABLE:WD' Disable RedPyjamas Watchdog");
    Serial.println("'ENABLE:HEATING' Enable Heating of Capacitors");
    Serial.println("'DISABLE:HEATING' Disable Heating of Capacitors");
    Serial.println("'RESET:WD' Reset RedPyjamas Watchdog");
    Serial.println("'RESET:FAIL' Reset FailState");
    Serial.println("'RESET:RP' Reset RedPyjamas.");
    Serial.println("'RESET:ARDUINO' Reset Arduino");
    Serial.println("'ENABLE:WD:GUI' Enable GUI Watchdog");
    Serial.println("'DISABLE:WD:GUI' Disable GUI Watchdog");
    Serial.println("#");
}

//======================================================================================
void setup() //
{

  ////////////////////////
  ///Define InOut State///
  ////////////////////////
  analogReference(DEFAULT); //5V reference, otherwise set to EXTERNAL and connect with C to pin.

  ///Important
  pinMode(P_ENABLEAC, OUTPUT);

  ///Red Pitayas
  pinMode(P_RP_Reset, OUTPUT);
  pinMode(P_RP_RSD, OUTPUT);
  pinMode(P_Al_RP_M,  INPUT_PULLUP);
  pinMode(P_Al_RP_S1,  INPUT_PULLUP);
  pinMode(P_Al_RP_S2,  INPUT_PULLUP);
  pinMode(P_Al_RP_S3,  INPUT_PULLUP);
  pinMode(P_Al_RP_S4,  INPUT_PULLUP);

  for (int i = 0; i < numRPWatchdogs; i++) {
    rpWatchdogs[i].setup();
  }

  // Serial Monitor
  Serial.begin(9600);
  /* Serial.println("--- Start Serial Monitor SEND_RCVE ---");
    Serial.println(" Type in Box above, . ");
    Serial.println("(Decimal)(Hex)(Character)");
    Serial.println();
  */
  
  Watchdog.enable(4000); // internal Arduino watchdog, is reset at end of loop()
  digitalWrite(P_RP_RSD, LOW);

  //Debugging AETechrons:
  pinMode(49, OUTPUT);
}



//======================================================================================
void ArReset() //Arduino Reset
{
  countdownMS = Watchdog.enable(1); //Arduino watchdog time es set to 1 ms?
  while (1); //Arduino will go into a shutdown because the watchdog will be triggerd in the while loop
}
//======================================================================================
void SCANNER_SHUTDOWN() //Shuts down drive field, SeFo and RPs (Isel?)
{
  digitalWrite(P_RP_RSD, HIGH);
//  digitalWrite(P_Delta_RSD, LOW);
  digitalWrite(P_ENABLEAC, LOW); 
  GUI_ENABLEAC = false; //test: deny AC_enable, wait for new command
//  digitalWrite(P_HeatingRelais, HEATING_ENABLE); // you may heat, if not measuring and not too hot
//  digitalWrite(P_AEs_Disable, HIGH);

//  digitalWrite(49, LOW); //toDo: fynn?
}
//======================================================================================
void SCANNER_ENABLE() //No blocking of drive field and SeFos, RPs are on, (Isel?)
{
  digitalWrite(P_RP_RSD, LOW);
//  digitalWrite(P_Delta_RSD, HIGH);
  digitalWrite(P_ENABLEAC, HIGH); 
//  digitalWrite(P_HeatingRelais, LOW); //turn off heating when measuring
  HEATING_ENABLE = false;
//  digitalWrite(P_AEs_Disable, LOW);

//  digitalWrite(49, HIGH); //toDo: Fynn?
}

//=====================================================================================
//RPs surveillance. Goal: We want to be aware when RPs are not responding anymore. When RPs crash they sonetimes coninue to send their signal like forever.
//Working Principle: Send a 1 to every RP. Aftwerwards the RP sends a 1 back to the Arduino. We measure that second signal with the Arduino. If its a one, we are happy. 
//Aftwerwards we are sending a 0 to the RPs. When the RPs respond again with a zero then the RPs are alive. When we don't get the expected signal from the RPs we wait til it comes.
//When we have to wait longer than a certain time, the Watch Dog Fail is triggered. toDo: Better explanation
void WatchDog()
{
  if (WatchDogEnabled)
  {
    if (WDboot)
    {

      // Initiate Watch Dog start
      WDboot = false;
      for (int i = 0; i < numRPWatchdogs; i++)
      {
        rpWatchdogs[i].reset();
      }
      // lcd.clear();
      // lcd.setCursor(0,0);
      // lcd.print("WD_BOOT");
    }

    WD_FAIL = false;
    for (int i = 0; i < numRPWatchdogs; i++)
    {
      if (!rpWatchdogs[i].check())
      {
        WD_FAIL = true;
        Errormessage = (Errormessage | 1 << (5 + i));
      }
    }

    // WD_FAIL = WD_FAIL_RP_M || WD_FAIL_RP_S1|| WD_FAIL_RP_S2 || WD_FAIL_RP_S3|| WD_FAIL_RP_S4 || WD_FAIL_RP_S5 || WD_FAIL_RP_S6 || WD_FAIL_RP_S7 || WD_FAIL_RP_S8; //||WD_FAIL_RP_S8; toDo: Zurzeit sind nur Slave 1 und Slave 2 eingebaut. Die anderen beiden können nicht antworten.
    //   Serial.println(WD_FAIL_RP_S);
    ////////////////AAAAAAACHTUNGGG Hier wird gerade gepfuscht!!! toDo
    RSD_Delta = rpWatchdogs[0].hasFailed() || rpWatchdogs[1].hasFailed() || rpWatchdogs[2].hasFailed() || rpWatchdogs[3].hasFailed();
    if (!WD_FAIL)
    {
      Errormessage = (Errormessage & (0x3F)); // Clear upper 5 Bits of errormessage
    }
  }
  else
  { // redpitaya watchdog disabled
    WD_FAIL = false;
  }
}

//======================================================================================
void resetWatchDog() //Reset all WD variables. No Shut down of RPs
{
  WDboot = true;
  WD_FAIL = false;
}

//======================================================================================
void checkFailState() //Check whether all state variables are ok
{
  if (RP_RESET_CB) //The Shutdown of the RP's would also be triggerd by SCANNER_SHUTDOWN()
  {
    Serial.println("****** RESET BY CB *******");
    RP_Reset();
    RP_RESET_CB = false;
  }

  if (SU_FAIL_ACK)
  {
    PERMISSION = true; //The scanner will only restart, when Acknowlegde is pressed.
    SU_Reset();
    Serial.println("***** SU_FAIL_ACK *****");
    SU_FAIL_ACK = false;
  }

  // if no switch toggled: CONTINUE CHECKS
  // -----------------------------------------
  SEFO_24V = true;
  //SEFO_24V = !digitalRead(P_SEFO_24V);   // toDo: wire it up! former EMERGENCY SHUTDOWN

  //toDO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  AEs_FAIL = false; //checkAETechrons();
  //toDo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  //OVERTEMP = digitalRead(P_Overtemp);     // low: temp ok. high: Temp fail. Else: bad conn, check tempbox.
  DFFieldON = false;
  //DFFieldON = digitalRead(this will not happen);  // NottoDo: Implement independent Rogowski DF-PickUp Circuit
  LNA_STATUS = true;
  //LNA_STATUS = digitalRead(P_LNA_status);      // toDo: wiring, how to get signal w/o interfering?
 
  // HEATING of capcitors
  if (!OVERTEMP && HEATING_PERMISSION) {
    HEATING_ENABLE = true;
  } else {
    HEATING_ENABLE = false;
  }

  // checks done. update LEDs
//  ControlBox_update_LEDs(); 


  // FINALLY: decide on scanner state
  ANY_FAIL  = (WD_FAIL || !SEFO_24V || OVERTEMP || !LNA_STATUS || AEs_FAIL || GUI_FAIL);

  if (ANY_FAIL) 
  { //in case of any fail, take permission away - wait for new SU_FAIL_ACK
    PERMISSION = false;
  }

  // ---- SCANNER -----
  if ( (!ANY_FAIL && PERMISSION && GUI_ENABLEAC) || !SURVEILLIANCE) //need SEFO to measure?
  {
    SCANNER_ENABLE();
  }
  else
  {
    SCANNER_SHUTDOWN();
  }
}

//======================================================================================
// Serial Communication

int getAnalog(char* cmd) {
  char *numStart = strrchr(cmd, ':');
  if (numStart != NULL) {
    Serial.print(analogRead(strtol(numStart + 1, &numStart, 0)));
  }
  Serial.print("#");
}

int getDigital(char* cmd) {
  char *numStart = strrchr(cmd, ':');
  if (numStart != NULL) {
    Serial.print(digitalRead(strtol(numStart + 1, &numStart, 0)));
  }
  Serial.print("#");
}

int setDigital(char* cmd) {
  // nothing yet? set that port, or to dangerous to implement?
}

int getStatus(char* cmd) {
  Serial.print(Errormessage, BIN);
  Serial.print("#");
}

int getStats(char* cmd) {
  Serial.println("--- debug interface ---");
  Serial.print("SURVEILLIANCE: ");
  Serial.println(SURVEILLIANCE);
  Serial.print("WatchDogEnabled: ");
  Serial.println(WatchDogEnabled);
  Serial.print("WatchDogGUIEnabled: ");
  Serial.println(WatchDogGUIEnabled);

  Serial.print("SU_READY: ");
  Serial.println(SU_READY);
  Serial.print("GUI_ENABLEAC: ");
  Serial.println(GUI_ENABLEAC);

  Serial.print("SEFO_24V: ");
  Serial.println(SEFO_24V);
  Serial.print("LNA_STATUS: ");
  Serial.println(LNA_STATUS);
  Serial.print("DFFieldON: ");
  Serial.println(DFFieldON);
  Serial.print("OVERTEMP: ");
  Serial.println(OVERTEMP);
  Serial.print("HEATING ENABLE: ");
  Serial.println(HEATING_ENABLE);
  Serial.print("WD_FAIL: ");
  Serial.println(WD_FAIL);
  Serial.print("GUI_FAIL: ");
  Serial.println(GUI_FAIL);
  Serial.print("ANY_FAIL: ");
  Serial.println(ANY_FAIL);
    
  Serial.print("LED_RP_ALIVE: ");
  Serial.println(LED_RP_ALIVE);
  Serial.print("LED_SU_FAIL (kombi): ");
  Serial.println(LED_SU_FAIL);

  Serial.print("SU_FAIL_ACK: ");
  Serial.println(SU_FAIL_ACK);
  Serial.print("RP_RESET_CB: ");
  Serial.println(RP_RESET_CB);
  Serial.print("PERMISSION: ");
  Serial.println(PERMISSION);
}

int resetWatchdog(char* cmd) {
  resetWatchDog();
  Serial.print(SerialACQ);
}

int disableWatchdog(char* cmd) {
  WatchDogEnabled = false;
  Serial.print(SerialACQ);
}

int enableWatchdog(char* cmd){
  WatchDogEnabled = true;
  resetWatchDog();
  Serial.print(SerialACQ);
}

int getWatchdogStatus(char* cmd){
  for (int i = 0; i < numRPWatchdogs; i++) {
    Serial.print(rpWatchdogs[i].hasFailed());
  }
  Serial.print("#");
}

int enableHeating(char* cmd){
  HEATING_PERMISSION = true;
  Serial.print(SerialACQ);
}

int disableHeating(char* cmd){
  HEATING_PERMISSION = false; 
  Serial.print(SerialACQ);
}

int resetFailState(char* cmd){
  SU_Reset();
  Serial.print(SerialACQ);
}

int resetRedPitayas(char* cmd){
  Serial.print(SerialACQ);
  RP_Reset();
  Serial.print("#");
}

int enableSurveillance(char* cmd){
  SURVEILLIANCE = true;
  Serial.print(SerialACQ);
}

int disableSurveillance(char* cmd){
  SURVEILLIANCE = false;
  Serial.print(SerialACQ);
}

int getCycleTime(char* cmd){
  Serial.print(SerialACQ);
  Serial.print(tcycle);
  Serial.print("#");
}

int resetArduino(char* cmd){
  Serial.print(SerialACQ);
  ArReset();
}

int enableAC(char* cmd){
  GUI_ENABLEAC = true;
  //Added additional checkFailState() here than the Acknowledge comes when the pin of the AEtechrons is already high/low.
  checkFailState();
  Serial.print(SerialACQ);
}

int disableAC(char* cmd){
  GUI_ENABLEAC = false;
  //Added additional checkFailState() here than the Acknowledge comes when the pin of the AEtechrons is already high/low.
  checkFailState();
  Serial.print(SerialACQ);
}

int disableGUIWatchdog(char* cmd){
  WatchDogGUIEnabled = false;
  Serial.print(SerialACQ);
}

int enableGUIWatchdog(char* cmd){
  WatchDogGUIEnabled = true;
  resetWatchDog();
  Serial.print(SerialACQ);
}

int debug(char* cmd){
  Serial.print("SEFO_24V ");
  Serial.println(SEFO_24V);
  Serial.print("Overtemp ");
  Serial.println(OVERTEMP);
  Serial.print("WD_FAIL ");
  Serial.println(WD_FAIL);
  Serial.print("WDEnable ");
  Serial.println(WatchDogEnabled);
}

int getVersion(char *cmd) {
  Serial.print(ARDUINO_TYPE);
  Serial.print(":");
  Serial.print(VERSION);
  Serial.flush(); 
}

//======================================================================================
void RP_Reset() //Shut the RPs down for a specific time
{
  WD_FAIL = true;  //turns off LED_RP_Alive and LED_SU_FAIL on
  SCANNER_SHUTDOWN(); //sets AE_ENABLE off 
  // ControlBox_update_LEDs(); // .. then update LEDs

  digitalWrite(P_RP_Reset,HIGH);
  Serial.println("Red Pitaya Off");
  Serial.println("Red Pitaya reboot in:");
  for(int t=1;t<=RP_DownTime/1000;t++)
  {
    Watchdog.reset();
    delay(1000);
//    digitalWrite(P_LED_SU_fail, (RP_DownTime/1000-t+1)%2 );
    Serial.println(RP_DownTime/1000-t+1);
//    ControlBox_update_LEDs(); //During RP downtime the states of all fail variables are not updated!
  }
  digitalWrite(P_RP_Reset,LOW);
  Serial.println("Red Pitaya restart...");
  resetWatchDog(); //fabi: only for testing LEDs

  //Watchdog test:
  Serial.println("Wait 10s - so that the RPs have time to wake up again.");
  int t_0 = micros();
  int t=0;
  while(t<=RP_WakeUpTime)
  {
    t=micros()-t_0;
    WatchDog();
  }
  
  Serial.println("Red Pitaya restart finished.");
}

//======================================================================================
void SU_Reset()
{
 // TODO 
}
 
void loop()
{
  t0 = micros();

  WatchDog();                 // WatchDog for Red Pitaya Surveilliance
  checkFailState();           // Check for all single fail states and trigger overall fail state
  serialHandler.read();
  Watchdog.reset();           // Arduino internal watchDog reset (Without this reset the Arduino would reboot after a given time)
  
  t1 = micros(); // toDo: sollten t1 und t2 nicht resetett werden, laufen die sonst irgendwann über - long unsigned int.
  tcycle = t1 - t0; 
  digitalWrite(P_RSD_Delta,RSD_Delta);
}































// void testSurveillance()
// {

//   //Function to test all error states and their further processing.
  
//   boolean WD_Test = false;
//   boolean NotAus_Test = false;
//   boolean OverTemp_Test = false;
//   boolean AE1_OverVoltage_Test = false;
//   boolean AE2_OverVoltage_Test = false;
//   boolean AE1_OverLoad_Test = false; 
//   boolean AE2_OverLoad_Test = false;
//   boolean AE1_OverTemp_Test = false; 
//   boolean AE2_OverTemp_Test = false;
//   boolean AEs_Test = false;

//   //No Fail at the beginning:
//   WD_FAIL = false;
//   SEFO_24V = false;
//   OVERTEMP = false;
//   AEs_FAIL = false;
  
//   ///////////////////////////
//   //Test: RedPitaya timeout//
//   ///////////////////////////

//   Serial.println("-------------------------------");
//   Serial.println("--------ARDUINO SU TEST--------");
//   Serial.println("-------------------------------");
//   Serial.println("Test: Watchdog RedPitayas:");
  
//   digitalWrite(P_RP_RSD, HIGH); //Remote shutdown RPs
//   WatchDog();
//   delay(1000);

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   Serial.print("Watchdog fail state: ");
//   Serial.println(WD_FAIL);
  
//   if (WD_FAIL && SURVEILLIANCE)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     WD_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed!");
//     Serial.println(Errormessage, BIN);
//     WD_Test = false;
//   }
//   digitalWrite(P_RP_RSD, LOW);
//   WD_FAIL = false;
//   //Am besten wäre es von den RedPitayas überprüfen zu lassen, ob tatsächlich kein Signal in die Spulen gibt, wenn sie Signal ausgeben. Also mit den Pickup coils, die für die Feldregelung genutzt werden.

//   ////////////////
//   //Test: NotAus//
//   ////////////////
//   //toDo Jede Variable printen checkFailState!!!
//   Serial.println("-------------------------------");
//   Serial.println("Test: NotAus"); //Nochmal schauen, welcher Knopf von Fabi das wirklich ist.
//   Serial.println("Please press emergency stop! You have 5 seconds to do it! Hurry!");
  
//   for (int i=0; i <= 5000; i++)
//   {
//     SEFO_24V = !digitalRead(P_SEFO_24V); //NotAus should remain in its state when the button is pressed once
       
//     if(i % 1000 == 0)
//     {
//         Serial.println(i/1000);
//     }
//     delay(1);
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   if (!SEFO_24V && SURVEILLIANCE)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     NotAus_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed!");
//     Serial.println(Errormessage, BIN);
//     NotAus_Test = false;
//   }
//   SEFO_24V = true;

//   //////////////////
//   //Test: OverTemp//
//   //////////////////

//   Serial.println("-------------------------------");
//   Serial.println("Test: OverTemp");
//   //Schicke an Arduino Temperature Control Box Signal, dass Testmodus läuft. Der schickt mir dann zu hohe Temperaturwerte.
//   checkOvertemp();
//   if ( OVERTEMP && SURVEILLIANCE)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     OverTemp_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed!");
//     Serial.println(Errormessage, BIN);
//     OverTemp_Test = false;
//   }
//   OVERTEMP = false;

//   ////////////////////
//   //Test: ErrorRobot//
//   ////////////////////
  
//   //???

//   ////////////////////
//   //Test: AE-Techron//
//   ////////////////////

//   String A = "";
//   Serial.println("-------------------------------");
//   Serial.println("Test: AE-Techron 1");
//   Serial.println("Connect Dummy AE-Techron to AE1 connector at the IO-Box and cause AE1 OverVoltage. Afterwards: Send any String to proceed.");
  
//   while(A=="") //Wait till someone sends a String
//   {
//     A = Serial.readString();
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   if(AE1_OverVoltage)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     AE1_OverVoltage_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed!");
//     Serial.println(Errormessage, BIN);
//     AE1_OverVoltage_Test = false;
//   }

//   A = "";
//   Serial.println("Connect Dummy AE-Techron to AE1 connector at the IO-Box and cause AE1 OverLoad. Afterwards: Send any String to proceed.");
//   while(A=="")
//   {
//     A = Serial.readString();
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   if(AE1_OverLoad)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     AE1_OverLoad_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed");
//     Serial.println(Errormessage, BIN);
//     AE1_OverLoad_Test = false;
//   }

//   A = "";
//   Serial.println("Connect Dummy AE-Techron to AE1 connector at the IO-Box and cause AE1 OverTemp. Afterwards: Send any String to proceed.");
//   while(A=="")
//   {
//     A = Serial.readString();
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   checkAETechrons();
//   if(AE1_OverTemp)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     AE1_OverTemp_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed");
//     Serial.println(Errormessage, BIN);
//     AE1_OverTemp_Test = false;
//   }

//   A = "";
//   Serial.println("Test: AE-Techron 2");
//   Serial.println("Connect Dummy AE-Techron to AE2 connector at the IO-Box and cause AE2 OverVoltage. Afterwards: Send any String to proceed.");
//   while(A=="")
//   {
//     A = Serial.readString();
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   if(AE2_OverVoltage)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     AE2_OverVoltage_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed");
//     Serial.println(Errormessage, BIN);
//     AE2_OverVoltage_Test = false;
//   }

//   A = "";
//   Serial.println("Connect Dummy AE-Techron to AE2 connector at the IO-Box and cause AE2 OverLoad. Afterwards: Send any String to proceed.");
//   while(A=="")
//   {
//     A = Serial.readString();
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   if(AE2_OverLoad)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     AE2_OverLoad_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed");
//     Serial.println(Errormessage, BIN);
//     AE2_OverLoad_Test = false;
//   }

//   A = "";
//   Serial.println("Connect Dummy AE-Techron to AE2 connector at the IO-Box and cause AE2 OverTemp. Afterwards: Send any String to proceed.");
//   while(A=="")
//   {
//     A = Serial.readString();
//     Watchdog.reset();
//   }

//   //loop:
//   checkOvertemp();  // check if Temperature Limits are crossed
//   WatchDog();     // WatchDog for RedPitaya Surveilliance
//   controlBoxCommunication(); // check for input from ControlBox
//   checkFailState(); //After thate time the watchdog sould get it when RPs are down
//   //serialCommand(); // Check for Serial Commands
//   Watchdog.reset(); // Arduino internal WatchDog!

//   if(AE2_OverTemp)
//   {
//     Serial.println("Test passed!");
//     Serial.println(Errormessage, BIN);
//     AE2_OverTemp_Test = true;
//   }
//   else
//   {
//     Serial.println("Test failed");
//     Serial.println(Errormessage, BIN);
//     AE2_OverTemp_Test = false;
//   }

//   if(AE1_OverVoltage_Test && AE2_OverVoltage_Test && AE1_OverLoad_Test && AE2_OverLoad_Test && AE1_OverTemp_Test && AE2_OverTemp_Test)
//   {
//     Serial.println("AE-Techron Test: Passed!");
//     Serial.println(Errormessage, BIN);
//     AEs_Test = true;
//   }
//   else
//   {
//     Serial.println("AE-Techron Test: Failed!");
//     Serial.println(Errormessage, BIN);
//     AEs_Test = false;
//   }

// Serial.println("-------------------------------");
// Serial.println("-------------------------------");
// if(AEs_Test && OverTemp_Test && NotAus_Test && WD_Test)
// {
//   Serial.println("CONGRATULATIONS, ALL TESTS WERE PASSED WELL! :)");
// }
// else
// {
//   Serial.println("SORRY, THERE ARE SOME BUGS. NOT ALL TESTS WERE PASSED. :(");
// }


  
// }
