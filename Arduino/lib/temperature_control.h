#ifndef TemperatureControl_h
#define TemperatureControl_h

#include "Arduino.h"

#define TempSensorVoltageSource A0

#define EMRDelayShort 10 //ms max.
#define SSRDelayOpen 5//ms max.

#define UDC 5.0
#define R3K 3000
#define row 2
#define maxSamplesNum 111

//LUT
#define ohm 0
#define temperature 1 //obsolete?

#define hysteresisWidth 10 //degreess Celsius

//storage
#define memoryLength 64
#define memoryMask 64 - 1

//PI varivables: obsolete, here
// #define kP 1
// #define kI 1
// #define sampleTimePI 0.200 //seconds
// #define maxIntegralControl 1
// #define minIntegralControl 0
// #define maxControl 1
// #define minControl 0

//VQRIABLKE DUTY CYCLE
#define deltaDutyCycle 0.1 


//controlPhase Variables
#define percentSetTemp 0.9


enum ControlMode {THRESHOLD, DUTYCYCLE};



static int Table[row][maxSamplesNum] = {
  // Resistance 0
  { //gerundet
    19903, 18973, 18092, 17257, 16465, 15714,
    15001, 14324, 13682, 13072, 12493, 11943,
    11420, 10922, 10449, 10000, 9572, 9164, 8776, 8407,
    8056, 7720, 7401, 7097, 6806, 6530, 6266,
    6014, 5773, 5544, 5325, 5116, 4916,
    4724, 4542, 4367, 4200, 4040, 3887,
    3741, 3601, 3467, 3339, 3216, 3098, 2985, 2877, 2773,
    2674, 2579, 2487, 2399, 2315, 2234,
    2157, 2082, 2011, 1942, 1876, 1813,
    1752, 1693, 1637, 1582, 1530, 1480, 1432, 1386,
    1341, 1298, 1257, 1217, 1178, 1141, 1106, 1071,
    1038, 1006, 975, 945, 917, 889, 862, 836,
    811, 787, 764, 742, 720, 699,
    679, 659, 640, 622, 604, 587,
    571, 555, 539, 524, 510, 496,
    482, 469, 457, 444, 433, 421,
    410, 399, 389 } 
 
  ,

  // Temperature 1
  {
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120 }
};

class HeatUnit
{
  public:
    HeatUnit(int measurementPin, int pinSSR, int pinEMR, int setTemperature, int maxTemperature, bool EMR_Enable);
    
    void temperatureMeasurement();

    void control();
    void controlThreshold();
    void controlDutyCycle(); 
        
    void optimizeDutyCycle(int memIndex);
    float movingAverage(int numAverages);

    void output();
    void disableOutput();
    void setOutput1(); //one and only, that acts on outputs
    void setOutput2();
    //void setOutputAC_Enabled();
    float getTargetTemp();
    float getMaxTemp();
    ControlMode getControlMode();

    //void controlPhase();
    //void phaseOne();
    //void phaseTwo();
    //void phaseThree();
 

    bool setControlMode(ControlMode mode);
    bool setTargetTemp(float target);
    bool setMaxTemp(float max);
        
    bool getOutput();
    float getDutyCycle();
    float getAvgTemp();
    float getTemp();
    float getPeriod();
    int setNewDutyCycle(float newDutyCycle);

    int checkForError();
    
    
  
  private:
    void enableTemperatureMeasurement();
    void disableTemperatureMeasurement();
    int updateTemperature();
    void updateTemperatureMemory(float temp);
    float getTemperatureLUT (int resistance);
    int checkState(int error);
    
    
    
    int _measurementPin;
    int _pinSSR;
    int _pinEMR;
    float _temperature = 25.0;
    

    float _targetTemperature;
    float _maxTemperature;
    bool _EMR_Enable;

    ControlMode _controlMode = DUTYCYCLE; // standard: 1: Threshold 2: PID /dutycycle
    
    //Output
    int _period = 1000;
    float _dutyCycle = 0.5;
    bool _stateOutput = false;
    bool _overshootDetected = false;
    float _phaseOneTargetTemperature = 0.0;

    void _setOutputDutycycle();

    //Memory
    float _temperatureMemory[memoryLength];
    unsigned int _write = 0;
    float _averageTemperature;
    float _previousAverageTemperature = 0.0;
    float _trendTemperature;

    //ERROR
    int _measurementError = 3; //init with overtemp, let arduino loop reset it
};


HeatUnit::HeatUnit(int measurementPin, int pinSSR, int pinEMR, int setTemperature, int maxTemperature, bool EMR_Enable) {
  //Init Information
  _measurementPin = measurementPin;
  _pinSSR = pinSSR;
  _pinEMR = pinEMR;
  _targetTemperature = setTemperature;
  _maxTemperature = maxTemperature;
  _EMR_Enable = EMR_Enable;
  _write = 0;

  //Memory Variables
  _temperatureMemory[memoryLength];

  // _stateOutput init with 0, then disable to begin with
  disableOutput();

  // if (_EMR_Enable == false) {
  //   digitalWrite(_pinEMR, HIGH);
  // } else {
  //   digitalWrite(_pinEMR, LOW);
  // }
  // digitalWrite(_pinSSR, HIGH); // disable to begin with

  //Serial.print("Initialised Unit:[");Serial.print(_pinSSR-1);Serial.print("]  Set Temperature to ");
  //Serial.print(_targetTemperature);Serial.println("°C");
}

//####################################################################################

int HeatUnit::checkForError() { // just return error status, DO NOT TAKE ACTION here
  return _measurementError;
}





//####################################################################################

// TODO get pin via Construktor, nicht aus dem .h file!!!
// void HeatUnit::enableTemperatureMeasurement() {
//   digitalWrite(pinOpto, HIGH); //NPN
// }

// void HeatUnit::disableTemperatureMeasurement() {
//   digitalWrite(pinOpto, LOW);
// }


// TODO: Funktionen die Fehlschlagen können, siehe "Invalid" Prints, können gerne auch bool returnen ob sie geklappt haben oder nichts
// Dann kann man im Programm entsprechenden Code schreiben

int HeatUnit::updateTemperature() {
  float voltageSource = (analogRead(TempSensorVoltageSource) * UDC) / 1023.0;
  float voltageMeasured = (analogRead(_measurementPin) / 1023.0) * voltageSource;
  float voltageSensor = UDC - voltageMeasured;  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  
  if (voltageSensor < 4.6) {
    if (voltageSensor > 0.1) {
      float resistanceSensor = R3K / ((voltageSource / voltageSensor) - 1);
      float tmp = getTemperatureLUT(resistanceSensor);

      //sanity check
      if (tmp > 10.0 && tmp < 130.0) {
        _temperature = tmp;
        return 0;
      } else {
        return 1;
      }
    } 
    else {
      // Sensor Voltage = 0V, check if ground connected
      return 1;
    }
  } 
  // Sensor not connected to Vcc?
  return 2; 
}
// TODO: Bau eine weitere HeatUnit Funktion log(msg) und schreib all deine Prints darein
      // Dann gib der Klasse eine boolean Flag die log, debug oder verbose heißt + setter
      // Nur wenn die Flag true ist schreib die Nachrichten auf den SerialPort.


float HeatUnit::getTemperatureLUT(int resistance) {
  //if (resistance > 6530 || resistance < 1257) {// below 35 degrees and above 80 degrees
  for (int i = 0; i < maxSamplesNum - 1; i++) {
    if (resistance > Table[0][i + 1]) {
      //if (resistance > 1257 && resistance < 6530) {
      float interpolatedTemperature = (float(Table[1][i] - Table[1][i + 1]) / (Table[0][i] - Table[0][i + 1])) * float(resistance - Table[0][i]) + float(Table[1][i]);
      return interpolatedTemperature;
      /*} 
      else {
        return float(Table[1][i]);
      }*/
    }
  }
}

//####################################################################################
void HeatUnit::updateTemperatureMemory(float temp) {
  _temperatureMemory[_write & memoryMask] = temp;
  _write++;
}

float HeatUnit::movingAverage(int numAverages){
  int valid = 0;
  long index = 0;
  float avg = 0.0;
  for (int i = 0; i < numAverages; i++) {
    index = _write - 1 - i;
    // Only average over temps that were actually already written
    // Inconsistent when _write overflows
    if (index >= 0) {
      avg += _temperatureMemory[index & memoryMask];
      valid++;
    } else {
      break;
    }
  }
  if (valid > 0) {
    return avg/valid;
  } else {
    return _temperature;
  }
}


void HeatUnit::temperatureMeasurement() {
  _measurementError = updateTemperature();
  if (_measurementError == 0) {
    updateTemperatureMemory(_temperature);
  }
  // second check for maxTemp (3)
  _measurementError = checkState(_measurementError);
}

int HeatUnit::checkState(int error) {
  if (_temperature >= _maxTemperature) {
    return 3;
  }
  return error;
}

//####################################################################################
// Control functions: set the state for output
// TODO Seperate select from control:
// selectControlMethod(mode): _controlMode = mode
// control(): if (bla) controlBla() ...
// TODO Use enums instead of 1-5
// TODO Use "else if" or switch statement

void HeatUnit::control() {
  if (_controlMode == THRESHOLD) {
    controlThreshold();
  }
  else if (_controlMode == DUTYCYCLE) {
    controlDutyCycle();
  }
  // else if (_controlMode == 3) {
  //   controlPhase();
  // }
}

void HeatUnit::controlThreshold() {
  if (_temperature < _targetTemperature) {
    _stateOutput = true;
  } else {
    _stateOutput = false;
  }
}

void HeatUnit::controlDutyCycle() {
  if (_dutyCycle < 0.007){
    _dutyCycle = 0.0;    
  }
  if (millis() % _period < _period * _dutyCycle) {
    _stateOutput = true;
    //Serial.println("HIGH");
  } else {  //if(millis() % (_period + _period * _dutyCycle / 100) < 40){
    _stateOutput = false;
    //Serial.println("LOW");
  }
}

void HeatUnit::optimizeDutyCycle(int memIndex) {
  int oldTemp = _temperatureMemory[(_write - memIndex) & memoryMask];
  int tempDeltaNow = _targetTemperature - _temperature;
  int tempDeltaPast = _targetTemperature - oldTemp;  //trend

  if (abs(tempDeltaNow) > 2) {  // do not act, if close enough to setTemp

    // above setTemp
    if (tempDeltaNow < 0) {
      if (tempDeltaPast < 0) {  // trend nach unten
        _dutyCycle -= deltaDutyCycle / 2;
      }
      else if (tempDeltaPast > 0) {  //trend hoch
        _dutyCycle -= deltaDutyCycle;
      }
    }
    // below setTemp
    else if (tempDeltaNow > 0) {

      if (tempDeltaPast < 0) {         // trend nach unten
        _dutyCycle += deltaDutyCycle;  //-> muss heizen
      }
      else if (tempDeltaPast > 0) {             //trend hoch
        _dutyCycle += deltaDutyCycle / 2;  //abwarten???
      }
    }

    // other cases: signs changed = zero crossing, do not act (or: implement later in detail)
  }

  if (_dutyCycle <= 0) {
    _dutyCycle = 0;
  }
  else if (_dutyCycle >= 1) {
    _dutyCycle = 1;
  }

  //Serial.print("OptimizedDutyCycle to: ");
  //Serial.println(_dutyCycle);
}





//##################################################################################
// Phasen Kontrolle
//###################################################################################
// void HeatUnit::phaseOne() {
//   _stateOutput = true;
// }
// void HeatUnit::phaseTwo() {
//   _stateOutput = false;

//   // Detect temperatureTrend
//   int oldTemp = _temperatureMemory[(_write - 1) % memoryLength];  // Temperature one second ago
//   int tempDeltaNow = _targetTemperature - _temperature;
//   int tempTrend = _targetTemperature - oldTemp;  //trend
//   bool overshoot = false;

//   if ((overshoot == true) && (tempTrend < 0)) {
//     _overshootDetected = true;
//   }
//   else if (tempTrend > 0) {  //  Temp Rises
//     if (_temperature > (_targetTemperature + 5.0)) {
//       overshoot = true;
//     }
//   }
//   else if (tempTrend < 0) {  // Temp Falls
//     _phaseOneTargetTemperature += 0.05;
//   }

  
// }

// void HeatUnit::phaseThree() {
//   //optimizeDutyCycle(10);
// }

// void HeatUnit::controlPhase() {
//   //Verschiedene Phase:
//   // 1. Aufheizphase --> heizen bis erste AufheizTemperatur
//   // 2. Overshootphase --> Heizen unterbrechen bis Temperatur die Set Temp zuerst überschreitet und anschließend wieder Fällt
//   // 3. Entweder PI oder PWM Control to keep settemperatur
//   _phaseOneTargetTemperature = percentSetTemp * _targetTemperature;
//   // Phase 1:
//   if ((_temperature < _phaseOneTargetTemperature) && (_overshootDetected = false)) {
//     phaseOne();
//   }
//   // Phse 2:
//   if ((_temperature > _phaseOneTargetTemperature) && (_overshootDetected = false)) {
//     phaseTwo();
//   }
//   if ((_temperature > _phaseOneTargetTemperature) && (_overshootDetected = true)) {
//     phaseThree();
//   }
// }


//####################################################################################
// Only here: Outputs are set to pin

// void HeatUnit::setOutputAC_Enabled() {  //Turn OFF Heating
//   digitalWrite(_pinSSR, HIGH);
//   delay(SSRDelayOpen);  // Nur benuitzt da nun ersteinmal nichts passieren soll... AC ENABLE!!!
//   digitalWrite(_pinEMR, HIGH);
// }



void HeatUnit::output() {
  if (_EMR_Enable == true) {
    setOutput2();
  } else {
    setOutput1();
  }
}

void HeatUnit::disableOutput() {
  digitalWrite(_pinSSR, HIGH);
  _stateOutput = false;
  if (_EMR_Enable == true) {
    delay(SSRDelayOpen);  
    digitalWrite(_pinEMR, HIGH);
  }
}

void HeatUnit::setOutput1()  // Ohne sekundärem Relais
{
  digitalWrite(_pinSSR, !_stateOutput);
  //Serial.println("Output:");
  //Serial.println(_stateOutput);
}

void HeatUnit::setOutput2() {  // Mit sekundärem Relais keine Verzögerung nach dem zweiten Schalten!!!
  int Tstart = millis();
  if (_stateOutput == HIGH) {
    digitalWrite(_pinEMR, !_stateOutput);
    //if ((millis() - Tstart )> 10000){
    delay(EMRDelayShort);
    digitalWrite(_pinSSR, !_stateOutput);
    //}
  } else {
    digitalWrite(_pinSSR, !_stateOutput);
    //if ((millis() - Tstart) > SSRDelayOpen){
    delay(SSRDelayOpen);
    digitalWrite(_pinEMR, !_stateOutput);
    //}
  }
}



//
//###############################################
//Communication functions
//read

float HeatUnit::getTemp() {
  return _temperature;
}

float HeatUnit::getTargetTemp() {
  return _targetTemperature;
}

float HeatUnit::getMaxTemp() {
  return _maxTemperature;
}

ControlMode HeatUnit::getControlMode() {
  return _controlMode;
}

// write
bool HeatUnit::setControlMode(ControlMode mode) {
  _controlMode = mode;
  return true;
}


// TODO Boolean return for the following
bool HeatUnit::setTargetTemp(float target) {
  if (target < _maxTemperature) {
    _targetTemperature = target;
    return true;
  }
  return false;
}
bool HeatUnit::setMaxTemp(float max) {
  if ((40 < max) && (max< 100) && (max > _targetTemperature)) {
    _maxTemperature = max;
    return true;
  }
  return false;
}


float HeatUnit::getPeriod() {
  return _period;
}
bool HeatUnit::getOutput(){
  return _stateOutput;
}
float HeatUnit::getDutyCycle(){
  return _dutyCycle;
}
float HeatUnit::getAvgTemp(){
  return movingAverage(4);
}
int HeatUnit::setNewDutyCycle(float newDutyCycle){
  _dutyCycle = newDutyCycle;
}


#endif
