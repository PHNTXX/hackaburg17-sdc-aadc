/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe#$  $Date:: 2015-10-01 13:37:19#$ $Rev:: 39633   $
**********************************************************************/

#include <AADC_Com.h>  // UART Protocoll
#include <Servo.h> // Actor Servos
#include <I2Cdev.h> // Gyro
#include <MPU6050_6Axis_MotionApps20.h> // Gyro
#include <Wire.h> // Gyro
#include <arduino_pin_defines.h> // Pins
#include <TimerOne.h> //Uss
#include <sensor_timings.h> // timings
#include <cCubic.h> // Poti Speed Calibration

//For testing and debugging without real sensors, e.g. checking protocol parser or ADTF filters
#undef DEBUG_MODE

const tUInt16 Version = 0x1301; // 1.3.0.0

/*************************** GLOABL VARIABLES ***************************************************************************/
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t mpuPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t mpuFifoCount;     // count of all bytes currently in FIFO
uint8_t mpuFifoBuffer[64]; // FIFO storage buffer
//#define _mpu6050_I2CAddress byte(0x68)
volatile bool gyroDataAvailable = false;
bool gyroIsInitalised = false;
MPU6050 mpu;

// USS
#define TIMER_US 50                                   // 50 uS timer duration 
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile byte ussTriggerstate = 0;                    // State machine variable
volatile bool ussSeperateSensorAreas = true;

volatile tUInt32 echo_startCenter = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endCenter = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationCenter = 0;                      // Duration - difference between end and start

volatile tUInt32 echo_startCenterLeft = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endCenterLeft = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationCenterLeft = 0;                      // Duration - difference between end and start

volatile tUInt32 echo_startCenterRight = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endCenterRight = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationCenterRight = 0;                      // Duration - difference between end and start

volatile tUInt32 echo_startLeft = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endLeft = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationLeft = 0;                      // Duration - difference between end and start
volatile tUInt32 echo_startRight = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endRight = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationRight = 0;                      // Duration - difference between end and start

volatile tUInt32 echo_startSideRight = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endSideRight = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationSideRight = 0;                      // Duration - difference between end and start

volatile tUInt32 echo_startSideLeft = 0;                         // Records start of echo pulse
volatile tUInt32 echo_endSideLeft = 0;                           // Records end of echo pulse
volatile tUInt32 echo_durationSideLeft = 0;                      // Duration - difference between end and start

volatile bool echo_DataAvailableCenter = false;
volatile bool echo_DataAvailableCenterLeft = false;
volatile bool echo_DataAvailableCenterRight = false;
volatile bool echo_DataAvailableLeft = false;
volatile bool echo_DataAvailableRight = false;
volatile bool echo_DataAvailableSideLeft = false;
volatile bool echo_DataAvailableSideRight = false;

//WheelEncoder
volatile tUInt32 wheelEnc_Right_interruptCtr = 0;
volatile tInt8 wheelEnc_RightDir = 0;
volatile tUInt32 wheelEnc_Right_interruptTimestamp = 0;
volatile tUInt32 wheelEnc_Left_interruptCtr = 0;
volatile tUInt32 wheelEnc_Left_interruptTimestamp = 0;
volatile tInt8 wheelEnc_LeftDir = 0;

// TimeStamps
tUInt32 led0LastBlinkTimeStamp = 0;
//tUInt32 lastSteeringValueTimeStamp = 0;
tUInt32 lastWheelEncValueTimeStamp = 0;
tUInt32 lastVoltageTimeStamp = 0;
tUInt32 lastVersionTimeStamp = 0;
tUInt32 lastWatchDogRecvTimeStamp = 0;

// Servo
Servo servoSteering;
Servo servoSpeed;

bool emergencyStopRecv = false;

//func ptr loop for each arduino
void (*myLoop)() = 0;

//Mask contains Lights State
tUInt8 lightMask = 0;

// USB Com Port
AADC_Com com;

/************************** Read Coding Pins to determine Arduino Nr ***************/
// one code base for all arduinos
int arduinoAddress = ARD_ADDRESS_UNKNOWN;
int getArduinoAddress() {
  //Read GPIOs to determine Arduino Adress
  pinMode(PIN_ARD_CODE_1, INPUT);
  pinMode(PIN_ARD_CODE_2, INPUT);

  if (digitalRead(PIN_ARD_CODE_1) == LOW && digitalRead(PIN_ARD_CODE_2) == LOW) {
    //is a arduino sensors front board
    arduinoAddress = ARD_ADDRESS_SENSORSFRONT;
  }
  if (digitalRead(PIN_ARD_CODE_1) == LOW && digitalRead(PIN_ARD_CODE_2) == HIGH) {
    //is a arduino sensors back board
    arduinoAddress = ARD_ADDRESS_SENSORSBACK;
  }
  if (digitalRead(PIN_ARD_CODE_1) == HIGH && digitalRead(PIN_ARD_CODE_2) == LOW) {
    //is a arduino sensors side board
    arduinoAddress = ARD_ADDRESS_SENSORSSIDE;
  }

  if (digitalRead(PIN_ARD_CODE_1) == HIGH && digitalRead(PIN_ARD_CODE_2) == HIGH) {
    //is a arduino actors board
    arduinoAddress = ARD_ADDRESS_ACTORS;
  }

  /*********************************** Debuging and Testing: No real Sensors needed! **********************/
  #ifdef DEBUG_MODE
  arduinoAddress = ARD_ADDRESS_UNKNOWN; //DEBUG Setup & Loop
  #endif

  return arduinoAddress;
}



/*********************************************** SETUPs *************************************/
/** assin setup and loop functions for corresponding Arduino Adresses  **/
/** one code base for all arduinos **/
void setup() {
  arduinoAddress = getArduinoAddress();

  //Open USB Serial Port
  com.open(115200);

  pinMode(PIN_ANA_LED0_BOARD, OUTPUT);
  pinMode(PIN_ANA_LED1_BOARD, OUTPUT);
  // pinMode(PIN_ANA_LED2_BOARD, OUTPUT);


  digitalWrite(PIN_ANA_LED0_BOARD, HIGH);
  digitalWrite(PIN_ANA_LED1_BOARD, LOW);

  //func pointer
  bool (*mySetup)() = 0;

  switch (arduinoAddress) {
    case ARD_ADDRESS_SENSORSFRONT:
      mySetup = setupSensorsFront;
      myLoop = loopSensorsFront;
      break;
    case ARD_ADDRESS_SENSORSBACK:
      mySetup = setupSensorsBack;
      myLoop = loopSensorsBack;
      break;
    case ARD_ADDRESS_SENSORSSIDE:
      mySetup = setupSensorsSide;
      myLoop = loopSensorsSide;
      break;
    case ARD_ADDRESS_ACTORS:
      mySetup = setupActorArduino;
      myLoop = loopActorArduino;
      break;
    default:
      #ifdef DEBUG_MODE
      mySetup = setupDebug;
      myLoop = loopDebug;
      #endif
      break;
  }

  //call specific setup func
  if (mySetup()) {
    //LED indicates sucessfull setup 
    digitalWrite(PIN_ANA_LED1_BOARD, HIGH);
  } else {
    //failed
    digitalWrite(PIN_ANA_LED1_BOARD, LOW);
  }

  //Wait till USB serial port open
  while (!Serial);

  //updates TimeStamps, to current millis
  led0LastBlinkTimeStamp = millis();
  //lastSteeringValueTimeStamp = millis();
  lastWheelEncValueTimeStamp = millis();
  lastVoltageTimeStamp = millis();
  lastWatchDogRecvTimeStamp = millis();
}


//only for debugung and testing
#ifdef DEBUG_MODE
bool setupDebug() {
  //nothing to do yet...
  return true;
}
#endif

//Setup for Sensors Back Board
bool setupSensorsBack() {

  pinMode(PIN_US_CENTER_ECH, INPUT);                            // Echo pins set to input
  pinMode(PIN_US_LEFT_ECH, INPUT);
  pinMode(PIN_US_RIGHT_ECH, INPUT);

  pinMode(PIN_US_CENTER_TRIG, OUTPUT);                            // Trigger pin set to output
  pinMode(PIN_US_LEFT_TRIG, OUTPUT);
  pinMode(PIN_US_RIGHT_TRIG, OUTPUT);

  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( trigger_pulse );                 // Attach interrupt to the timer service routine
  attachInterrupt(PIN_INT_US_CENTER_ECH, &interruptUssCenter, CHANGE);
  attachInterrupt(PIN_INT_US_LEFT_ECH, &interruptUssLeft, CHANGE);
  attachInterrupt(PIN_INT_US_RIGHT_ECH, &interruptUssRight, CHANGE);
  attachInterrupt(PIN_INT_US_CENTER_LEFT_ECH, &interruptUssCenterLeft, RISING);
  attachInterrupt(PIN_INT_US_CENTER_RIGHT_ECH, &interruptUssCenterRight, RISING);

  if (!gyroSetup())return false;

  return true;
}

//Setup for Actors Board
bool setupActorArduino() {
  pinMode(PIN_RELAIS, OUTPUT);

  pinMode(PIN_REMOTE_REC_EN, INPUT );

  pinMode(PIN_DIM_LIGHT, OUTPUT);
  pinMode(PIN_REVERSE_LIGHT, OUTPUT);
  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  pinMode(PIN_TURNSIGNAL_LEFT, OUTPUT);
  pinMode(PIN_TURNSIGNAL_RIGHT, OUTPUT);

  digitalWrite(PIN_DIM_LIGHT, LOW);
  digitalWrite(PIN_REVERSE_LIGHT, LOW);
  digitalWrite(PIN_BRAKE_LIGHT, LOW);
  digitalWrite(PIN_TURNSIGNAL_LEFT, LOW);
  digitalWrite(PIN_TURNSIGNAL_RIGHT, LOW);

  //Check if Radio Controlled Jumper or autonomous jumper is set
  if (digitalRead(PIN_REMOTE_REC_EN)) {
    digitalWrite(PIN_RELAIS, HIGH); 
   // digitalWrite(PIN_DIM_LIGHT, HIGH); 

    lightMask = ID_ARD_ACT_LIGHT_MASK_TURNRIGHT | ID_ARD_ACT_LIGHT_MASK_HEAD | ID_ARD_ACT_LIGHT_MASK_TURNLEFT;
    
    //Remote detected, nothing to do for arduino...
    while (true) { //never returns, please use arduino reset button
      tErrorData error;
      error.ui16ErrorNr = ERROR_REMOTE_DETECTED;
      if (millis() % 1000 == 0) com.sendSensorFrame(ID_ARD_SENS_ERROR, micros(), sizeof(tErrorData), (tUInt8*) &error);
      com.clearRecvBuffer();   //Fix ADTF: Workaround for Blocking cSerialDevice.write()
      cycleArduinoLight();
    }
  } else {
    // autonomus mode detected, attach servos to pin
    digitalWrite(PIN_RELAIS, LOW);

    servoSteering.attach( PIN_STEER_SERV );
    servoSpeed.attach( PIN_SPEED_CONTR );

    //set neutral servo position
    servoSteering.write( 90 );
    servoSpeed.write(90);
  }

  return true;

}

//Setup for Sensors Side
bool setupSensorsSide() {
  pinMode(PIN_WHEEL_RIGHT_DIR, INPUT);
  pinMode(PIN_WHEEL_LEFT_DIR, INPUT);
  pinMode(PIN_WHEEL_RIGHT_TACH, INPUT);
  pinMode(PIN_WHEEL_LEFT_TACH, INPUT);

  pinMode(PIN_US_SIDE_RIGHT_ECH, INPUT);                            // Echo pins set to input
  pinMode(PIN_US_SIDE_LEFT_ECH, INPUT);

  pinMode(PIN_US_SIDE_RIGHT_TRIG, OUTPUT);                            // Trigger pin set to output
  pinMode(PIN_US_CENTER_LEFT_TRIG, OUTPUT);

  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( trigger_pulse );                 // Attach interrupt to the timer service routine
  attachInterrupt(PIN_INT_US_SIDE_LEFT_ECH, &interruptUssSideLeft, CHANGE);
  attachInterrupt(PIN_INT_US_SIDE_RIGHT_ECH, &interruptUssSideRight, CHANGE);

  attachInterrupt(PIN_INT_WHEEL_LEFT_TACH, &interuptWheelLeft, RISING);
  attachInterrupt(PIN_INT_WHEEL_RIGHT_TACH, &interuptWheelRight, RISING);

  return true;
}


//Setup for Sensors Front
bool setupSensorsFront() {
  pinMode(PIN_US_CENTER_ECH, INPUT);                            // Echo pins set to input
  pinMode(PIN_US_CENTER_LEFT_ECH, INPUT);
  pinMode(PIN_US_CENTER_RIGHT_ECH, INPUT);
  pinMode(PIN_US_LEFT_ECH, INPUT);
  pinMode(PIN_US_RIGHT_ECH, INPUT);

  pinMode(PIN_US_CENTER_TRIG, OUTPUT);                            // Trigger pin set to output
  pinMode(PIN_US_CENTER_LEFT_TRIG, OUTPUT);
  pinMode(PIN_US_CENTER_RIGHT_TRIG, OUTPUT);
  pinMode(PIN_US_LEFT_TRIG, OUTPUT);
  pinMode(PIN_US_RIGHT_TRIG, OUTPUT);

  Timer1.initialize(TIMER_US);                        // Initialise timer 1 for USS trigger pulse
  Timer1.attachInterrupt( trigger_pulse );            // Attach interrupt to the timer service routine
  attachInterrupt(PIN_INT_US_CENTER_ECH, &interruptUssCenter, CHANGE);
  attachInterrupt(PIN_INT_US_CENTER_LEFT_ECH, &interruptUssCenterLeft, CHANGE);
  attachInterrupt(PIN_INT_US_CENTER_RIGHT_ECH, &interruptUssCenterRight, CHANGE);
  attachInterrupt(PIN_INT_US_LEFT_ECH, &interruptUssLeft, CHANGE);
  attachInterrupt(PIN_INT_US_RIGHT_ECH, &interruptUssRight, CHANGE);

  return true;
}

/********************************************** LOOPS *******************************************/
//
void loop() {
  led0Blink();
  loopVersion();
  myLoop(); //call func pointer, please see setup()
}


//Only for Debuging and Testing
//generate some data
#ifdef DEBUG_MODE
void loopDebug() {

  static tUInt32 lastUssTimeStamp = millis();
  static tUInt32 lastGyroTimeStamp = millis();

  if ( (millis() - lastWheelEncValueTimeStamp) >= (1000 / WheelEncSamplingRate)) {
    tSensWheelData frame;

    frame.ui32WheelTach = 20000;
    frame.i8WheelDir = -1;
    com.sendSensorFrame(ID_ARD_SENS_WHEEL_RIGHT, micros(), sizeof(tSensWheelData), (tUInt8*)&frame);
    com.sendSensorFrame(ID_ARD_SENS_WHEEL_LEFT, micros(), sizeof(tSensWheelData), (tUInt8*)&frame);

    lastWheelEncValueTimeStamp += 1000 / WheelEncSamplingRate;
  }


  if ( (millis() - lastVoltageTimeStamp) >= (1000 / VoltageSamplingRate)) {
    tVoltageData frame;

    frame.ui16VoltageData = 600;
    com.sendSensorFrame(ID_ARD_SENS_VOLT_SPEEDCONT, micros(), sizeof(tVoltageData), (tUInt8*)&frame);
    com.sendSensorFrame(ID_ARD_SENS_VOLT_MEAS, micros(), sizeof(tVoltageData), (tUInt8*)&frame);

    lastVoltageTimeStamp += 1000 / VoltageSamplingRate;
  }


  if ( (millis() - lastUssTimeStamp) >= (1000 / 40)) {
    tUsData frame;

    frame.ui16UsData = 100;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_LEFT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 110;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_CENTER_LEFT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 120;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_CENTER_RIGHT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 130;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_RIGHT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 140;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_CENTER, micros(), sizeof(tUsData), (tUInt8*)&frame);

    frame.ui16UsData = 150;
    com.sendSensorFrame(ID_ARD_SENS_US_SIDE_LEFT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 160;
    com.sendSensorFrame(ID_ARD_SENS_US_SIDE_RIGHT, micros(), sizeof(tUsData), (tUInt8*)&frame);


    frame.ui16UsData = 170;
    com.sendSensorFrame(ID_ARD_SENS_US_BACK_LEFT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 180;
    com.sendSensorFrame(ID_ARD_SENS_US_BACK_RIGHT, micros(), sizeof(tUsData), (tUInt8*)&frame);
    frame.ui16UsData = 190;
    com.sendSensorFrame(ID_ARD_SENS_US_BACK_CENTER, micros(), sizeof(tUsData), (tUInt8*)&frame);



    lastUssTimeStamp += 1000 / 40;
  }


  if ( (millis() - lastGyroTimeStamp) >= (1000 / 50)) {
    tImuData frame;

    frame.i16A_x = 1;
    frame.i16A_y = 2;
    frame.i16A_z = 3;
    frame.i16Q_w = 4;
    frame.i16Q_x = 5;
    frame.i16Q_y = 6;
    frame.i16Q_z = 7;
    com.sendSensorFrame(ID_ARD_SENS_IMU, micros(), sizeof(tImuData), (tUInt8*)&frame);


    lastGyroTimeStamp += 1000 / 50;
  }

  //Fix ADTF: Workaround for Blocking cSerialDevice.write()
  com.clearRecvBuffer();
}
#endif


void loopSensorsBack() {

  // no more Interupt PIN available -> we have to poll i2c
  pollGyro();
  if (gyroDataAvailable) cycleGyro(); // get gyro data

  cycleArduinoUssBack(); // get USS data
  
  //Fix ADTF: Workaround for Blocking cSerialDevice.write()
  com.clearRecvBuffer();
}


void loopActorArduino() {

  //very important
  checkActuatorWatchDog();

  tArduinoActorFrame frame;

  //try to get data from ADTF
  if (com.receiveActorFrame(frame)) {

    switch (frame.sActorHeader.ui8ID) {
      case ID_ARD_ACT_WATCHDOG:
        lastWatchDogRecvTimeStamp = millis();
        break;
      case ID_ARD_ACT_EMERGENCY_STOP:
        emergencyStopRecv = true;
        digitalWrite(PIN_RELAIS, LOW);
        break;
      case ID_ARD_ACT_STEER_SERVO:
        //servoSteering.write(frame.sData);
        servoSteering.write(potiSteering(frame.sData)); //write calibrated data to servo
        break;
      case ID_ARD_ACT_SPEED_CONTR:
        //servoSpeed.write(frame.sData);
        servoSpeed.write(potiSpeed(frame.sData)); //write calibrated data to servo
        break;
      case ID_ARD_ACT_LIGHT:
        lightMask = frame.sData;
        break;
    };
  } else {
    cycleArduinoLight();
  }
}

// Reads neutral steering neutral position from analog poti
tUInt8 potiSteering(tUInt8 angle) {
  int potiSteerOffset = (analogRead(PIN_ANA_POTI_STEER_SERVO) - 512) / 20; //approx. [-25, +25]
  return angle + potiSteerOffset;
}

// Reads acceleration maping from analog poti
tUInt8 potiSpeed(tUInt8 angle) {
  /**
 * PotiState is mapped to an cubic graph
 * PotiState 0 = steep graph, fast acclereation
 * PotiState 1023 = flat graph, slow acceleration
 */

  int currentPotiState = analogRead(PIN_ANA_POTI_SPEED_CONTR);


  //map the PotiState to a new range
  //this range is taken from experience
  float normalizedPotiState = (float)currentPotiState / 1024.f;
  float normValue = (normalizedPotiState * 45) + 15;


  //these are new support points for the cubic graph
  float xValues[5] = {0, 60, 90, 120, 180};
  float yValues[5] = {0, 90 - normValue, 90, 90 + normValue, 180};

  //calculate the graph
  Cubic cubic(5, xValues, yValues);

  //caculate the y and return it
  float calcValue = cubic.getValue(angle);

  //check boarders
  if (calcValue < 0.0) calcValue = 0.0;
  if (calcValue > 180.0) calcValue = 180.0;

  return (tUInt8) calcValue;
}

void loopSensorsFront() {
  cycleArduinoUssFront();
  //Fix ADTF: Workaround for Blocking cSerialDevice.write()
  com.clearRecvBuffer();
}

void loopSensorsSide() {
  cycleArduinoUssSide();
  
  /** depricated, we've got digital servos
  if (millis() - lastSteeringValueTimeStamp >= (1000 / SteeringSamplingRate)) {
    // Read and Transmit Steering Data
    lastSteeringValueTimeStamp += 1000 / SteeringSamplingRate;
    tSensSteeringData frame;
    //TODO: on layout is connected to D5, should be to D4 or D6 (A6 or A7)
    //frame.ui16SteeringSens = analogRead(PIN_ANA_STEER_SENS);
    com.sendSensorFrame(ID_ARD_SENS_STEERING, micros(), sizeof(tSensSteeringData), (tUInt8*)&frame);
  }
  **/

  if (millis() - lastWheelEncValueTimeStamp >= (1000 / WheelEncSamplingRate)) {
    // Read and Transmit WheelEnc Data
    lastWheelEncValueTimeStamp += 1000 / WheelEncSamplingRate;
    tSensWheelData frame;
    frame.ui32WheelTach =   wheelEnc_Left_interruptCtr;
    frame.i8WheelDir = wheelEnc_LeftDir;
    com.sendSensorFrame(ID_ARD_SENS_WHEEL_LEFT,   wheelEnc_Left_interruptTimestamp , sizeof(tSensWheelData), (tUInt8*)&frame);
    frame.ui32WheelTach = wheelEnc_Right_interruptCtr;
    frame.i8WheelDir = wheelEnc_RightDir;
    com.sendSensorFrame(ID_ARD_SENS_WHEEL_RIGHT, wheelEnc_Right_interruptTimestamp, sizeof(tSensWheelData), (tUInt8*)&frame);
  }


  if (millis() - lastVoltageTimeStamp >= (1000 / VoltageSamplingRate)) {
    // Read and Transmit Voltage Data
    tVoltageData frame;
    frame.ui16VoltageData = analogRead(PIN_ANA_VOLT_MEAS);
    com.sendSensorFrame(ID_ARD_SENS_VOLT_MEAS, micros(), sizeof(tVoltageData), (tUInt8*)&frame);
    frame.ui16VoltageData = analogRead(PIN_ANA_VOLT_SPEEDCTRL);
    com.sendSensorFrame(ID_ARD_SENS_VOLT_SPEEDCONT, micros(), sizeof(tVoltageData), (tUInt8*)&frame);

    lastVoltageTimeStamp += 1000 / VoltageSamplingRate;
  }

  //Fix ADTF: Workaround for Blocking cSerialDevice.write()
  com.clearRecvBuffer();
}

/******************************************** LED *************************/
void led0Blink() {
  long int currentTimeStamp = millis();
  if (currentTimeStamp - led0LastBlinkTimeStamp > (1000 / led0SamplingRate) / 2) {
    digitalWrite(PIN_ANA_LED0_BOARD, HIGH);
  } else {
    digitalWrite(PIN_ANA_LED0_BOARD, LOW);
  }

  if (currentTimeStamp - led0LastBlinkTimeStamp > (1000 / led0SamplingRate)) {
    led0LastBlinkTimeStamp += (1000 / led0SamplingRate);
  }
}

/************************************** Version ********************************/
// transmit sw version and arduino adress to ADTF
void loopVersion() {
  long int currentTimeStamp = millis();
  if (currentTimeStamp - lastVersionTimeStamp > (1000 / VersionSamplingRate)) {
    lastVersionTimeStamp = currentTimeStamp;
    tInfoData frame;
    frame.ui8ArduinoAddress = arduinoAddress;
    frame.ui16ArduinoVersion = Version;
    com.sendSensorFrame(ID_ARD_SENSOR_INFO, micros(), sizeof(tInfoData), (tUInt8*)&frame);
  }

}

/*************************************** Light ******************************************/
// cars 
void cycleArduinoLight() {

  long int currentTimeStamp = millis();
  digitalWrite(PIN_DIM_LIGHT, lightMask & ID_ARD_ACT_LIGHT_MASK_HEAD);
  digitalWrite(PIN_REVERSE_LIGHT, lightMask & ID_ARD_ACT_LIGHT_MASK_REVERSE);
  digitalWrite(PIN_BRAKE_LIGHT, lightMask & ID_ARD_ACT_LIGHT_MASK_BRAKE);

  if (currentTimeStamp % (2 * (1000 / LightSamplingRate)) >= (1000 / LightSamplingRate)) {
    digitalWrite(PIN_TURNSIGNAL_RIGHT, lightMask & ID_ARD_ACT_LIGHT_MASK_TURNRIGHT);
    digitalWrite(PIN_TURNSIGNAL_LEFT, lightMask & ID_ARD_ACT_LIGHT_MASK_TURNLEFT);
  } else {
    digitalWrite(PIN_TURNSIGNAL_RIGHT, LOW);
    digitalWrite(PIN_TURNSIGNAL_LEFT, LOW);
  }
}

/**********************************************Watchdog ***********************************/
void checkActuatorWatchDog() {
  if (millis() - lastWatchDogRecvTimeStamp < (1000 / WatchDogSamplingRate) && !emergencyStopRecv) {
    digitalWrite(PIN_RELAIS, HIGH);
  } else {
    // no watchdog trigger recv -> switch of engine
    digitalWrite(PIN_RELAIS, LOW);
  }
}

/************************************************* USS ***************************************************************/
void cycleArduinoUssBack() {
  if (echo_DataAvailableCenter) {
    echo_DataAvailableCenter = false;
    tUsData frame;
    frame.ui16UsData = echo_durationCenter / 58; // Formula: uS / 58 = centimeters, please see Datasheet hc-sr04
    com.sendSensorFrame(ID_ARD_SENS_US_BACK_CENTER, echo_startCenter, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableLeft) {
    echo_DataAvailableLeft = false;
    tUsData frame;
    frame.ui16UsData = echo_durationLeft / 58;
    // change left <-> right, because of symetric board layout
    com.sendSensorFrame(ID_ARD_SENS_US_BACK_RIGHT, echo_startLeft, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableRight) {
    echo_DataAvailableRight = false;
    tUsData frame;
    frame.ui16UsData = echo_durationRight / 58;
    // change left <-> right, because of symetric board layout
    com.sendSensorFrame(ID_ARD_SENS_US_BACK_LEFT, echo_startRight, sizeof(tUsData), (tUInt8*)&frame);
  };
}

void cycleArduinoUssFront() {
  if (echo_DataAvailableCenter) {
    echo_DataAvailableCenter = false;
    tUsData frame;
    frame.ui16UsData = echo_durationCenter / 58;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_CENTER, echo_startCenter, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableLeft) {
    echo_DataAvailableLeft = false;
    tUsData frame;
    frame.ui16UsData = echo_durationLeft / 58;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_LEFT, echo_startLeft, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableRight) {
    echo_DataAvailableRight = false;
    tUsData frame;
    frame.ui16UsData = echo_durationRight / 58;
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_RIGHT, echo_startRight, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableCenterLeft) {
    echo_DataAvailableCenterLeft = false;
    tUsData frame;
    frame.ui16UsData = echo_durationCenterLeft / 58;
    // change left <-> right, because of symetric board layout
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_CENTER_LEFT, echo_startCenterLeft, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableCenterRight) {
    echo_DataAvailableCenterRight = false;
    tUsData frame;
    frame.ui16UsData = echo_durationCenterRight / 58;
    // change left <-> right, because of symetric board layout
    com.sendSensorFrame(ID_ARD_SENS_US_FRONT_CENTER_RIGHT, echo_startCenterRight, sizeof(tUsData), (tUInt8*)&frame);
  };
}

void cycleArduinoUssSide() {
  if (echo_DataAvailableSideLeft) {
    echo_DataAvailableSideLeft = false;
    tUsData frame;
    frame.ui16UsData = echo_durationSideLeft / 58;
    com.sendSensorFrame(ID_ARD_SENS_US_SIDE_LEFT, echo_startSideLeft, sizeof(tUsData), (tUInt8*)&frame);
  };

  if (echo_DataAvailableSideRight) {
    echo_DataAvailableSideRight = false;
    tUsData frame;
    frame.ui16UsData = echo_durationSideRight / 58;
    com.sendSensorFrame(ID_ARD_SENS_US_SIDE_RIGHT, echo_startSideRight, sizeof(tUsData), (tUInt8*)&frame);
  };
}

//Interupts functions for USS ECHO, called if echo recveived
void interruptUssCenter()
{
  switch (digitalRead(PIN_US_CENTER_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endCenter = 0;                                 // Clear the end time
      echo_durationCenter = 0;
      echo_startCenter = micros();                        // Save the start time
      echo_DataAvailableCenter = false;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endCenter = micros();                          // Save the end time
      echo_durationCenter = echo_endCenter - echo_startCenter;        // Calculate the pulse duration
      echo_DataAvailableCenter = true;
      break;
  }
}

void interruptUssCenterLeft()
{
  switch (digitalRead(PIN_US_CENTER_LEFT_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endCenterLeft = 0;                                 // Clear the end time
      echo_startCenterLeft = micros();                        // Save the start time
      echo_DataAvailableCenterLeft = false;
      echo_durationCenterLeft = 0;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endCenterLeft = micros();                          // Save the end time
      echo_durationCenterLeft = echo_endCenterLeft - echo_startCenterLeft;        // Calculate the pulse duration
      echo_DataAvailableCenterLeft = true;
      break;
  }
}

void interruptUssCenterRight()
{
  switch (digitalRead(PIN_US_CENTER_RIGHT_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endCenterRight = 0;                                 // Clear the end time
      echo_startCenterRight = micros();                        // Save the start time
      echo_DataAvailableCenterRight = false;
      echo_durationCenterRight = 0;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endCenterRight = micros();                          // Save the end time
      echo_durationCenterRight = echo_endCenterRight - echo_startCenterRight;        // Calculate the pulse duration
      echo_DataAvailableCenterRight = true;
      break;
  }
}

void interruptUssLeft()
{
  switch (digitalRead(PIN_US_LEFT_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endLeft = 0;                                 // Clear the end time
      echo_startLeft = micros();                        // Save the start time
      echo_DataAvailableLeft = false;
      echo_durationLeft = 0;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endLeft = micros();                          // Save the end time
      echo_durationLeft = echo_endLeft - echo_startLeft;        // Calculate the pulse duration
      echo_DataAvailableLeft = true;
      break;
  }

}

void interruptUssRight()
{

  switch (digitalRead(PIN_US_RIGHT_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endRight = 0;                                 // Clear the end time
      echo_startRight = micros();                        // Save the start time
      echo_DataAvailableRight = false;
      echo_durationRight = 0;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endRight = micros();                          // Save the end time
      echo_durationRight = echo_endRight - echo_startRight;        // Calculate the pulse duration
      echo_DataAvailableRight = true;
      break;
  }

}
void interruptUssSideLeft()
{
  switch (digitalRead(PIN_US_SIDE_LEFT_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endSideLeft = 0;                                 // Clear the end time
      echo_startSideLeft = micros();                        // Save the start time
      echo_DataAvailableSideLeft = false;
      echo_durationSideLeft = 0;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endSideLeft = micros();                          // Save the end time
      echo_durationSideLeft = echo_endSideLeft - echo_startSideLeft;        // Calculate the pulse duration
      echo_DataAvailableSideLeft = true;
      break;
  }

}

void interruptUssSideRight()
{

  switch (digitalRead(PIN_US_SIDE_RIGHT_ECH))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endSideRight = 0;                                 // Clear the end time
      echo_startSideRight = micros();                        // Save the start time
      echo_DataAvailableSideRight = false;
      echo_durationSideRight = 0;
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endSideRight = micros();                          // Save the end time
      echo_durationSideRight = echo_endSideRight - echo_startSideRight;        // Calculate the pulse duration
      echo_DataAvailableSideRight = true;
      break;
  }

}
// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{

  if (!(--trigger_time_count))                   // Count to 200mS
  { // Time out - Initiate trigger pulse
    trigger_time_count =   (1000000 / UssSamplingRate) / TIMER_US;       // trigger_time_count = 500 @ 40 Hz
    ussTriggerstate = 1;                                  // Changing to state 1 initiates a pulse
  }

  switch (ussTriggerstate)                                 // State machine handles delivery of trigger pulse
  {
    case ARD_ADDRESS_UNKNOWN:                                      // Normal state does nothing
      break;

    case 1:                                      // Initiate pulse
      if (arduinoAddress == ARD_ADDRESS_SENSORSFRONT) {
        if (ussSeperateSensorAreas) {  //Seperate Trigger because of sensor areas overlappig. -> only 20 HZ
          digitalWrite(PIN_US_CENTER_LEFT_TRIG, HIGH);              // Set the trigger output high
          digitalWrite(PIN_US_CENTER_RIGHT_TRIG, HIGH);              // Set the trigger output high
          ussSeperateSensorAreas = false;
        } else {
          digitalWrite(PIN_US_CENTER_TRIG, HIGH);              // Set the trigger output high
          digitalWrite(PIN_US_RIGHT_TRIG, HIGH);              // Set the trigger output high
          digitalWrite(PIN_US_LEFT_TRIG, HIGH);              // Set the trigger output high
          ussSeperateSensorAreas = true;
        }
      } else if ( arduinoAddress == ARD_ADDRESS_SENSORSBACK) {
        digitalWrite(PIN_US_CENTER_TRIG, HIGH);              // Set the trigger output high
        digitalWrite(PIN_US_LEFT_TRIG, HIGH);              // Set the trigger output high
        digitalWrite(PIN_US_RIGHT_TRIG, HIGH);              // Set the trigger output high
      } else if (arduinoAddress == ARD_ADDRESS_SENSORSSIDE) {
        digitalWrite(PIN_US_SIDE_LEFT_TRIG, HIGH);              // Set the trigger output high
        digitalWrite(PIN_US_SIDE_RIGHT_TRIG, HIGH);              // Set the trigger output high
      }
      ussTriggerstate = 2;                                // and set state to 2
      break;

    case 2:                                      // Complete the pulse
    default:
      if (arduinoAddress == ARD_ADDRESS_SENSORSFRONT) {
        digitalWrite(PIN_US_CENTER_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_CENTER_LEFT_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_CENTER_RIGHT_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_RIGHT_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_LEFT_TRIG, LOW);              // Set the trigger output high
      } else if ( arduinoAddress == ARD_ADDRESS_SENSORSBACK) {
        digitalWrite(PIN_US_CENTER_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_LEFT_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_RIGHT_TRIG, LOW);              // Set the trigger output high
      } else if (arduinoAddress == ARD_ADDRESS_SENSORSSIDE) {
        digitalWrite(PIN_US_SIDE_LEFT_TRIG, LOW);              // Set the trigger output high
        digitalWrite(PIN_US_SIDE_RIGHT_TRIG, LOW);              // Set the trigger output high
      }
      ussTriggerstate = 0;                                // and return state to normal 0
      break;
  }
}

/********************************************************** Gyro *****************************************************************/
/** No interupt pin available
void interuptGyro() {
  gyroDataAvailable = true;
}
**/

void pollGyro() {

  if (!gyroIsInitalised) {
    tErrorData error;
    error.ui16ErrorNr = ERROR_NO_GYRO_DETECTED;
    // Send Error Message
    if (millis() % 101 == 0) com.sendSensorFrame(ID_ARD_SENS_ERROR, micros(), sizeof(tErrorData), (tUInt8*) &error);
    return;
  }

  mpuIntStatus = mpu.getIntStatus();

  if ((mpuIntStatus & 0x10 /**FIFO overflow Bit **/))
  {
    gyroDataAvailable = false;
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02 /** DMP Ready Bit **/)
  {
    gyroDataAvailable = true;
  }
}

void cycleGyro() {
  tImuData frame;

   // get current FIFO count
  mpuFifoCount = mpu.getFIFOCount();

  //check for FIFO overlow
  if(mpuFifoCount == 1024){
       gyroDataAvailable = false;
       // reset so we can continue cleanly
       mpu.resetFIFO();
       return;
  }

 //check fifo count and pop data
  while (mpuFifoCount >= mpuPacketSize)
  {
     //pop data from FIFO
      mpu.getFIFOBytes(mpuFifoBuffer, mpuPacketSize);
      mpuFifoCount -= mpuPacketSize;
      //copy data to frame
      mpu.dmpGetQuaternion((int16_t *) & (frame.i16Q_w), mpuFifoBuffer);
      mpu.dmpGetAccel((int16_t *) & (frame.i16A_x), mpuFifoBuffer);
      //transmit data
      com.sendSensorFrame( ID_ARD_SENS_IMU, micros(), sizeof(tImuData), (tUInt8 *) &frame );
  }

  gyroDataAvailable = false;
}

bool gyroSetup()
{
  Wire.begin();
  //depricated: use slow i2c because of better reliability with long cables
  //Wire.setClock(50 * 1000);

  pinMode(PIN_GYRO_VSS_ENABLE, OUTPUT);
  //Power Off
  digitalWrite(PIN_GYRO_VSS_ENABLE, LOW);
  delay(500);
  digitalWrite(PIN_GYRO_VSS_ENABLE, HIGH);
  delay(200);


  //Gyro
  uint8_t devStatus;
  mpu.initialize();
  if (!mpu.testConnection()) return false;
  devStatus =  mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    //attachInterrupt(PIN_INT_GYRO, interuptGyro, RISING); // no more interups pins available
    mpuIntStatus = mpu.getIntStatus();
    mpuPacketSize = mpu.dmpGetFIFOPacketSize();
    gyroIsInitalised = true;
    return true;
  }
  else
    return false;
}

/******************************************************** Wheel Enc *************************************************************/
// wheel encoder disc trigger interupts
void interuptWheelLeft() {
  wheelEnc_LeftDir = digitalRead(PIN_WHEEL_LEFT_DIR);
  wheelEnc_Left_interruptCtr++; // count nr of ticks, overflow at > 40 000 km
  wheelEnc_Left_interruptTimestamp = micros();
}

void interuptWheelRight() {
  //has to be inverted because sensor is mounted the other way round
  wheelEnc_RightDir = !(digitalRead(PIN_WHEEL_RIGHT_DIR));
  wheelEnc_Right_interruptCtr++; // count nr of ticks, overflow at > 40 000 km
  wheelEnc_Right_interruptTimestamp = micros();
}

/*** EOF ***/




