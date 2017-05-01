/*
 * SerialReceive sketch
 * Blink the LED at a rate proportional to the received digit value
*/
#include <AADC_Com.h>  // UART Protocoll
#include <Servo.h> // Actor Servos
#include <arduino_pin_defines.h> // Pins
#include <cCubic.h> // Poti Speed Calibration
#include <sensor_timings.h> // timings

tUInt32 led0LastBlinkTimeStamp = 0;
tUInt32 lastVersionTimeStamp = 0;
tUInt32 lastCurrentTimeStamp = 0;
tUInt32 lastWatchDogRecvTimeStamp =0;

bool emergencyStopRecv =false;
const tUInt16 Version = 0x1301; // 1.3.0.0

// Servo
Servo servoSteering;
Servo servoSpeed;

bool remotedetected=false;
int arduinoAddress = ARD_ADDRESS_ACTORS;
tUInt8 lightMask = 0;

void setup()
{
  Serial.begin(115200); // Initialize serial port to send and receive at 9600 baud
  
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
    remotedetected=true;
    digitalWrite(PIN_RELAIS, HIGH); 
    while(true){
      loopVersion();
    }
  } else {
    // autonomus mode detected, attach servos to pin
    digitalWrite(PIN_RELAIS, HIGH);

    servoSteering.attach( PIN_STEER_SERV );
    servoSpeed.attach( PIN_SPEED_CONTR );

    //set neutral servo position
    servoSteering.write( 90 );
    servoSpeed.write(90);
  }

  return true;
}

tUInt8 current_speed=90;
tUInt8 current_steer=90;
tUInt8 actorcase=0;

int startmillis=0;
float amp=10;
float fre=0.1;
tUInt8 steeringsendvalue=90;
int speedvalue=90;
void loop()
{
    loopVersion();
    loopCurrent();
    current_steer=steeringsendvalue;
    servoSteering.write(potiSteering(steeringsendvalue)); //write calibrated data to servo
    switch(actorcase){
        case 0:{
          current_speed = 90; // actual rate is 100ms times received digit
          servoSpeed.write(potiSpeed(90)); //write calibrated data to servo
          break;      
        }
        case 1:{
          int ti=millis()-startmillis;
          float tisek=(float)ti/1000;
          current_speed = 90 + amp*sin(2*3.1415*fre*tisek); // actual rate is 100ms times received digit
          servoSpeed.write(potiSpeed(current_speed)); //write calibrated data to servo
          break;
        }
        case 2:{
          servoSpeed.write(potiSpeed(speedvalue)); //write calibrated data to servo
          current_speed=speedvalue;
          break;
        }
        case 3:{
          
          servoSpeed.write(potiSpeed(speedvalue)); //write calibrated data to servo
          current_speed=speedvalue;
          break;
        }
        default:{
          current_speed = 90; // actual rate is 100ms times received digit
          servoSpeed.write(potiSpeed(current_speed)); //write calibrated data to servo
          break;
        }
    }
  if ( Serial.available()) // Check to see if at least one character is available
  {
   
    char ch = Serial.read();
    int sw=0;
    if( isDigit(ch) ) // is this an ascii digit between 0 and 9?
    {
       sw=(ch - '0');      // ASCII value converted to numeric value
       if(sw==0){
          speedvalue=90;
          actorcase=0;
          amp=10;
          fre=0.1;
       }
       if(sw==1){
          actorcase=1;
          startmillis=millis();
       }
       if(sw==2){
          amp=amp+2;
       }
       if(sw==3){
          fre=fre*2;
       }
       if(sw==4){
          steeringsendvalue=steeringsendvalue-2;
       }
       if(sw==5){
          steeringsendvalue=90;
       }
       if(sw==6){
          steeringsendvalue=steeringsendvalue+2;
       }
       if(sw==7){
          actorcase=2;
          speedvalue=speedvalue-2;
          current_speed=speedvalue;
       }
       if(sw==8){
          actorcase=3;
          speedvalue=speedvalue+2;
          current_speed=speedvalue;
//          servoSpeed.write(potiSpeed(115));
       }
       if(sw==9){
          
          speedvalue=90;
//          servoSpeed.write(potiSpeed(115));
       }
    }
  }
  
}

tUInt8 potiSteering(tUInt8 angle) {
  int potiSteerOffset = (analogRead(PIN_ANA_POTI_STEER_SERVO) - 512) / 20; //approx. [-25, +25]
  return angle + potiSteerOffset;
}

void loopVersion() {
  long int currentTimeStamp = millis();
  if (currentTimeStamp - lastVersionTimeStamp > (1000 / VersionSamplingRate)) {
    lastVersionTimeStamp = currentTimeStamp;
    tInfoData frame;
    frame.ui8ArduinoAddress = arduinoAddress;
    frame.ui16ArduinoVersion = Version;
    sendSensorFrame(ID_ARD_SENSOR_INFO, micros(), sizeof(tInfoData), (tUInt8*)&frame);
  }

}


void loopCurrent() {
  long int currentTimeStamp = millis();
  if (currentTimeStamp - lastCurrentTimeStamp > (1000 / CurrentSamplingRate)) {
    lastCurrentTimeStamp = currentTimeStamp;
    tCurrData framespeed;
    framespeed.ui8CurrData = current_speed;
    sendSensorFrame(ID_ARD_SENS_CURR_SPEED, micros(), sizeof(tCurrData), (tUInt8*)&framespeed);
    tCurrData framesteer;
    framesteer.ui8CurrData = current_steer;
    sendSensorFrame(ID_ARD_SENS_CURR_STEER, micros(), sizeof(tCurrData), (tUInt8*)&framesteer);
    
  }

}



// Reads acceleration maping from analog poti
tUInt8 potiSpeed(tUInt8 angle) {
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

 int sendSensorFrame(const tUInt8 id, const tUInt32 timestamp, const tUInt8 len,const tUInt8 rawData[] ){
  
  if(len > sizeof(tArduinoSensorFrame) - sizeof(tArduinoSensorHeader) - sizeof(INIT_CRC))return 0;
   tArduinoSensorFrame sensorFrame;
  //tArduinoSensorFrame frame;
  memset(&sensorFrame,0xFF,sizeof(tArduinoSensorFrame));
  
  byte *buffer = (byte*) &sensorFrame;
  
  sensorFrame.sSensorHeader.ui8SOF = ID_ARD_SOF;
  sensorFrame.sSensorHeader.ui8ID = id;
  sensorFrame.sSensorHeader.ui32ArduinoTimestamp = timestamp;
  sensorFrame.sSensorHeader.ui8DataLength = len;
  
  //copy data
 
  memcpy(&sensorFrame.sData,rawData, len);
  
  tUInt16 calcCRC = fletcher16(buffer,sizeof(tArduinoSensorHeader) + len);
  
  byte *crcBuffer = (byte*) &calcCRC;
  
  //copy crc
  memcpy(&buffer[sizeof(tArduinoSensorHeader) + len],crcBuffer, sizeof(calcCRC));
  
  tUInt8 bytesWritten = 0;
  tUInt8 bytesToWrite = sizeof(tArduinoSensorHeader) + len + sizeof(calcCRC);
  
  //write frame to serial
  bytesWritten = Serial.write(&(buffer[bytesWritten]),bytesToWrite);
  
  Serial.flush();
  
  return bytesWritten;
 }

