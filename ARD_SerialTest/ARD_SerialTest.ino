#include <AADC_Com.h>  // UART Protocoll
#include <Servo.h> // Actor Servos
#include <arduino_pin_defines.h> // Pins
#include <cCubic.h> // Poti Speed Calibration
#include <sensor_timings.h> // timings

// constants
const tUInt16 Version = 0x1301; // 1.3.0.0

// servos
Servo servoSteering;
Servo servoSpeed;

// state
tUInt32 led0LastBlinkTimeStamp = 0;
tUInt32 lastVersionTimeStamp = 0;
tUInt32 lastCurrentTimeStamp = 0;
tUInt32 lastWatchDogRecvTimeStamp = 0;

int current_steering = 90;
int current_speed = 90;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_DIM_LIGHT, OUTPUT);
  pinMode(PIN_REVERSE_LIGHT, OUTPUT);
  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  pinMode(PIN_TURNSIGNAL_LEFT, OUTPUT);
  pinMode(PIN_TURNSIGNAL_RIGHT, OUTPUT);

  digitalWrite(PIN_DIM_LIGHT, LOW);
  digitalWrite(PIN_REVERSE_LIGHT, HIGH);
  digitalWrite(PIN_BRAKE_LIGHT, HIGH);
  digitalWrite(PIN_TURNSIGNAL_LEFT, HIGH);
  digitalWrite(PIN_TURNSIGNAL_RIGHT, HIGH);

  servoSteering.attach(PIN_STEER_SERV);
  servoSpeed.attach(PIN_SPEED_CONTR);

  servoSteering.write(current_steering);
  servoSpeed.write(current_speed);

  return;
}

void loop() {
  servoSteering.write(current_steering);
  servoSpeed.write(current_speed);

  if (!Serial.available()) {
    return;
  }

  char ch = Serial.read();
  switch (ch) {
    case 'S': {
      int speed = readNumber();
      current_speed = speed;
      Serial.print("speed=");
      Serial.print(current_speed);
      Serial.print("\n");
      break;
    }

    case 'T': {
      int steering = readNumber();
      current_steering = steering;
      Serial.print("steering=");
      Serial.print(current_steering);
      Serial.print("\n");
      break;
    }

    default: {
      Serial.print("NOT A VALID VALUE YOU IDIOT WRITE ONE OF THESE:\n");
      Serial.print("S<number>n - set speed\n");
      Serial.print("T<number>n - set steering\n");
    }
  }
}

int readNumber() {
  char buffer[] = {' ',' ',' ',' '};
  Serial.readBytesUntil('n', buffer, 4);
  return atoi(buffer);
}

