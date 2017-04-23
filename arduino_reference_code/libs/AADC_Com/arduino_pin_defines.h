
/*! input of right wheel encoder - arduino #4 Sensors */
#define PIN_WHEEL_RIGHT_TACH int(2)
#define PIN_INT_WHEEL_RIGHT_TACH int(1)
#define PIN_WHEEL_RIGHT_DIR int(10)

/*! input of left wheel encoder - arduino #4 Sensors */
#define PIN_WHEEL_LEFT_TACH int(1)
#define PIN_INT_WHEEL_LEFT_TACH int(3)
#define PIN_WHEEL_LEFT_DIR int(8)

/*! ultrasonic middle right side - arduino #4 Sensors */
#define PIN_US_SIDE_LEFT_ECH int(3)
#define PIN_INT_US_SIDE_LEFT_ECH int(0)
#define PIN_US_SIDE_LEFT_TRIG int(9)

/*! ultrasonic middle left side - arduino #4 Sensors */
#define PIN_US_SIDE_RIGHT_ECH int(0)
#define PIN_INT_US_SIDE_RIGHT_ECH int(2)
#define PIN_US_SIDE_RIGHT_TRIG int(11)

/*! input of voltage measurement circuit - arduino #4 Sensors */
#define PIN_ANA_VOLT_MEAS int(5)

/*! input of voltage speed controller circuit - arduino #4 Sensors */
#define PIN_ANA_VOLT_SPEEDCTRL int(4)

/*! input of steering servo sensor - arduino #4 Sensors */
//#define PIN_ANA_STEER_SENS int(3) //TODO: on layout is connected to D5, should be to D4 or D6 (A6 or A7)

/*! input of arduino device coding bit one  - all arduinos 
#	Arduino #1 LOW  (sensors front)
#	Arduino #2 LOW  (sensors rear)
#	Arduino #3 HIGH (sensors center)
#	Arduino #4 HIGH (actuators)
*/
#define PIN_ARD_CODE_1 A0

/*! input of arduino device coding bit two - all arduinos 
#	Arduino #1 LOW  (sensors front)
#	Arduino #2 HIGH (sensors rear)
#	Arduino #3 LOW  (sensors center)
#	Arduino #4 HIGH (actuators)
*/
#define PIN_ARD_CODE_2 int(12)

/*! LED on board - all arduinos */
#define PIN_ANA_LED0_BOARD int(13) 

/*! LED on board - all arduinos */
#define PIN_ANA_LED1_BOARD A1 

/*! LED on board - all arduinos */
//#define PIN_ANA_LED2_BOARD SS 

/*! relais for enabling speed controller - arduino #3 actuators */
#define PIN_RELAIS int(4)

/*! output of pwm for speed contr - arduino #3 actuators*/
#define PIN_SPEED_CONTR int(5)

/*! output of pwm for steering contr - arduino #3 actuators*/
#define PIN_STEER_SERV int(6)

/*! output for turnsignal left light - arduino #3 actuators*/
#define PIN_TURNSIGNAL_LEFT int(11)

/*! output for turnsignal right light - arduino #3 actuators*/
#define PIN_TURNSIGNAL_RIGHT int(7)

/*! output for brake light - arduino #3 actuators */
#define PIN_BRAKE_LIGHT int(3)

/*! output for tail light - arduino #3 actuators */
#define PIN_REVERSE_LIGHT int(8)

/*! output for dim light - arduino #3 actuators */
#define PIN_DIM_LIGHT int(10)

/*! input for calibration poti of steering servo - arduino #3 actuators */
#define PIN_ANA_POTI_STEER_SERVO A5

/*! input for calibration poti of speed controller - arduino #3 actuators */
#define PIN_ANA_POTI_SPEED_CONTR A4

/*! input for check if remote receiver enabled - arduino #3 actuators */
#define PIN_REMOTE_REC_EN A2

/*! ultrasonic front and rear left - arduino #1 sensors and arduino #2 sensors */
#define PIN_US_LEFT_ECH int(7)
#define PIN_INT_US_LEFT_ECH int(4)
#define PIN_US_LEFT_TRIG int(6)

/*! ultrasonic front and rear center - arduino #1 sensors and arduino #2 sensors */
#define PIN_US_CENTER_ECH int(0)
#define PIN_INT_US_CENTER_ECH int(2)
#define PIN_US_CENTER_TRIG int(5)

/*! ultrasonic front and rear center left - arduino #1 sensors and arduino #2 sensors */
#define PIN_US_CENTER_LEFT_ECH int(3)
#define PIN_INT_US_CENTER_LEFT_ECH int(0)
#define PIN_US_CENTER_LEFT_TRIG int(9)

/*! ultrasonic front and rear center right - arduino #1 sensors and arduino #2 sensors */
#define PIN_US_CENTER_RIGHT_ECH int(2)
#define PIN_INT_US_CENTER_RIGHT_ECH int(1)
#define PIN_US_CENTER_RIGHT_TRIG int(8)

/*! ultrasonic front and rear right - arduino #1 sensors and arduino #2 sensors */
#define PIN_US_RIGHT_ECH int(1)
#define PIN_INT_US_RIGHT_ECH int(3)
#define PIN_US_RIGHT_TRIG int(4)

/*! gyro input - arduino #2 sensors */
#define PIN_INT_GYRO int(9)
#define PIN_GYRO_VSS_ENABLE int(10)

