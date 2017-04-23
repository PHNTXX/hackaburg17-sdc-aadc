/*
 * ArduinoComunicator.cpp
 *
 *  Created on: Feb 10, 2017
 *      Author: WagPhi
 */

#include "ArduinoComunicator.h"
#include <signal.h>

namespace PITX {

bool ArduinoComunicator::cancellation = false;


ArduinoComunicator::ArduinoComunicator()
{
	printf("\033[34m#####################################################\033[0m\n");
	printf("\033[34m##            ALEXA   TOOLKIT  VER. 0.0.3          ##\033[0m\n");
	printf("\033[34m##                                                 ##\033[0m\n");
	printf("\033[34m##                       for                       ##\033[0m\n");
	printf("\033[34m##              AADC BOARD PITX XUBUNTU            ##\033[0m\n");
	printf("\033[34m##                                                 ##\033[0m\n");
	printf("\033[34m##                                                 ##\033[0m\n");
	printf("\033[34m#####################################################\033[0m\n");
	actor_data.sWatchdogData.ltWatchdogDataSetPitxTimestamp=0;
	actor_data.sEmergencyStopData.ltEmergencyStopDataSetPitxTimestamp=0;
	actor_data.sServoSteeringData.ltServoSteeringDataSetPitxTimestamp=0;
	actor_data.sServoSpeedData.ltServoSpeedDataSetPitxTimestamp=0;
	actor_data.sLightData.ltLightDataSetPitxTimestamp=0;

	debugbool=true;

	int counter=0;
	int TIMEOUT=10;
	bool b_ard_a=false;
	bool b_ard_f=false;
	bool b_ard_s=false;
	bool b_ard_b=false;
	bool b_all=false;
	if(debugbool){
		pFile = fopen ("LogCSVFile.csv","w");
		fprintf(pFile,"PITX Timestamp [ms];ARD Timestamp [ms];SensorID;Sensordata\n");
	}

	signal(SIGINT, LinuxSignalCatcher);

	ArduinoComunicator::cancellation=false;
	printf("###########-- INITIALISIERUNG --#####################\n");
	while(!b_all && (counter < TIMEOUT)){
		Serial::SerialDevice tempserial(counter);
		tempserial.init();
		switch(tempserial.getARDADRESS()){
			case ARD_ADDRESS_SENSORSFRONT:{
				printf("Detect Arduino Front on Port %d \t\t\t\t\t\033[32;1m [OK] \033[0m\n", counter);
				this->ard_front.ID=counter;
				b_ard_f=true;
				break;
			}
			case ARD_ADDRESS_SENSORSBACK:{
				printf("Detect Arduino Back on Port %d \t\t\t\t\t\033[32;1m [OK] \033[0m\n", counter);
				this->ard_back.ID=counter;
				b_ard_b=true;
				break;
			}
			case ARD_ADDRESS_SENSORSSIDE:{
				printf("Detect Arduino Side on Port %d \t\t\t\t\t\033[32;1m [OK] \033[0m\n", counter);
				this->ard_side.ID=counter;
				b_ard_s=true;
				break;
			}
			case ARD_ADDRESS_ACTORS:{
				printf("Detect Arduino Actor on Port %d \t\t\t\t\t\033[32;1m [OK] \033[0m\n", counter);
				this->ard_actor.ID=counter;
				b_ard_a=true;
				break;
			}
			default:{
				break;
			}
		};
		b_all=((b_ard_a && b_ard_b ) && ( b_ard_f && b_ard_s));
		counter++;
	}
	this->ard_actor.openPort();
	this->ard_actor.init();
	this->ard_front.openPort();
	this->ard_front.init();
	this->ard_back.openPort();
	this->ard_back.init();
	this->ard_side.openPort();
	this->ard_side.init();
	this->working();
}

ArduinoComunicator::~ArduinoComunicator() {
	// TODO Auto-generated destructor stub
	if(debugbool){
		fclose (pFile);
	}
}

bool ArduinoComunicator::allARDsReady()
{
	return ((this->ard_actor.getConnectionStatus() ));
}

void ArduinoComunicator::producer_thread(Serial::SerialDevice &dev, uint8_t id){
	tArduinoSensorFrame frame;
	while(!ArduinoComunicator::cancellation){
		dev.readFrame(frame);	//thats an active wait, so in the background calling linux' poll and sleeping until there are some data
		std::unique_lock<std::mutex> lock(mutexs[id]);
		queues[id].push(frame);
		run_commands[id] = true;
		conditions[id].notify_all();	//say the other thread there are data in its queue
	}
}

void ArduinoComunicator::consumer_thread(uint8_t id){
	tArduinoSensorFrame frame;
	while(!ArduinoComunicator::cancellation){
		std::unique_lock<std::mutex> lock(mutexs[id]);
		while(not run_commands[id]) conditions[id].wait(lock);	//waiting until we get the wake up signal from the producer
		file_mutex.lock();
		while(not queues[id].empty()){	//doing this because of there could be new data in the queue available when reading out of this because of slow processing and writing to file
			frame = queues[id].front();
			queues[id].pop();
			generateData(frame, sensor_data);
		}
		file_mutex.unlock();
		run_commands[id] = false;
	}
}


void ArduinoComunicator::generateData(tArduinoSensorFrame& frame, tSensorDataArdComunicator& sensor_data)
{
	switch(frame.sSensorHeader.ui8ID){
	 	case ID_ARD_SENS_STEERING:
	 		sensor_data.sSteeringData.ui32SteeringSensArdTimestamp= (uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
	 		sensor_data.sSteeringData.lSteeringSensPitxTimestamp= get_millis();
	 		sensor_data.sSteeringData.ui16SteeringSensData=(uint16_t)frame.sData.sSteeringData.ui16SteeringSens;
	 		if(debugbool){
	 		fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sSteeringData.lSteeringSensPitxTimestamp, sensor_data.sSteeringData.ui32SteeringSensArdTimestamp, ID_ARD_SENS_STEERING,sensor_data.sSteeringData.ui16SteeringSensData );
	 		}
	 		break;
		case ID_ARD_SENS_WHEEL_RIGHT:
			sensor_data.sWheelData.ui32WheelRightArdTimestamp =(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sWheelData.lWheelRightPitxTimestamp=get_millis();
			sensor_data.sWheelData.i8WheelDirRight = (int8_t)frame.sData.sWheelEncData.i8WheelDir;
			sensor_data.sWheelData.ui32WheelTachRight= (uint32_t)frame.sData.sWheelEncData.ui32WheelTach;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%d,%u\n",sensor_data.sWheelData.lWheelRightPitxTimestamp, sensor_data.sWheelData.ui32WheelRightArdTimestamp, ID_ARD_SENS_WHEEL_RIGHT,sensor_data.sWheelData.i8WheelDirRight, sensor_data.sWheelData.ui32WheelTachRight );
			}
			break;
		case ID_ARD_SENS_WHEEL_LEFT:
			sensor_data.sWheelData.ui32WheelLeftArdTimestamp =(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sWheelData.lWheelLeftPitxTimestamp=get_millis();
			sensor_data.sWheelData.i8WheelDirLeft= (int8_t)frame.sData.sWheelEncData.i8WheelDir;
		    sensor_data.sWheelData.ui32WheelTachLeft= (uint32_t)frame.sData.sWheelEncData.ui32WheelTach;
		    if(debugbool){
		    	fprintf(pFile,"%ld,%u,%u,%d,%u\n",sensor_data.sWheelData.lWheelLeftPitxTimestamp, sensor_data.sWheelData.ui32WheelLeftArdTimestamp, ID_ARD_SENS_WHEEL_LEFT,sensor_data.sWheelData.i8WheelDirLeft, sensor_data.sWheelData.ui32WheelTachLeft);
		    }
			break;
		case ID_ARD_SENS_IMU:
			sensor_data.sImuData.ui32ImuArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sImuData.lImuPitxTimestamp=get_millis();
			sensor_data.sImuData.i16A_x=(int16_t)frame.sData.sImuData.i16A_x;
			sensor_data.sImuData.i16A_y=(int16_t)frame.sData.sImuData.i16A_y;
			sensor_data.sImuData.i16A_z=(int16_t)frame.sData.sImuData.i16A_z;
			sensor_data.sImuData.i16Q_w=(int16_t)frame.sData.sImuData.i16Q_w;
			sensor_data.sImuData.i16Q_x=(int16_t)frame.sData.sImuData.i16Q_x;
			sensor_data.sImuData.i16Q_y=(int16_t)frame.sData.sImuData.i16Q_y;
			sensor_data.sImuData.i16Q_z=(int16_t)frame.sData.sImuData.i16Q_z;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%d,%d,%d,%d,%d,%d,%d\n",sensor_data.sImuData.lImuPitxTimestamp, sensor_data.sImuData.ui32ImuArdTimestamp, ID_ARD_SENS_IMU,sensor_data.sImuData.i16A_x, sensor_data.sImuData.i16A_y,sensor_data.sImuData.i16A_z,sensor_data.sImuData.i16Q_w, sensor_data.sImuData.i16Q_x, sensor_data.sImuData.i16Q_y, sensor_data.sImuData.i16Q_z);
			}
			break;
		case ID_ARD_SENS_US_FRONT_LEFT:
			sensor_data.sUSData.sUSDataFront.ui32USFrontLeftArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataFront.lUSFrontLeftPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataFront.ui16USFrontLeftData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataFront.lUSFrontLeftPitxTimestamp, sensor_data.sUSData.sUSDataFront.ui32USFrontLeftArdTimestamp, ID_ARD_SENS_US_FRONT_LEFT,sensor_data.sUSData.sUSDataFront.ui16USFrontLeftData);
			}
			break;
		case ID_ARD_SENS_US_FRONT_RIGHT:
			sensor_data.sUSData.sUSDataFront.ui32USFrontRightArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataFront.lUSFrontRightPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataFront.ui16USFrontRightData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataFront.lUSFrontRightPitxTimestamp, sensor_data.sUSData.sUSDataFront.ui32USFrontRightArdTimestamp, ID_ARD_SENS_US_FRONT_RIGHT,sensor_data.sUSData.sUSDataFront.ui16USFrontRightData);
			}
			break;
		case ID_ARD_SENS_US_FRONT_CENTER:
			sensor_data.sUSData.sUSDataFront.ui32USFrontCenterArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataFront.lUSFrontCenterPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataFront.ui16USFrontCenterData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataFront.lUSFrontCenterPitxTimestamp, sensor_data.sUSData.sUSDataFront.ui32USFrontCenterArdTimestamp, ID_ARD_SENS_US_FRONT_CENTER,sensor_data.sUSData.sUSDataFront.ui16USFrontCenterData);
			}
			break;
		case ID_ARD_SENS_US_FRONT_CENTER_LEFT :
			sensor_data.sUSData.sUSDataFront.ui32USFrontCenterLeftArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataFront.lUSFrontCenterLeftPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataFront.ui16USFrontCenterLeftData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataFront.lUSFrontCenterLeftPitxTimestamp, sensor_data.sUSData.sUSDataFront.ui32USFrontCenterLeftArdTimestamp, ID_ARD_SENS_US_FRONT_CENTER_LEFT,sensor_data.sUSData.sUSDataFront.ui16USFrontCenterLeftData);
			}
			break;
		case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:
			sensor_data.sUSData.sUSDataFront.ui32USFrontCenterRightArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataFront.lUSFrontCenterRightPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataFront.ui16USFrontCenterRightData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataFront.lUSFrontCenterRightPitxTimestamp, sensor_data.sUSData.sUSDataFront.ui32USFrontCenterRightArdTimestamp, ID_ARD_SENS_US_FRONT_CENTER_RIGHT,sensor_data.sUSData.sUSDataFront.ui16USFrontCenterRightData);
			}
			break;
		case ID_ARD_SENS_US_SIDE_LEFT:
			sensor_data.sUSData.sUSDataSide.ui32USSideLeftArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataSide.lUSSideLeftPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataSide.ui16USSideLeftData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataSide.lUSSideLeftPitxTimestamp, sensor_data.sUSData.sUSDataSide.ui32USSideLeftArdTimestamp, ID_ARD_SENS_US_SIDE_LEFT,sensor_data.sUSData.sUSDataSide.ui16USSideLeftData);
			}
			break;
		case ID_ARD_SENS_US_SIDE_RIGHT:
			sensor_data.sUSData.sUSDataSide.ui32USSideRightArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataSide.lUSSideRightPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataSide.ui16USSideRightData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataSide.lUSSideRightPitxTimestamp, sensor_data.sUSData.sUSDataSide.ui32USSideRightArdTimestamp, ID_ARD_SENS_US_SIDE_RIGHT,sensor_data.sUSData.sUSDataSide.ui16USSideRightData);
			}
			break;
		case ID_ARD_SENS_US_BACK_LEFT:
			sensor_data.sUSData.sUSDataBack.ui32USBackLeftArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataBack.lUSBackLeftPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataBack.ui16USBackLeftData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataBack.lUSBackLeftPitxTimestamp, sensor_data.sUSData.sUSDataBack.ui32USBackLeftArdTimestamp, ID_ARD_SENS_US_BACK_LEFT,sensor_data.sUSData.sUSDataBack.ui16USBackLeftData);
			}
			break;
		case ID_ARD_SENS_US_BACK_RIGHT:
			sensor_data.sUSData.sUSDataBack.ui32USBackRightArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataBack.lUSBackRightPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataBack.ui16USBackRightData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataBack.lUSBackRightPitxTimestamp, sensor_data.sUSData.sUSDataBack.ui32USBackRightArdTimestamp, ID_ARD_SENS_US_BACK_RIGHT,sensor_data.sUSData.sUSDataBack.ui16USBackRightData);
			}
			break;
		case ID_ARD_SENS_US_BACK_CENTER:
			sensor_data.sUSData.sUSDataBack.ui32USBackCenterArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sUSData.sUSDataBack.lUSBackCenterPitxTimestamp=get_millis();
			sensor_data.sUSData.sUSDataBack.ui16USBackCenterData=(uint16_t)frame.sData.sUsData.ui16UsData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sUSData.sUSDataBack.lUSBackCenterPitxTimestamp, sensor_data.sUSData.sUSDataBack.ui32USBackCenterArdTimestamp, ID_ARD_SENS_US_BACK_CENTER,sensor_data.sUSData.sUSDataBack.ui16USBackCenterData);
			}
			break;
		case ID_ARD_SENS_VOLT_SPEEDCONT:
			sensor_data.sVoltageSpeedData.ui32VoltageSpeedArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sVoltageSpeedData.lVoltageSpeedPitxTimestamp=get_millis();
			sensor_data.sVoltageSpeedData.ui16VoltageSpeedData=(uint16_t)frame.sData.sVoltageData.ui16VoltageData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sVoltageSpeedData.lVoltageSpeedPitxTimestamp, sensor_data.sVoltageSpeedData.ui32VoltageSpeedArdTimestamp, ID_ARD_SENS_VOLT_SPEEDCONT,sensor_data.sVoltageSpeedData.ui16VoltageSpeedData);
			}
			break;
		case ID_ARD_SENS_VOLT_MEAS:
			sensor_data.sVoltageMeasData.ui32VoltageMeasArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sVoltageMeasData.lVoltageMeasPitxTimestamp=get_millis();
			sensor_data.sVoltageMeasData.ui16VoltageMeasData=(uint16_t)frame.sData.sVoltageData.ui16VoltageData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sVoltageMeasData.lVoltageMeasPitxTimestamp, sensor_data.sVoltageMeasData.ui32VoltageMeasArdTimestamp, ID_ARD_SENS_VOLT_MEAS,sensor_data.sVoltageMeasData.ui16VoltageMeasData);
			}
			break;
		case ID_ARD_SENS_CURR_SPEED:
			sensor_data.sCurrSpeedData.ui32CurrSpeedArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sCurrSpeedData.lCurrSpeedPitxTimestamp=get_millis();
			sensor_data.sCurrSpeedData.ui8CurrSpeedData=(uint8_t)frame.sData.sCurrData.ui8CurrData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sCurrSpeedData.lCurrSpeedPitxTimestamp, sensor_data.sCurrSpeedData.ui32CurrSpeedArdTimestamp, ID_ARD_SENS_CURR_SPEED,sensor_data.sCurrSpeedData.ui8CurrSpeedData);
			}
			break;
		case ID_ARD_SENS_CURR_STEER:
			sensor_data.sCurrSteerData.ui32CurrSteerArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sCurrSteerData.lCurrSteerPitxTimestamp=get_millis();
			sensor_data.sCurrSteerData.ui8CurrSteerData=(uint8_t)frame.sData.sCurrData.ui8CurrData;
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sCurrSteerData.lCurrSteerPitxTimestamp, sensor_data.sCurrSteerData.ui32CurrSteerArdTimestamp, ID_ARD_SENS_CURR_STEER,sensor_data.sCurrSteerData.ui8CurrSteerData);
			}
			break;

		case ID_ARD_SENS_RESERVED_FRONT:
			break;
		case ID_ARD_SENS_RESERVED_BACK:
			break;
		case ID_ARD_SENSOR_INFO:
			sensor_data.sInfoData.ui32InfoArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sInfoData.ui8ArduinoAddress=(uint8_t)frame.sData.sInfoData.ui8ArduinoAddress;
			sensor_data.sInfoData.ui16ArduinoVersion=(uint16_t)frame.sData.sInfoData.ui16ArduinoVersion;
			sensor_data.sInfoData.lInfoPitxTimestamp=get_millis();
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u,%u\n",sensor_data.sInfoData.lInfoPitxTimestamp, sensor_data.sInfoData.ui32InfoArdTimestamp,ID_ARD_SENSOR_INFO, sensor_data.sInfoData.ui8ArduinoAddress, sensor_data.sInfoData.ui16ArduinoVersion);
			}
			break;
		case ID_ARD_SENS_ERROR:
			sensor_data.sErrorData.ui32ErrorArdTimestamp=(uint32_t)frame.sSensorHeader.ui32ArduinoTimestamp;
			sensor_data.sErrorData.ui16ErrorNr=(uint16_t)frame.sData.sErrorData.ui16ErrorNr;
			sensor_data.sErrorData.lErrorPitxTimestamp=get_millis();
			if(debugbool){
				fprintf(pFile,"%ld,%u,%u,%u\n",sensor_data.sErrorData.lErrorPitxTimestamp, sensor_data.sErrorData.ui32ErrorArdTimestamp,ID_ARD_SENS_ERROR, sensor_data.sErrorData.ui16ErrorNr);
			}
			break;
		default:
			break;
	};
}

void ArduinoComunicator::working()
{
	printf("###############-- WORKING --#########################\n");
	//this could be a little bit more nicer...
	producers[0] = std::thread(&ArduinoComunicator::producer_thread, this, std::ref(ard_actor), 0);
	producers[1] = std::thread(&ArduinoComunicator::producer_thread, this, std::ref(ard_side), 1);
	producers[2] = std::thread(&ArduinoComunicator::producer_thread, this, std::ref(ard_front), 2);
	producers[3] = std::thread(&ArduinoComunicator::producer_thread, this, std::ref(ard_back), 3);

	for(uint8_t i = 0; i < number_of_threads; i++){
		consumers[i] = std::thread(&ArduinoComunicator::consumer_thread, this, i);
	}

	for(uint8_t i = 0; i < number_of_threads; i++){
		producers[i].join();
		conditions[i].notify_all();
		consumers[i].join();
	}


	printf("###############--   END   --#########################\n");
}

void ArduinoComunicator::settActorDataArdComunicator()
{

}


void ArduinoComunicator::controlcircuit()
{
	while(!(ArduinoComunicator::cancellation)){
	}
}




long ArduinoComunicator::get_millis(void) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC,&ts);
	return (long)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

void ArduinoComunicator::setActorData(uint8_t ID, uint8_t data)
{
//	std::lock_guard<std::mutex> lock(mx_actor_data);
	switch (ID){
		case ID_ARD_ACT_WATCHDOG:
		{
			std::lock_guard<std::mutex> lock(mx_actor_data);
			actor_data.sWatchdogData.ltWatchdogDataSetPitxTimestamp = get_millis();
			actor_data.sWatchdogData.ltWatchdogDataSendPitxTimestamp =0;
			actor_data.sWatchdogData.ui8tWatchdogData=data;
			break;
		}
		case ID_ARD_ACT_EMERGENCY_STOP:
		{
			std::lock_guard<std::mutex> lock(mx_actor_data);
			actor_data.sEmergencyStopData.ltEmergencyStopDataSetPitxTimestamp=get_millis();
			actor_data.sEmergencyStopData.ltEmergencyStopDataSendPitxTimestamp=0;
			actor_data.sEmergencyStopData.ui8tEmergencyStopData=data;
			break;
		}
		case ID_ARD_ACT_STEER_SERVO:
			actor_data.sServoSteeringData.ltServoSteeringDataSetPitxTimestamp=get_millis();
			actor_data.sServoSteeringData.ltServoSteeringDataSendPitxTimestamp=0;
			actor_data.sServoSteeringData.ui8tServoSteeringData=data;
			break;
		case ID_ARD_ACT_SPEED_CONTR:
			actor_data.sServoSpeedData.ltServoSpeedDataSetPitxTimestamp=get_millis();
			actor_data.sServoSpeedData.ltServoSpeedDataSendPitxTimestamp=0;
			actor_data.sServoSpeedData.ui8tServoSpeedData=data;
			break;
		case ID_ARD_ACT_LIGHT:
			actor_data.sLightData.ltLightDataSetPitxTimestamp=get_millis();
			actor_data.sLightData.ltLightDataSendPitxTimestamp=0;
			actor_data.sLightData.ui8tLightData=data;
			break;
	}
}

int ArduinoComunicator::unsentFrame()
{
	printf("AktordataSEND==0>  %d\tAktorSet!=0>  %d\n",actor_data.sWatchdogData.ltWatchdogDataSendPitxTimestamp==0,actor_data.sWatchdogData.ltWatchdogDataSetPitxTimestamp!=0);
	if(actor_data.sWatchdogData.ltWatchdogDataSendPitxTimestamp==0 && actor_data.sWatchdogData.ltWatchdogDataSetPitxTimestamp!=0){
			return ID_ARD_ACT_WATCHDOG;
		}
	if(actor_data.sEmergencyStopData.ltEmergencyStopDataSendPitxTimestamp==0 && actor_data.sEmergencyStopData.ltEmergencyStopDataSetPitxTimestamp!=0){
			return ID_ARD_ACT_EMERGENCY_STOP;
		}
	if(actor_data.sServoSteeringData.ltServoSteeringDataSendPitxTimestamp==0 && actor_data.sServoSteeringData.ltServoSteeringDataSetPitxTimestamp!=0){
			return ID_ARD_ACT_STEER_SERVO;
		}
	if(actor_data.sServoSpeedData.ltServoSpeedDataSendPitxTimestamp==0 && actor_data.sServoSpeedData.ltServoSpeedDataSetPitxTimestamp!=0){
			return ID_ARD_ACT_SPEED_CONTR;
		}
	if(actor_data.sLightData.ltLightDataSendPitxTimestamp==0 && actor_data.sLightData.ltLightDataSetPitxTimestamp!=0){
		return ID_ARD_ACT_LIGHT;
	}
	return -1;
}



void ArduinoComunicator::send_actor_data_ard()
{
	while(!(ArduinoComunicator::cancellation)){
//		std::lock_guard<std::mutex> lock(mx_actor_data);
		switch(unsentFrame()){
			case ID_ARD_ACT_WATCHDOG:
				ard_actor.writeFrame(ID_ARD_ACT_WATCHDOG, actor_data.sWatchdogData.ui8tWatchdogData);
				actor_data.sWatchdogData.ltWatchdogDataSendPitxTimestamp =get_millis();
				actor_data.sWatchdogData.ltWatchdogDataSetPitxTimestamp=0;
				break;
			case ID_ARD_ACT_EMERGENCY_STOP:
				ard_actor.writeFrame(ID_ARD_ACT_EMERGENCY_STOP, actor_data.sEmergencyStopData.ui8tEmergencyStopData);
				actor_data.sEmergencyStopData.ltEmergencyStopDataSendPitxTimestamp=get_millis();
				actor_data.sEmergencyStopData.ltEmergencyStopDataSetPitxTimestamp=0;
				break;
			case ID_ARD_ACT_STEER_SERVO:
				ard_actor.writeFrame(ID_ARD_ACT_STEER_SERVO, actor_data.sServoSteeringData.ui8tServoSteeringData);
				actor_data.sServoSteeringData.ltServoSteeringDataSendPitxTimestamp=get_millis();
				actor_data.sServoSteeringData.ltServoSteeringDataSetPitxTimestamp=0;
				break;
			case ID_ARD_ACT_SPEED_CONTR:

				ard_actor.writeFrame(ID_ARD_ACT_SPEED_CONTR, actor_data.sServoSpeedData.ui8tServoSpeedData);
				actor_data.sServoSpeedData.ltServoSpeedDataSendPitxTimestamp=get_millis();
				actor_data.sServoSpeedData.ltServoSpeedDataSetPitxTimestamp=0;
				break;
			case ID_ARD_ACT_LIGHT:
				ard_actor.writeFrame(ID_ARD_ACT_LIGHT, actor_data.sLightData.ui8tLightData);
				actor_data.sLightData.ltLightDataSendPitxTimestamp=get_millis();
				actor_data.sLightData.ltLightDataSetPitxTimestamp=0;
				break;
		}
	}
}

void ArduinoComunicator::LinuxSignalCatcher(int signal){
	ArduinoComunicator::cancellation = true;
}

}
