/*
 * ActorDataStruct.h
 *
 *  Created on: Feb 14, 2017
 *      Author: WagPhi
 */

#ifndef INCLUDE_ACTORDATASTRUCT_H_
#define INCLUDE_ACTORDATASTRUCT_H_

typedef struct tag_WatchdogData_Ard_Comunicator
{
	uint8_t ui8tWatchdogData;
	long ltWatchdogDataSetPitxTimestamp;
	long ltWatchdogDataSendPitxTimestamp;
} tWatchdogDataArdComunicator;

typedef struct tag_EmergencyStopData_Ard_Comunicator
{
	uint8_t ui8tEmergencyStopData;
	long ltEmergencyStopDataSetPitxTimestamp;
	long ltEmergencyStopDataSendPitxTimestamp;
} tEmergencyStopDataArdComunicator;

typedef struct tag_LightData_Ard_Comunicator
{
	uint8_t ui8tLightData;
	long ltLightDataSetPitxTimestamp;
	long ltLightDataSendPitxTimestamp;
} tLightDataArdComunicator;

typedef struct tag_ServoSpeedData_Ard_Comunicator
{
	uint8_t ui8tServoSpeedData;
	long ltServoSpeedDataSetPitxTimestamp;
	long ltServoSpeedDataSendPitxTimestamp;
} tServoSpeedDataArdComunicator;

typedef struct tag_ServoSteeringData_Ard_Comunicator
{
	uint8_t ui8tServoSteeringData;
	long ltServoSteeringDataSetPitxTimestamp;
	long ltServoSteeringDataSendPitxTimestamp;
} tServoSteeringDataArdComunicator;

typedef struct tag_ActorData_Ard_Comunicator
{
	tServoSteeringDataArdComunicator sServoSteeringData;
	tServoSpeedDataArdComunicator sServoSpeedData;
	tLightDataArdComunicator sLightData;
	tWatchdogDataArdComunicator sWatchdogData;
	tEmergencyStopDataArdComunicator sEmergencyStopData;
} tActorDataArdComunicator;


#endif /* INCLUDE_ACTORDATASTRUCT_H_ */
