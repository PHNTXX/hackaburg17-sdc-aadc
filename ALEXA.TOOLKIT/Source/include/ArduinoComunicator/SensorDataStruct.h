/*
 * SensorDataStruct.h
 *
 *  Created on: Feb 14, 2017
 *      Author: WagPhi
 */

#ifndef INCLUDE_SENSORDATASTRUCT_H_
#define INCLUDE_SENSORDATASTRUCT_H_


//ULTRASCHALLDATA

typedef struct tag_USData_Front_Ard_Comunicator
{
	uint16_t ui16USFrontLeftData;            /*!< counter from right wheel */
	uint16_t ui16USFrontCenterLeftData;
	uint16_t ui16USFrontCenterData;
	uint16_t ui16USFrontCenterRightData;
	uint16_t ui16USFrontRightData;
	uint32_t ui32USFrontLeftArdTimestamp;            /*!< counter from right wheel */
	uint32_t ui32USFrontCenterLeftArdTimestamp;
	uint32_t ui32USFrontCenterArdTimestamp;
	uint32_t ui32USFrontCenterRightArdTimestamp;
	uint32_t ui32USFrontRightArdTimestamp;
	long lUSFrontLeftPitxTimestamp;            /*!< counter from right wheel */
	long lUSFrontCenterLeftPitxTimestamp;
	long lUSFrontCenterPitxTimestamp;
	long lUSFrontCenterRightPitxTimestamp;
	long lUSFrontRightPitxTimestamp;
} tUSDataFrontArdComunicator;

typedef struct tag_USData_Side_Ard_Comunicator
{
	uint32_t ui32USSideLeftArdTimestamp;            /*!< counter from right wheel */
	uint32_t ui32USSideRightArdTimestamp;
	long lUSSideLeftPitxTimestamp;            /*!< counter from right wheel */
	long lUSSideRightPitxTimestamp;
	uint16_t ui16USSideLeftData;            /*!< counter from right wheel */
	uint16_t ui16USSideRightData;

} tUSDataSideArdComunicator;

typedef struct tag_USData_Back_Ard_Comunicator
{
	uint16_t ui16USBackLeftData;            /*!< counter from right wheel */
	uint16_t ui16USBackCenterData;
	uint16_t ui16USBackRightData;
	uint32_t ui32USBackLeftArdTimestamp;            /*!< counter from right wheel */
	uint32_t ui32USBackCenterArdTimestamp;
	uint32_t ui32USBackRightArdTimestamp;
	long lUSBackLeftPitxTimestamp;            /*!< counter from right wheel */
	long lUSBackCenterPitxTimestamp;
	long lUSBackRightPitxTimestamp;
} tUSDataBackArdComunicator;

typedef struct tag_USData_Ard_Comunicator
{
	tUSDataFrontArdComunicator sUSDataFront;            /*!< counter from right wheel */
	tUSDataSideArdComunicator sUSDataSide;            /*!< counter from right wheel */
	tUSDataBackArdComunicator sUSDataBack;            /*!< counter from right wheel */
} tUSDataArdComunicator;




//Steering
typedef struct tag_SteeringData_Ard_Comunicator
{
	uint16_t ui16SteeringSensData;
	uint32_t ui32SteeringSensArdTimestamp;
	long lSteeringSensPitxTimestamp;
} tSteeringDataArdComunicator;


//WheelData
typedef struct tag_WheelData_Ard_Comunicator
{
	uint32_t ui32WheelTachRight;            /*!< counter from right wheel */
	int8_t i8WheelDirRight;
	uint32_t ui32WheelRightArdTimestamp;            /*!< counter from right wheel */
	long lWheelRightPitxTimestamp;            /*!< counter from right wheel */
	uint32_t ui32WheelTachLeft;            /*!< counter from right wheel */
	int8_t i8WheelDirLeft;
	uint32_t ui32WheelLeftArdTimestamp;            /*!< counter from right wheel */
	long lWheelLeftPitxTimestamp;            /*!< counter from right wheel */

} tWheelDataArdComunicator;

//IMU
typedef struct tag_ImuData_Ard_Comunicator
{
	int16_t i16A_x;                    /*!< acceleration x-axis */
	int16_t i16A_y;                    /*!< acceleration y-axis */
	int16_t i16A_z;                     /*!< acceleration z-axis */

	int16_t i16Q_w;                     /*!< attitude */
	int16_t i16Q_x;                     /*!< attitude */
	int16_t i16Q_y;                     /*!< attitude */
	int16_t i16Q_z;                     /*!< attitude */
	uint32_t ui32ImuArdTimestamp;
	long lImuPitxTimestamp;
} tImuDataArdComunicator;

typedef struct tag_VoltageSpeedData_Ard_Comunicator
{
	uint16_t ui16VoltageSpeedData;                /*!< data from voltage measurement pins; has to be calculated */
	uint32_t ui32VoltageSpeedArdTimestamp;
	long lVoltageSpeedPitxTimestamp;
} tVoltageSpeedDataArdComunicator;

typedef struct tag_VoltageMeasData_Ard_Comunicator
{
	uint16_t ui16VoltageMeasData;                /*!< data from voltage measurement pins; has to be calculated */
	uint32_t ui32VoltageMeasArdTimestamp;
	long lVoltageMeasPitxTimestamp;
} tVoltageMeasDataArdComunicator;

typedef struct tag_CurrSpeedData_Ard_Comunicator
{
	uint16_t ui8CurrSpeedData;                /*!< data from voltage measurement pins; has to be calculated */
	uint32_t ui32CurrSpeedArdTimestamp;
	long lCurrSpeedPitxTimestamp;
} tCurrSpeedDataArdComunicator;

typedef struct tag_CurrSteerData_Ard_Comunicator
{
	uint16_t ui8CurrSteerData;                /*!< data from voltage measurement pins; has to be calculated */
	uint32_t ui32CurrSteerArdTimestamp;
	long lCurrSteerPitxTimestamp;
} tCurrSteerDataArdComunicator;



typedef struct tag_InfoData_Ard_Comunicator
{
	uint8_t ui8ArduinoAddress;  //e.g. ARD_ADDRESS_SENSORSFRONT
	uint16_t ui16ArduinoVersion;
	uint32_t ui32InfoArdTimestamp;
	long lInfoPitxTimestamp;
} tInfoDataArdComunicator;

typedef struct tag_ErrorData_Ard_Comunicator
{
	uint16_t ui16ErrorNr;
	uint32_t ui32ErrorArdTimestamp;
	long lErrorPitxTimestamp;
} tErrorDataArdComunicator;

typedef struct tag_SensorData_Ard_Comunicator
{
	tUSDataArdComunicator sUSData;            /*!< counter from right wheel */
	tSteeringDataArdComunicator sSteeringData;
	tWheelDataArdComunicator sWheelData;
	tImuDataArdComunicator sImuData;
	tVoltageSpeedDataArdComunicator sVoltageSpeedData;
	tVoltageMeasDataArdComunicator sVoltageMeasData;
	tCurrSpeedDataArdComunicator sCurrSpeedData;
	tCurrSteerDataArdComunicator sCurrSteerData;
	tInfoDataArdComunicator sInfoData;
	tErrorDataArdComunicator sErrorData;
} tSensorDataArdComunicator;

#endif /* INCLUDE_SENSORDATASTRUCT_H_ */
