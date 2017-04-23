/*
 * SerialDevice.h
 *
 *  Created on: Feb 2, 2017
 *      Author: WagPhi
 */

#ifndef INCLUDE_SERIALDEVICE_H_
#define INCLUDE_SERIALDEVICE_H_
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>



#include "arduinoProtokol.h"
#define _POSIX_SOURCE 1
namespace Serial {

class SerialDevice {

	int portReference;
	bool connected;
	std::string ARD_NAME;
	tUInt8 ARD_ADRESS;



	void flush();
	int BytesAvailableInput();
	int BytesAvailableOutput();
	bool isSensorIdValid(tUInt8 ui8Id);
	void CheckForArduinoErrorMessage(tArduinoSensorFrame& m_oReceivedFrame);
	void DetectArduinoAddress(tArduinoSensorFrame& m_oReceivedFrame);
	tUInt16  fletcher16( tUInt8* data, tUInt8 bytes );
public:
	int ID;
	SerialDevice();
	SerialDevice( int portNumber);
	SerialDevice( const SerialDevice &serialdevice);

	~SerialDevice();
	void openPort();
	void init();
	void readFrame(tArduinoSensorFrame& m_oReceivedFrame);
	void writeFrame(tUInt8 ui8Id, tUInt8 ui8Data);
	void writeFrame(tUInt8 ui8Id);

	int getID(){return this->ID;};
	bool getConnectionStatus(){return (bool)(this->connected);};
	tUInt8 getARDADRESS(){return this->ARD_ADRESS;};
	std::string getName(){return this->ARD_NAME;};
	int getBufferOutputCount(){return BytesAvailableOutput();};

};

} /* namespace Serial */

#endif /* INCLUDE_SERIALDEVICE_H_ */
