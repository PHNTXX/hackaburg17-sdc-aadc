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
* $Author:: forchhe#$  $Date:: 2016-01-14 13:25:26#$ $Rev:: 43021   $
**********************************************************************/

 
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

#include "AADC_Com.h"

AADC_Com::AADC_Com( )
{
	recvFrame.sActorHeader.ui8ID = ID_ARD_EMPTY;
}

void AADC_Com::open( unsigned long baudrate )
{
    Serial.begin( baudrate );
    Serial.flush();
}

int AADC_Com::clearRecvBuffer(){
	int avlbleByte = Serial.available();
	for(int i = 0; i < avlbleByte; i++){
		Serial.read();
    }
	
	return avlbleByte;
}

void AADC_Com::close( )
{
      Serial.end( );
}

 bool AADC_Com::receiveActorFrame(tArduinoActorFrame &frame)
 {
	int avlbleByte = Serial.available();
	
	if(avlbleByte < sizeof(tArduinoActorFrame)){
		return false;
	}
	
	tUInt8 *buffer = (tUInt8*) &frame;
	
	//search SOF
	frame.sActorHeader.ui8SOF = Serial.read(); 

	if(frame.sActorHeader.ui8SOF != ID_ARD_SOF){
		tErrorData error;
		error.ui16ErrorNr = ERROR_SOF_NOT_FOUND;
		sendSensorFrame(ID_ARD_SENS_ERROR,millis(),sizeof(tErrorData),(tUInt8*) &error);
		return false; 
	}
	
	// SOF found
	for(int i = 0; i < sizeof(tArduinoActorFrame) - sizeof(ID_ARD_SOF); i++){
		buffer[sizeof(ID_ARD_SOF) +i] = Serial.read(); 
	}
	
	
	tUInt16 calc_crc = fletcher16(buffer,sizeof(tArduinoActorFrame) - sizeof(INIT_CRC));
	
	//check CRC
	if(frame.ui16CRC != calc_crc){
		tErrorData error;
		error.ui16ErrorNr = ERROR_CRC_INVALID;
		sendSensorFrame(ID_ARD_SENS_ERROR,millis(),sizeof(tErrorData),(tUInt8*) &error);
		return false;
	}
	
	return true;
	
 }
 
 int AADC_Com::sendSensorFrame(const tUInt8 id, const tUInt32 timestamp, const tUInt8 len,const tUInt8 rawData[] )
 {
	 
	if(len > sizeof(tArduinoSensorFrame) - sizeof(tArduinoSensorHeader) - sizeof(INIT_CRC))return 0;
	
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
 
// Very Fast checksum calculation (like CRC16). Optimized for 8-bit CPUs.
tUInt16 fletcher16( tUInt8 const *data, tUInt8 bytes )
{
        tUInt16 sum1 = 0xff, sum2 = 0xff;
 
        while (bytes) {
                tUInt8 tlen = bytes > 20 ? 20 : bytes;
                bytes -= tlen;
                do {
                        sum2 += sum1 += *data++;
                } while (--tlen);
                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
        }
        /* Second reduction step to reduce sums to 8 bits */
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
        return sum2 << 8 | sum1;
}
 
 #pragma pack(pop)   /* restore original alignment from stack */

