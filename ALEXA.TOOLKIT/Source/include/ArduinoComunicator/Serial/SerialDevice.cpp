/*
 * SerialDevice.cpp
 *
 *  Created on: Feb 2, 2017
 *      Author: WagPhi
 */

#include "SerialDevice.h"

namespace Serial {

SerialDevice::SerialDevice()
{
	this->ID=-1;
	this->portReference=-1;
	this->connected=false;
	this->ARD_ADRESS=ARD_ADDRESS_UNKNOWN;
	this->ARD_NAME="UNKOWN";
}


SerialDevice::SerialDevice( int portNumber)
{
	this->ID=portNumber;
	this->portReference=-1;
	this->connected=false;
	this->ARD_ADRESS=ARD_ADDRESS_UNKNOWN;
	this->ARD_NAME="UNKOWN";

	char filename[10];
	sprintf(filename,"/dev/ttyACM%d", portNumber);
//	printf("Connecting SerialPort %d with Port %s\t", portNumber, filename );
	close(portReference);
	int n=open(filename, O_RDWR | O_NOCTTY |O_SYNC);
	if(n<0){
//		printf("\033[31m [FAILED] \033[0m\n");
	}
	else{
		this->portReference=n;
//		printf("\033[32;1m [OK] \033[0m\n");
	}


}

SerialDevice::SerialDevice( const SerialDevice& serialdevice)
{
	this->ID=serialdevice.ID;
	this->portReference=serialdevice.portReference;
	this->connected=serialdevice.connected;
	this->ARD_ADRESS=serialdevice.ARD_ADRESS;
	printf("PortID%d\n",ID);
}



SerialDevice::~SerialDevice() {
	// TODO Auto-generated destructor stub
	close(this->portReference);
	this->connected=false;
	this->ID=-1;
	this->ARD_ADRESS=ARD_ADDRESS_UNKNOWN;
	this->ARD_NAME="UNKOWN";
}

void SerialDevice::openPort()
{
	char filename[10];
	sprintf(filename,"/dev/ttyACM%d", ID);
	printf("Connecting SerialPort %d with Port %s\t\t\t", ID, filename );
	close(portReference);
	int n=open(filename, O_RDWR | O_NOCTTY |O_SYNC );
	if(n<0){
		printf("\033[31m [FAILED] \033[0m\n");
	}
	else{
		this->portReference=n;
		printf("\033[32;1m [OK] \033[0m\n");
	}
}

void SerialDevice::init(){
	struct termios tTermiosOptions;
	if (tcgetattr(this->portReference, &tTermiosOptions) == -1) {
		printf("Error (SerialDevice::Init): Get Attributes termios \t\t\x1B[31m [FAILED] \033[0m\n");
		exit(0);
	}
	cfsetospeed (&tTermiosOptions, (speed_t)B115200);
	cfsetispeed (&tTermiosOptions, (speed_t)B115200);
	cfmakeraw(&tTermiosOptions);
	tTermiosOptions.c_cflag |= CLOCAL;
	tTermiosOptions.c_cflag |= CREAD;
	tTermiosOptions.c_cflag |= B115200 ;

	tTermiosOptions.c_lflag &= ~ECHOE;
	tTermiosOptions.c_oflag |= HUPCL;
	tTermiosOptions.c_lflag &= ~ISIG;
	tTermiosOptions.c_lflag &= ~ICANON;
	tTermiosOptions.c_lflag &= ~IEXTEN;
	tTermiosOptions.c_lflag &= ~ECHO;
	tTermiosOptions.c_lflag |= ECHOK;
	tTermiosOptions.c_lflag &= ~ECHONL;
	tTermiosOptions.c_lflag &= ~NOFLSH;
	tTermiosOptions.c_lflag &= ~XCASE;
	tTermiosOptions.c_lflag &= ~TOSTOP;
	tTermiosOptions.c_lflag &= ~ECHOPRT;
	tTermiosOptions.c_lflag |=ECHOCTL;
	//neu werte
	tTermiosOptions.c_cflag &=~CRTSCTS ;
	//neue werte
	tTermiosOptions.c_cc[VTIME] = 0;
	tTermiosOptions.c_cc[VMIN] = 7; // blocking until 7 charse are received i.e. length of header of arduino frame
	// send setting to serial port
	cfmakeraw(&tTermiosOptions);
	tcflush( this->portReference, TCIFLUSH );
	if (tcsetattr(this->portReference, TCSAFLUSH, &tTermiosOptions) == -1){
		printf("Error (SerialDevice::Init): Set Attributes termios\t\t\x1B[31m [FAILED] \033[0m\n");
		exit(0);
	}


//	int RTS_flag;
//	RTS_flag = TIOCM_RTS;
////	ioctl(this->portReference,TIOCMBIS,&RTS_flag);
//	ioctl(this->portReference,TIOCMBIC,&RTS_flag);//Clear RTS pin
//	int DTR_flag;
//	DTR_flag=TIOCM_DTR;
//	ioctl(this->portReference,TIOCMBIS,&DTR_flag);
	int timeout=1000;
	int counter=0;
	while((this->ARD_ADRESS==ARD_ADDRESS_UNKNOWN) && (counter<timeout)){
		tArduinoSensorFrame m_oReceivedFrame;
//		printf("Read Frame\n");
		this->readFrame(m_oReceivedFrame);
		counter++;
	}

}

int SerialDevice::BytesAvailableInput()
{
	int _ibytenumber;
	 ioctl(this->portReference, FIONREAD, &_ibytenumber);
	 if(_ibytenumber>30){
		 printf("Info (SerialDevice::BytesAvailableInput) > 30 \t\t\t\x1B[33m [INFO] \033[0m\n");
	 }
	return _ibytenumber;
}
int SerialDevice::BytesAvailableOutput()
{
	int _obytenumber;
	ioctl(this->portReference, TIOCOUTQ, &_obytenumber);
	return _obytenumber;
}

void SerialDevice::readFrame(tArduinoSensorFrame& m_oReceivedFrame){
//	memset(&m_oReceivedFrame,0xFF,sizeof(m_oReceivedFrame));

	//a few pointers for comfortable access
	tUInt8 *header = (tUInt8*) &m_oReceivedFrame.sSensorHeader;
	tUInt8 *data = (tUInt8*) &m_oReceivedFrame.sData;

	int bytesRead = 0;
	int bytesToRead = 1;

	// some of the arduino don`t send data at all, so the receive thread will be quited here
	if(this->BytesAvailableInput()<(int)sizeof(m_oReceivedFrame))
	{
//		printf("No Bytes available\n");
	}

	//Search for SOF
	do
	 {
		int ret = read(this->portReference,&header[0], bytesToRead);

		if(ret < 0){
//			printf("No Data received\n");
		}
		bytesRead += ret;
		//Message only at second try, because of LOG spamming
		if(bytesRead == 2){
			printf("SOF not found, try again..\n");
		}
	 } while(header[0] != ID_ARD_SOF);

	bytesToRead = sizeof(tArduinoSensorHeader) - sizeof(ID_ARD_SOF);
	bytesRead = 0;

	//Read Header
	do
	 {
		tInt32 ret =  read(this->portReference,&header[sizeof(ID_ARD_SOF)], bytesToRead - bytesRead);
		if(ret < 0){
			printf("Error (readFrame) while reading Header\t\t\x1B[31m [FAILED] \033[0m\n");
		}
		bytesRead += ret;
	 } while(bytesRead < bytesToRead);

	//Check ID
	if(!isSensorIdValid(m_oReceivedFrame.sSensorHeader.ui8ID))
	{

	}
	else
	{
		tUInt8 len =  m_oReceivedFrame.sSensorHeader.ui8DataLength;
		if(len > sizeof(tArduinoSensDataUnion) || len == 0)
		{
			printf("Error (readFrame) Serial read Length mismatch\t\t\t\x1B[31m [FAILED] \033[0m\n");
		}
	 else
		{
		bytesToRead = len;
		bytesRead = 0;
		//Read Data
		do
			{
				tInt32 ret = read(this->portReference,&data[bytesRead], bytesToRead - bytesRead);
				if(ret < 0){
					printf("Error (readFrame) Serial Read Timeout\t\t\x1B[31m [FAILED] \033[0m\n");
				}
				bytesRead += ret;
			} while(bytesRead < bytesToRead);

		bytesToRead = sizeof(INIT_CRC);
		bytesRead = 0;

		//Read CRC
		do
			{
				tInt32 ret = read(this->portReference,&m_oReceivedFrame.ui16CRC, bytesToRead - bytesRead);
				if(ret < 0){
					printf("Error (readFrame) Read Timeout\t\t\x1B[31m [FAILED] \033[0m\n");
				}
				bytesRead += ret;
			} while(bytesRead < bytesToRead);


		//Calc CRC
		tUInt16 crc = fletcher16(header,sizeof(tArduinoSensorHeader) + len);
		if(m_oReceivedFrame.ui16CRC != crc)
			{
			printf("Error (readFrame) CRC Error %x != %x\t\t\t\x1B[31m [FAILED] \033[0m\n",m_oReceivedFrame.ui16CRC,crc);
			}
		else
			{
			//is valid message a arduino error messages
			CheckForArduinoErrorMessage(m_oReceivedFrame);

			//Update Arduino Address
			DetectArduinoAddress(m_oReceivedFrame);

			}
		}
	}
//	this->flush();
}


bool SerialDevice::isSensorIdValid(tUInt8 ui8Id){
	bool returnvalue=true;
//	printf("ID: %d\t%d\t%d\t%d\t%d\t%d\n",ui8Id,ID_ARD_SENS_STEERING,ID_ARD_SENS_WHEEL_RIGHT,
//			ID_ARD_SENS_WHEEL_LEFT,ID_ARD_SENS_IMU,ID_ARD_SENS_US_FRONT_LEFT);
//	printf("Test: %d\n",ui8Id-ID_ARD_SENS_WHEEL_RIGHT);
	switch(ui8Id)
    {
        case ID_ARD_SENS_STEERING:

        case ID_ARD_SENS_WHEEL_RIGHT:
        	break;
        case ID_ARD_SENS_WHEEL_LEFT:
        	break;
        case ID_ARD_SENS_IMU:
        	break;
        case ID_ARD_SENS_US_FRONT_LEFT:
        	break;
        case ID_ARD_SENS_US_FRONT_RIGHT:
        	break;
        case ID_ARD_SENS_US_FRONT_CENTER:
        	break;
        case ID_ARD_SENS_US_FRONT_CENTER_LEFT :
        	break;
        case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:
        	break;
        case ID_ARD_SENS_US_SIDE_LEFT:
        	break;
        case ID_ARD_SENS_US_SIDE_RIGHT:
        	break;
        case ID_ARD_SENS_US_BACK_LEFT:
        	break;
        case ID_ARD_SENS_US_BACK_RIGHT:
        	break;
        case ID_ARD_SENS_US_BACK_CENTER:
        	break;
        case ID_ARD_SENS_VOLT_SPEEDCONT:
        	break;
        case ID_ARD_SENS_VOLT_MEAS:
        	break;
        case ID_ARD_SENS_CURR_SPEED:
        	break;
        case ID_ARD_SENS_CURR_STEER:
        	break;
        case ID_ARD_SENS_RESERVED_FRONT:
        	break;
        case ID_ARD_SENS_RESERVED_BACK:
        	break;
        case ID_ARD_SENSOR_INFO:
        	break;
        case ID_ARD_SENS_ERROR:
        	break;
        default:
        	returnvalue=false;
        	printf("Error (isSensorIdValid) in ID\t\t\t\t\t\x1B[31m [FAILED] \033[0m\n");
        	break;
    }
    return returnvalue;
}


void SerialDevice::DetectArduinoAddress(tArduinoSensorFrame& m_oReceivedFrame){
	if(m_oReceivedFrame.sSensorHeader.ui8ID == ID_ARD_SENSOR_INFO){
		tUInt8 m_ArduinoAddress=m_oReceivedFrame.sData.sInfoData.ui8ArduinoAddress;
		std::string arduinoName="";
		switch(m_ArduinoAddress){
			case ARD_ADDRESS_SENSORSFRONT:
				arduinoName = "SensorsFront";
				this->ARD_ADRESS=ARD_ADDRESS_SENSORSFRONT;
				break;
			case ARD_ADDRESS_SENSORSBACK:
				arduinoName = "SensorsBack";
				this->ARD_ADRESS=ARD_ADDRESS_SENSORSBACK;
				break;
			case ARD_ADDRESS_SENSORSSIDE:
				arduinoName = "SensorsSide";
				this->ARD_ADRESS=ARD_ADDRESS_SENSORSSIDE;
				break;
			case ARD_ADDRESS_ACTORS:
				arduinoName = "Actors";
				this->ARD_ADRESS=ARD_ADDRESS_ACTORS;
				break;
			default:
				arduinoName = "UNKNOWN";
				this->ARD_ADRESS=ARD_ADDRESS_UNKNOWN;
				break;
		};
		this->ARD_NAME=arduinoName;
// 		printf("Found Arduino %d\n",this->ARD_ADRESS);
	}

}


void SerialDevice::CheckForArduinoErrorMessage(tArduinoSensorFrame& m_oReceivedFrame){
    if(m_oReceivedFrame.sSensorHeader.ui8ID == ID_ARD_SENS_ERROR){
        tErrorData error = m_oReceivedFrame.sData.sErrorData;
        switch(error.ui16ErrorNr){
        case ERROR_SOF_NOT_FOUND:
            printf("Error (CheckForArduinoErrorMessage) ERROR_SOF_NOT_FOUND\t\t\t\x1B[33m [INFO] \033[0m\n");
            break;
        case ERROR_CRC_INVALID:
            printf("Error (CheckForArduinoErrorMessage) ERROR_CRC_INVALID\t\t\t\x1B[33m [INFO] \033[0m\n");
            break;
        case ERROR_REMOTE_DETECTED:
//            printf("Error (CheckForArduinoErrorMessage) ERROR_REMOTE_DETECTED\n");
            this->ARD_ADRESS=ARD_ADDRESS_ACTORS;
            break;
        case ERROR_NO_GYRO_DETECTED:
            printf("Error (CheckForArduinoErrorMessage) ERROR_NO_GYRO_DETECTED\t\t\t\x1B[33m [INFO] \033[0m\n");
            break;
        default:
            printf("Error (CheckForArduinoErrorMessage) UNKNOWN ERROR NR %i\t\t\t\x1B[33m [INFO] \033[0m\n",error.ui16ErrorNr);
        }

    }
}

tUInt16  SerialDevice::fletcher16( tUInt8* data, tUInt8 bytes )
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

void SerialDevice::flush(){
	tcflush(this->portReference, TCIOFLUSH);
}


void SerialDevice::writeFrame(tUInt8 ui8Id, tUInt8 ui8Data){
	if(this->ARD_ADRESS != ARD_ADDRESS_ACTORS){
		//don t send data to sensors
		printf("Error (writeFrame) Wrong Adress 2 send Actorframe\t\t\x1B[31m [FAILED] \033[0m\n");
	}
	else
	{
		tUInt8 *buffer;
	    tArduinoActorFrame frame;
	    memset(&buffer,0,sizeof(frame));
	    buffer = (tUInt8*) &frame;

	    frame.sActorHeader.ui8SOF = ID_ARD_SOF;
	    frame.sActorHeader.ui8ID = ui8Id;
	    frame.sData = ui8Data;
	    frame.ui16CRC = fletcher16(buffer,sizeof(tArduinoActorFrame) - sizeof(INIT_CRC));

	    if(write(this->portReference,&buffer, sizeof(tArduinoActorFrame)) != sizeof(tArduinoActorFrame))
		{
			printf("Error (writeFrame) while writing Actorframe\t\t\x1B[31m [FAILED] \033[0m\n");
		}
	    if(syncfs(this->portReference==-1)){
	    	printf("Error (writeFrame) Sync failed! \t\t\t\t\x1B[31m [FAILED] \033[0m\n");
	    }
	    if(BytesAvailableOutput()>=20480){
	    	printf("Error (writeFrame) Buffer Overflow! \t\t\t\t\x1B[31m [FAILED] \033[0m\n");
	    }
	    tcdrain(this->portReference);
	    fsync(this->portReference);
	}
}
void SerialDevice::writeFrame(tUInt8 ui8Id){
	if(this->ARD_ADRESS != ARD_ADDRESS_ACTORS){
		//don t send data to sensors
		printf("Error (writeFrame) Wrong Adress 2 send Actorframe\t\t\x1B[31m [FAILED] \033[0m\n");
	}
	else
	{
		tUInt8 *buffer;
	    tArduinoActorFrame frame;
	    memset(&buffer,0,sizeof(frame));
	    buffer = (tUInt8*) &frame;

	    frame.sActorHeader.ui8SOF = ID_ARD_SOF;
	    frame.sActorHeader.ui8ID = ui8Id;
//	    frame.sData = ui8Data;
	    frame.ui16CRC = fletcher16(buffer,sizeof(tArduinoActorFrame) - sizeof(INIT_CRC));

	    if(write(this->portReference,&ui8Id, sizeof(ui8Id)) != sizeof(ui8Id))
		{
			printf("Error (writeFrame) while writing Actorframe\t\t\x1B[31m [FAILED] \033[0m\n");
		}
	    if(syncfs(this->portReference==-1)){
	    	printf("Error (writeFrame) Sync failed! \t\t\t\t\x1B[31m [FAILED] \033[0m\n");
	    }
	    if(BytesAvailableOutput()>=20480){
	    	printf("Error (writeFrame) Buffer Overflow! \t\t\t\t\x1B[31m [FAILED] \033[0m\n");
	    }
	    tcdrain(this->portReference);
	    fsync(this->portReference);
	}
}

} /* namespace Serial */
