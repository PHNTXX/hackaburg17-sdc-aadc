/*
 * main.cpp
*
 *
 *  Created on: Feb 2, 2017
 *      Author: WagPhi
 */
#include <iostream>
#include "include/ArduinoComunicator/ArduinoComunicator.h"



int main()
{
	tSensorDataArdComunicator data;
    PITX::ArduinoComunicator board;
    data =  board.gettSensorDataArdComunicator();
    return 1;
}
