/*
 * Buffer.h
 *
 *  Created on: Feb 13, 2017
 *      Author: WagPhi
 */

#ifndef INCLUDE_BUFFER_H_
#define INCLUDE_BUFFER_H_
#include <iostream>
#include <deque>
#include <mutex>
#include <condition_variable>
#include "arduinoProtokol.h"
using std::deque;




namespace PITX {

class Buffer
{
public:

    void add_ard_front_frame(tArduinoSensorFrame frame);
    tArduinoSensorFrame remove_ard_front_frame();

    void add_ard_side_frame(tArduinoSensorFrame frame);
    tArduinoSensorFrame remove_ard_side_frame();

	void add_ard_back_frame(tArduinoSensorFrame frame);
	tArduinoSensorFrame remove_ard_back_frame();

	void add_ard_actor_frame(tArduinoSensorFrame frame);
	tArduinoSensorFrame remove_ard_actor_frame();


    Buffer();
    ~Buffer();
private:
    //FRONT
    std::mutex _mu_front;
    std::condition_variable _cond_front;
    deque<tArduinoSensorFrame> _buffer_front;
    const unsigned int _size_front = 10;

    //SIDE
	std::mutex _mu_side;
	std::condition_variable _cond_side;
	deque<tArduinoSensorFrame> _buffer_side;
	const unsigned int _size_side= 10;


    //BACK
	std::mutex _mu_back;
	std::condition_variable _cond_back;
	deque<tArduinoSensorFrame> _buffer_back;
	const unsigned int _size_back= 10;

    //Actor
	std::mutex _mu_actor;
	std::condition_variable _cond_actor;
	deque<tArduinoSensorFrame> _buffer_actor;
	const unsigned int _size_actor= 10;

};

} /* namespace PITX */

#endif /* INCLUDE_BUFFER_H_ */
