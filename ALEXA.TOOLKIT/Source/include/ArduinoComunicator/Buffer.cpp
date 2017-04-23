/*
 * Buffer.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: WagPhi
 */

#include "Buffer.h"

namespace PITX {

Buffer::Buffer() {
	// TODO Auto-generated constructor stub

}

Buffer::~Buffer() {
	// TODO Auto-generated destructor stub
}




void Buffer::add_ard_front_frame(tArduinoSensorFrame frame) {
	while (true) {
		std::unique_lock<std::mutex> locker(_mu_front);
		_cond_front.wait(locker, [this](){return _buffer_front.size() < _size_front;});
		_buffer_front.push_back(frame);
		locker.unlock();
		_cond_front.notify_all();
		return;
	}
}



tArduinoSensorFrame Buffer::remove_ard_front_frame() {
	while (true)
	{
		std::unique_lock<std::mutex> locker(_mu_front);
		_cond_front.wait(locker, [this](){return _buffer_front.size() > 0;});
		tArduinoSensorFrame back = _buffer_front.back();
		_buffer_front.pop_back();
		locker.unlock();
		_cond_front.notify_all();
		return back;
	}
}
} /* namespace PITX */
