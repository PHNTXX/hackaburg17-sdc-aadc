/*
 * ArduinoComunicator.h
 *
 *  Created on: Feb 10, 2017
 *      Author: WagPhi
 */

#ifndef INCLUDE_ARDUINOCOMUNICATOR_H_
#define INCLUDE_ARDUINOCOMUNICATOR_H_
#include <iostream>
#include <cstdio>
#include <stdint.h>
#include "Serial/SerialDevice.h"
#include <thread>
#include <mutex>
#include <queue>
#include "Buffer.h"
#include "SensorDataStruct.h"
#include "ActorDataStruct.h"
#include <time.h>


namespace PITX {

class ArduinoComunicator {
private:
	Serial::SerialDevice ard_actor;
	Serial::SerialDevice ard_front;
	Serial::SerialDevice ard_side;
	Serial::SerialDevice ard_back;

	bool allARDsReady();
	void working();

	static const uint8_t number_of_threads = 4;

	/**
	 * @brief      The producer thread. It waits for incoming data from the serial device and pushing it into a queue. It wakes up the corresponding consumer thread via a conditional variable (C++11).
	 *
	 * @param      dev   The device this producer waits for data.
	 * @param[in]  id    The identifier for accessing elements in the various arrays.
	 */
	void producer_thread(Serial::SerialDevice &dev, uint8_t id);
	/**
	 * @brief      Consumes the data on the queue  until there are no more data on the queue. The thread will be woken up by the producer and writing the data into a file, holding
	 * 			   the #file_mutex!
	 *
	 * @param[in]  id    The identifier for accessing elements in the various arrays.
	 */
	void consumer_thread(uint8_t id);

	/// array of producer threads, at the moment there are four of these
	std::array<std::thread, number_of_threads> consumers;
	/// array of consumer threads, there should be a unique pair of consumer/producer threads
	std::array<std::thread, number_of_threads> producers;
	/// an array of queues for saving data uniqueless for every consumer/producer pair
	std::array<std::queue<tArduinoSensorFrame>, number_of_threads> queues;
	/// the mutexs for the queues, so there is no push/pull at the same time
	std::array<std::mutex, number_of_threads> mutexs;
	/// the conditional variables to sleep the consumer threads
	std::array<std::condition_variable, number_of_threads> conditions;
	/// a little trick we use for making the queue access clear and avoiding deadlocks
	std::array<bool, number_of_threads> run_commands;

	/// the mutex for the csv file we could generate where all sensor data will be stored.
	std::mutex file_mutex;
	static bool cancellation;
	tSensorDataArdComunicator sensor_data;
	void generateData(tArduinoSensorFrame& frame, tSensorDataArdComunicator& actor_data);

	//ActorSending
	tActorDataArdComunicator actor_data;

	void setActorData(uint8_t ID, uint8_t data);
	std::mutex mx_actor_data;
	std::condition_variable cv_actor_data;
	int unsentFrame();
	void send_actor_data_ard();


	void controlcircuit();

	static void LinuxSignalCatcher(int signal);

	static long get_millis(void);

	FILE * pFile;


	bool debugbool;


public:
	ArduinoComunicator();
	~ArduinoComunicator();
	void cancelsensorread();
	tSensorDataArdComunicator gettSensorDataArdComunicator(){return sensor_data;};
	void settActorDataArdComunicator();
};

} /* namespace PITX */

#endif /* INCLUDE_ARDUINOCOMUNICATOR_H_ */
