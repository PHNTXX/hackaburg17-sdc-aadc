import serial
import time
import struct
import sys
import os
import traceback

ID_ARD_SOF = b'\x55'

ID_ARD_SENS_US_FRONT_LEFT = b'\x41'
ID_ARD_SENS_US_FRONT_CENTER_LEFT = b'\x42'
ID_ARD_SENS_US_FRONT_CENTER_RIGHT = b'\x43'
ID_ARD_SENS_US_FRONT_RIGHT = b'\x44'
ID_ARD_SENS_US_FRONT_CENTER = b'\x45'
ID_ARD_SENS_US_SIDE_LEFT = b'\x46'
ID_ARD_SENS_US_SIDE_RIGHT = b'\x47'
ID_ARD_SENS_US_BACK_LEFT = b'\x48'
ID_ARD_SENS_US_BACK_RIGHT = b'\x49'
ID_ARD_SENS_US_BACK_CENTER = b'\x4A'

ser0 = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser1 = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

SPEED_SCALE = 0.25

sensor_vals_by_id = {}
speed = 90
steering = 90
steering_delta = -1
raw_speed = -1

def mean(numbers):
        return float(sum(numbers)) / max(len(numbers), 1)

def writeValues():
        global speed
        global steering
        global raw_speed
        global steering_delta

        ser0.write(b'S' + bytes(speed) + b'n')
        print 'speed=', speed, 'raw_speed=', raw_speed

        steering = 90 - steering_delta
        #ser0.write(b'T' + bytes(steering) + b'n')
        print 'steering=', steering, 'steering_delta=', steering_delta

def update():
        global sensor_vals_by_id
        global speed
        global steering
        global raw_speed
        global steering_delta

        if ord(ID_ARD_SENS_US_FRONT_CENTER) in sensor_vals_by_id:
                sensor_value = sensor_vals_by_id[ord(ID_ARD_SENS_US_FRONT_CENTER)]
                raw_speed = int(90 - SPEED_SCALE * sensor_value + 20)
                speed = raw_speed
                if speed > 105:
                        speed = 105
                if speed < 75:
                        speed = 75

        front_sensor_vals = [
                ord(ID_ARD_SENS_US_FRONT_LEFT),
                ord(ID_ARD_SENS_US_FRONT_CENTER_LEFT),
                ord(ID_ARD_SENS_US_FRONT_CENTER),
                ord(ID_ARD_SENS_US_FRONT_CENTER_RIGHT),
                ord(ID_ARD_SENS_US_FRONT_RIGHT)
        ]

        steering_delta = 0
        smallest_sensor_val = min(front_sensor_vals)
        if smallest_sensor_val == ord(ID_ARD_SENS_US_FRONT_LEFT):
                steering_delta = -5
        if smallest_sensor_val == ord(ID_ARD_SENS_US_FRONT_CENTER_LEFT):
                steering_delta = -3
        if smallest_sensor_val == ord(ID_ARD_SENS_US_FRONT_CENTER):
                steering_delta = 0
        if smallest_sensor_val == ord(ID_ARD_SENS_US_FRONT_CENTER_RIGHT):
                steering_delta = 3
        if smallest_sensor_val == ord(ID_ARD_SENS_US_FRONT_RIGHT):
                steering_delta = 5

        writeValues()
	print '\ninw=', ser1.inWaiting()

def read():
	global ser1

        first_byte = ser1.read(1)
        if first_byte != ID_ARD_SOF:
                return

        (sof, sensor_id, arduino_ts, data_len) = struct.unpack('=bbib', first_byte + ser1.read(6))

        if sensor_id not in [
                ord(ID_ARD_SENS_US_FRONT_LEFT),
                ord(ID_ARD_SENS_US_FRONT_CENTER_LEFT),
                ord(ID_ARD_SENS_US_FRONT_CENTER_RIGHT),
                ord(ID_ARD_SENS_US_FRONT_RIGHT),
                ord(ID_ARD_SENS_US_FRONT_CENTER),
                ord(ID_ARD_SENS_US_SIDE_LEFT),
                ord(ID_ARD_SENS_US_SIDE_RIGHT),
                ord(ID_ARD_SENS_US_BACK_LEFT),
                ord(ID_ARD_SENS_US_BACK_RIGHT),
                ord(ID_ARD_SENS_US_BACK_CENTER)
        ]:
                return
	
	if sensor_id != ord(ID_ARD_SENS_US_FRONT_CENTER):
		return

        (sensor_value,) = struct.unpack('=H', ser1.read(data_len))
        sensor_vals_by_id[sensor_id] = sensor_value

        print sof, sensor_id, arduino_ts, data_len, 'sv=', sensor_value
        update()

def start():
	global ser1

	try:
        	while True:
                	read()
			#if ser1.inWaiting() > 100:
				#ser1.close()
				#raise Exception("flooded")
				#ser1.close()
				#time.sleep(2)
				#ser1 = serial.Serial('/dev/ttyACM1', 115200)
			#	break
		os._exit(1)
				
	except:
		print(traceback.format_exc())

        	speed = 90
        	steering = 90
        	writeValues()

        	print 'Interrupted, quitting'
        	os._exit(1)

start()
