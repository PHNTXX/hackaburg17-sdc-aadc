import serial
import time
import struct

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

ser0 = serial.Serial('/dev/ttyACM0', 115200)
ser1 = serial.Serial('/dev/ttyACM1', 115200)

recent_vals = []

SPEED_SCALE = 0.25

def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

while True:
        first_byte = ser1.read(1)

        if first_byte != ID_ARD_SOF:
                continue

        header = first_byte + ser1.read(6)
        (sof, sensor_id, arduino_ts, data_len) = struct.unpack('=bbib', header)

        if sensor_id != ord(ID_ARD_SENS_US_FRONT_CENTER):
                continue

        data_bytes = ser1.read(data_len)
        (sensor_value,) = struct.unpack('=H', data_bytes)

	# average sensor readings
	recent_vals = recent_vals + [sensor_value]
	averaged_sensor_value = mean(recent_vals)
	if len(recent_vals) > 2:
		recent_vals.pop(0)

	speed = int(90 - SPEED_SCALE * averaged_sensor_value + 20)
	raw_speed = speed
        if speed > 110:
            speed = 110
        if speed < 70:
            speed = 70
	
	ser0.write(b'S' + bytes(speed) + b'n')

        print sof, sensor_id, arduino_ts, data_len, 'sv=', sensor_value, 'avg_sv=', averaged_sensor_value, 'spd=', speed, 'raw_spd=', raw_speed

ser0.close()
ser1.close()
