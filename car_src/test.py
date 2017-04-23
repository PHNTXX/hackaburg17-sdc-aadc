import serial
import math

ser0 = serial.Serial('/dev/ttyACM4', 9600)
while 1:

	input = (int(ser0.readline()) + 0.0)
	aboutval = math.floor(input)

	print ser0.readline()

	#if math.floor(float(ser0.readline())) == 200:
	#	print "JACKPOT!"

