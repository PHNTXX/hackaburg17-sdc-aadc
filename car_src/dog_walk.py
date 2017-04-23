import sys
import os
import time
import serial

try:
        ser0 = serial.Serial('/dev/ttyACM0', 115200)
        time.sleep(3)
        ser0.write(b'T' + bytes(55) + b'n')
        ser0.write(b'S' + bytes(80) + b'n')
        time.sleep(1.2)
	ser0.write(b'T' + bytes(125) + b'n')
        time.sleep(1.2)
	ser0.write(b'T' + bytes(55) + b'n')
        time.sleep(1.2)
	ser0.write(b'T' + bytes(125) + b'n')
        time.sleep(1.2)
        ser0.write(b'S' + bytes(90) + b'n')
	time.sleep(55555)
except KeyboardInterrupt:
        ser0.write(b'T' + bytes(90) + b'n')
        ser0.write(b'S' + bytes(90) + b'n')
        
        print 'Interrupted'
        try:
                sys.exit(0)
        except SystemExit:
                os._exit(0)
