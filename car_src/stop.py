import serial
import time
import struct
import sys
import os
import traceback

ser0 = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser0.write(b'S90n')
ser0.write(b'T90n')

