import time
import serial

ser0 = serial.Serial('/dev/ttyACM0', 115200)

def rf():
    ser0.write(b'T' + bytes(130) + b'n')
    ser0.write(b'S' + bytes(75) + b'n')

def lb():
    ser0.write(b'T' + bytes(48) + b'n')
    ser0.write(b'S' + bytes(110) + b'n')

def ter():
    ser0.write(b'T' + bytes(90) + b'n')
    ser0.write(b'S' + bytes(90) + b'n')

time.sleep(1.5)
rf()
time.sleep(1.5)
lb()
time.sleep(1.5)
rf()
time.sleep(1.5)
lb()
time.sleep(1.5)
ter()
