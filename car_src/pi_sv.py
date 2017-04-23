from bottle import route, run, request
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)
more_act = ''

@route('/remote')
def remote():
    try:
        if request.query.dir == 'left':
            ser.write('T70n')
	    more_act = 'left'
        if request.query.dir == 'right':
            ser.write('T110n')
	    more_act = 'right'

        if request.query.spd == 'forward':
            ser.write('S80n')
	    time.sleep(1.5)
	    ser.write('S90n')
	    more_act = 'forward'
        if request.query.spd == 'backward':
            ser.write('S100n')
	    time.sleep(1.5)
	    ser.write('S90n')
	    more_act = 'backward'

        if request.query.cmd == 'stop':
            ser.write('T90n')
            ser.write('S90n')
	    more_act = ''

	if request.query.cmd == 'more':
	    if more_act == 'left':
		ser.write('T70n')
	    if more_act == 'right':
		ser.write('T110n')
	    if request.query.spd == 'forward':
                ser.write('S80n')
                time.sleep(1.5)
                ser.write('S90n')
            if request.query.spd == 'backward':
                ser.write('S100n')
                time.sleep(1.5)
                ser.write('S90n')
    except:
        print "Could not write to serial"
    return 'ok'

run(host='0.0.0.0', port=8700)
