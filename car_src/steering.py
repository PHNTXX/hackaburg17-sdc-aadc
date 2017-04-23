import curses
import os
import time

key = ''
while 1:
    key = curses.initscr().getch()
    if key == 115: 	#back	 s	8
	for i in range(7):
		os.system("echo '8' > /dev/ttyACM0")
    elif key == 119: 	#forward w	7
	for i in range(10):
		os.system("echo '7' > /dev/ttyACM0")
    elif key == 97: 	#left 	 a	4
	for i in range(10):
        	os.system("echo '4' > /dev/ttyACM0")
    elif key == 100: 	#right 	 d	6
	for i in range(10):
        	os.system("echo '6' > /dev/ttyACM0")

    elif key == 101: 	#right 	 e	0
        	os.system("echo '0' > /dev/ttyACM0")
    elif key == 57: 	#right 	 9	9
        	os.system("echo '0' > /dev/ttyACM0")

	#difference between forward and backward is 3
	#backwards needs +3


    elif key == 121: 	 #experiment
	for i in range(15):
		os.system("echo '4' > /dev/ttyACM0")
	#for i in range(7):
		#os.system("echo '7' > /dev/ttyACM0")
	time.sleep(1.5)
	os.system("echo '0' > /dev/ttyACM0")

	time.sleep(0.5)

	for i in range(10):
		#os.system("echo '8' > /dev/ttyACM0")
		os.system("echo '6' > /dev/ttyACM0")
	time.sleep(1.5)

	os.system("echo '0' > /dev/ttyACM0")

	for i in range(15):
		os.system("echo '6' > /dev/ttyACM0")


    else:
        os.system("echo '0' > /dev/ttyACM0")
	curses.endwin()
	quit()
