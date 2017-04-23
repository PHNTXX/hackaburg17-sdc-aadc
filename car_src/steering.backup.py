import curses
import os

key = ''
while 1:
    key = curses.initscr().getch()
    if key == 119:
	for i in range(10):
		os.system("echo '8' > /dev/ttyACM0")
    elif key == 115:
	for i in range(7):
		os.system("echo '7' > /dev/ttyACM0")
    elif key == 97:
        os.system("echo '4' > /dev/ttyACM0")
    elif key == 100:
        os.system("echo '6' > /dev/ttyACM0")
    else:
        os.system("echo '0' > /dev/ttyACM0")
	curses.endwin()
	quit()
