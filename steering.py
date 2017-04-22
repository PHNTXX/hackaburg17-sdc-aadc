import curses
import os

key = ''
while 1:
    key = curses.initscr().getch()

    if key == curses.KEY_UP or key == 119:
        print("FORWARD")
    elif key == curses.KEY_DOWN or key == 115:
        print("BACK")
    elif key == curses.KEY_LEFT or key == 97:
        print("LEFT")
        os.system("echo '4' > /dev/ttyACM0")
    elif key == curses.KEY_RIGHT or key == 100:
        print("RIGHT")
        os.system("echo '6' > /dev/ttyACM0")
    else:
        os.system("echo '5' > /dev/ttyACM0")
        curses.endwin()
        quit()
