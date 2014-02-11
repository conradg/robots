import curses
import week1

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.keypad(1)


def turnLeft():
    print "turning left"
    week1.turn_cw(-1)


def turnRight():
    print "turning right"
    week1.turn_cw(1)


def goForwards():
    print "going forwards"
    week1.go(1)


def goBackwards():
    print "going backwards"
    week1.go(-1)


while 1:
    c = stdscr.getch()
    if c == curses.KEY_UP:
        goForwards()
    elif c == curses.KEY_DOWN:
        goBackwards()
    elif c == curses.KEY_LEFT:
        turnLeft()
    elif c == curses.KEY_RIGHT:
        turnRight()
    elif c == curses.KEY_BACKSPACE:
        break
    else:
        print "waiting"
