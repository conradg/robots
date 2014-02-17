from movement import *
import time
import random

mu = 0 # mean (no error)
sigma = 50 #standard dev (possible error)

x = 0
y = 0
theta = 0

def getRandomError():
    return random.gauss(mu, sigma)

def pointCloud():
    x = 40
    y = 440
    theta = 90
    numberOfParticles = 10

    side1 = (40, 40, 440, 40)    # (x0, y0, x1, y1)
    side2 = (440, 40, 440, 440)  # (x0, y0, x1, y1)
    side3 = (440, 440, 40, 440)  # (x0, y0, x1, y1)
    side4 = (40, 440, 40, 40)    # (x0, y0, x1, y1)

    print "drawLine:" + str(side1)
    print "drawLine:" + str(side2)
    print "drawLine:" + str(side3)
    print "drawLine:" + str(side4)

    i = 0
    particles = [(x,y,theta) for particle in range(numberOfParticles)]
    for particle in range(numberOfParticles):
        turning = False
        i += 100
        if i == 100:
            turning = True
        if i == 200:
            turning = False

        if turning:
            thetaChange = 90
            x = x
            y = y
            theta = theta + thetaChange + getRandomError()
        else :
            xChange = 10
            yChange = 10
            x = x + (xChange + getRandomError()) * math.cos(theta)
            y = y + (yChange + getRandomError()) * math.sin(theta)
            theta = theta + getRandomError()
        # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    print "drawParticles:" , particles
    time.sleep(0.1)


def goTo (xnew,ynew):
    global x
    global y
    global theta
    xdiff = xnew - x
    ydiff = ynew - y
    angle  = math.atan2(ydiff,xdiff) * (180/math.pi)
    anglediff = angle - theta

    while anglediff > 180: anglediff -=360
    while anglediff < -180: anglediff +=360

    print "angle:" ,  angle
    distance  = math.sqrt(xdiff**2 + ydiff**2) * 100 # *100 to convert to cm
    turn_cw(-anglediff)
    go(distance)
    x = xnew
    y = ynew

    while angle > 180: angle -= 360
    while angle < -180: angle += 360

    theta = angle

pointCloud()
