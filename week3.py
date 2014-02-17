from movement import *
import time
import random

mu = 0 # mean (no error)
sigma = 0.5 #standard dev (possible error)

DISPLAY_SQUARE_MARGIN = 50
DISPLAY_SQUARE_SIDE = 500

x = 0
y = 0
theta = 0

def getRandomError():
    return random.gauss(mu, sigma)

def recalculatePointCloud(particles, dx, dy, dtheta):
    out = []
    for particle in particles:
        x, y , theta = particle
        if dtheta :
            theta = theta + dtheta + getRandomError()
        else:
            x = x + (dx + getRandomError()) * math.cos(theta)
            y = y + (dy + getRandomError()) * math.sin(theta)
            theta = theta + getRandomError()
        x = int(x)
        y = int(y)
        out.append((x,y,theta))
    return out


def pointCloud(x,y,theta):
    numberOfParticles = 10

    dsm = DISPLAY_SQUARE_MARGIN
    dss = DISPLAY_SQUARE_SIDE
    side1 = (dsm, dsm, dss+dsm, dsm)    # (x0, y0, x1, y1)
    side2 = (dss+dsm, dsm, dss+dsm, dss+dsm)  # (x0, y0, x1, y1)
    side3 = (dss+dsm, dss+dsm, dsm, dss+dsm)  # (x0, y0, x1, y1)
    side4 = (dsm, dss+dsm, dsm, dsm)    # (x0, y0, x1, y1)

    print "drawLine:" + str(side1)
    print "drawLine:" + str(side2)
    print "drawLine:" + str(side3)
    print "drawLine:" + str(side4)

    pointcloud = [(dsm,dss+dsm,0) for j in range(numberOfParticles)]
    print "drawParticles:" , pointcloud
    for i in range(100):
        pointcloud = recalculatePointCloud(pointcloud,5,0,0)
        print "drawParticles:" , pointcloud
        time.sleep(0.1)
    pointcloud = recalculatePointCloud(pointcloud,0,0,math.pi/2)
    print "drawParticles:" , pointcloud
    time.sleep(0.1)
    for i in range(100):
        pointcloud = recalculatePointCloud(pointcloud,0,-5,0)
        print "drawParticles:" , pointcloud
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

pointCloud(0,0,0)
#goTo(.3,.3)
#goTo(0,0)

