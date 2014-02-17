from movement import *
import time
import random

mu = 0 # mean (no error)
sigmaDist = 0.5 #standard dev (possible error) for distance
sigmaAngle = 0.005 #standard dev (possible error) for angle on forward movement
sigmaTurn = math.pi/36 #standard dev (possible error) for angle on turn

DISPLAY_SQUARE_MARGIN = 50
DISPLAY_SQUARE_SIDE = 500

x = 0
y = 0
theta = 0

def getRandomErrorDist():
    return random.gauss(mu, sigmaDist)

def getRandomErrorAngle():
    return random.gauss(mu, sigmaAngle)

def getRandomErrorTurn():
    return random.gauss(mu, sigmaTurn)

def recalculatePointCloud(particles, d, dtheta):
    out = []
    for particle in particles:
        x, y , theta = particle
        if dtheta :
            theta = theta + dtheta + getRandomErrorTurn()
        else:
            x = x + (d + getRandomErrorDist()) * math.cos(theta)
            y = y - (d - getRandomErrorDist()) * math.sin(theta)
            theta = theta + getRandomErrorAngle()
        out.append((x,y,theta))
    return out

def drawNewPointCloud(pointcloud, d, dtheta)
        pointcloud = recalculatePointCloud(pointcloud, d, dtheta)
        print "drawParticles:"  + str(pointcloud)
        return pointcloud


def pointCloud(x,y,theta):
    numberOfParticles = 100

    dsm = DISPLAY_SQUARE_MARGIN
    dss = DISPLAY_SQUARE_SIDE
    side1 = (dsm, dsm, dss+dsm, dsm)
    side2 = (dss+dsm, dsm, dss+dsm, dss+dsm)
    side3 = (dss+dsm, dss+dsm, dsm, dss+dsm)
    side4 = (dsm, dss+dsm, dsm, dsm)

    print "drawLine:" + str(side1)
    print "drawLine:" + str(side2)
    print "drawLine:" + str(side3)
    print "drawLine:" + str(side4)

    pointcloud = [(dsm,dss+dsm,0) for j in range(numberOfParticles)]
    print "drawParticles:" + str(pointcloud)
    for i in range(100):
        pointcloud = drawNewPointCloud(pointcloud,5,0)
        time.sleep(0.1)
    pointcloud = recalculatePointCloud(pointcloud,0,math.pi/2)
    time.sleep(0.1)
    for i in range(100):
        pointcloud = recalculatePointCloud(pointcloud,5,0)
        print "drawParticles:" + str(pointcloud)
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

