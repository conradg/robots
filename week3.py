import time
import random
import math

mu = 0 # mean (no error)
sigmaDist = 0.5 #standard dev (possible error) for distance
sigmaAngle = 0.005 #standard dev (possible error) for angle on forward movement
sigmaTurn = math.pi/36 #standard dev (possible error) for angle on turn

DISPLAY_SQUARE_MARGIN = 100
DISPLAY_SQUARE_SIDE   = 500
PHYSICAL_SQUARE_SIDE  = 40

def getRandomErrorDist():
    return random.gauss(mu, sigmaDist)

def getRandomErrorAngle():
    return random.gauss(mu, sigmaAngle)

def getRandomErrorTurn():
    return random.gauss(mu, sigmaTurn)

def recalculatePointCloud(particles, accd, dtheta):
    d = (DISPLAY_SQUARE_SIDE * accd) / PHYSICAL_SQUARE_SIDE
    for i in range(len(particles)):
        x, y , theta = particles[i]
        if dtheta :
            theta = theta + dtheta + getRandomErrorTurn()
        else:
            x = x + (d + getRandomErrorDist()) * math.cos(theta)
            y = y - (d - getRandomErrorDist()) * math.sin(theta)
            theta = theta + getRandomErrorAngle()
        particles[i] = (x,y,theta)
    return particles

def drawNewPointCloud(particles, d, dtheta):
    particles = recalculatePointCloud(particles, d, dtheta)
    print "drawParticles:"  + str(particles)
    time.sleep(0.1)
    return particles

