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
    accd = (DISPLAY_SQUARE_SIDE * d) / PHYSICAL_SQUARE_SIDE
    for particle in particles:
        x, y , theta = particle
        if dtheta :
            theta = theta + dtheta + getRandomErrorTurn()
        else:
            x = x + (accd + getRandomErrorDist()) * math.cos(theta)
            y = y - (accd - getRandomErrorDist()) * math.sin(theta)
            theta = theta + getRandomErrorAngle()
        out.append((x,y,theta))
    return out

def drawNewPointCloud(particles, d, dtheta):
    particles = recalculatePointCloud(particles, d, dtheta)
    print "drawParticles:"  + str(particles)
    time.sleep(0.1)
    return particles


#goTo(.3,.3)
#goTo(0,0)

