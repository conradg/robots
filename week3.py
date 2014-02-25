import time
import random
import math
from particleDataStructures import *

mu = 0 # mean (no error)
sigmaDist = 0.5 #standard dev (possible error) for distance
sigmaAngle = 0.005 #standard dev (possible error) for angle on forward movement
sigmaTurn = math.pi/360 #standard dev (possible error) for angle on turn

def getRandomErrorDist():
    return random.gauss(mu, sigmaDist)

def getRandomErrorAngle():
    return random.gauss(mu, sigmaAngle)

def getRandomErrorTurn():
    return random.gauss(mu, sigmaTurn)

def recalculateParticleCloud(particles, d, dtheta):
    for i in range(len(particles)):
        x, y, theta, weight = particles[i]
        # print "position: " , x , y 
        if not (dtheta==0) :
            print dtheta
            theta = theta + dtheta + getRandomErrorTurn()
        else:
            distanceError = getRandomErrorDist()
            x = x + (d + distanceError)* math.cos(theta)
            y = y + (d + distanceError)* math.sin(theta)
            theta = theta + getRandomErrorAngle()
        particles[i] = (x, y, theta, weight)
    return particles

def drawNewParticleCloud(particles, d, dtheta):
    #print "drawing particles: ", particles[0]
    particles = recalculateParticleCloud(particles, d, dtheta)
    canvas.drawParticles(particles)
    return particles

