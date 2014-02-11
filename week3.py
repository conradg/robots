from week1 import *
from week2 import *
import time
import sys
import random

mu = 5 # mean
sigma = 3 #standard dev

def getRandomError()
    return random.gauss(mu, sigma)

numberOfParticles = 100

side1 = (40, 40, 440, 40) # (x0, y0, x1, y1)
side2 = (440, 40, 440, 440)  # (x0, y0, x1, y1)
side3 = (440, 440, 40, 440) # (x0, y0, x1, y1)
side4 = (440, 40, 40, 40)  # (x0, y0, x1, y1)

print "drawLine:" + str(side1)
print "drawLine:" + str(side2)
print "drawLine:" + str(side1)
print "drawLine:" + str(side2)

while True:
	# Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
	particles = [(getRandomX(), getRandomY(), getRandomTheta()) for i in range(numberOfParticles)]
	print "drawParticles:" + str(particles)

	c += 1;
	time.sleep(0.05)
