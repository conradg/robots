from week1 import *
from week2 import *
import time
import sys
import random

mu = 0 # mean (no error)
sigma = 15 #standard dev (possible error)

x = 0
y = 0
theta = 0

def getRandomError():
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

i = 0

while True:
	turning = false
	i += 1
	if i = 100: 
		turning = true
	if i = 150:
		turning = false
	
	if turning:
		thetaChange = 90 #getThetaChange();
		x = x;
		y = y;
		theta = theta + thetaChange + getRandomError();
	else:	xChange = 10 #getXChange();
		yChange = 10 #getYChange();
		x = x + (xChange + getRandomError()) * cos(theta);
		y = y + (xchange + getRandomError()) * sin(theta);
		theta = theta + getRandomError();


	# Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
	particles = [(x, y, theta) for i in range(numberOfParticles)]
	print "drawParticles:" + str(particles)

	time.sleep(0.05)
