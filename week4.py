import math
import random

WAYPOINT_TOLERANCE = 4 #so the robot doesn't try to get infinitely nearer to the waypoint
SONAR_SIGMA = 3 #TODO this needs a real value
K = 0.005
NUMBER_OF_PARTICLES = 100
particleCloud = 0

#simpleWalls format [start, horizontal, start, finish]
simpleWalls = [(0, False, 0, 168), (168, True, 0,84), (84, False, 126,168), (210, True, 84,168), (168, False,84,210), (84, True, 168, 210), (210, False, 0, 84), (0, True, 0, 210)]

#from conrad, not used yet
point_dict = {
    'O': (0,0),
    'A': (0,168),
    'B': (84,168),
    'C': (84,126),
    'D': (84,210),
    'E': (168,210),
    'F': (168,84),
    'G': (210,84),
    'H': (210,0),
}

def resetParticleCloud():
    print 'resetting particle cloud'
    start_x = 30#150 #use 30
    start_y = 30 #use 30
    start_theta = 0 #use 0
    global particleCloud
    particleCloud = [(start_x + random.gauss(0,3), start_y + random.gauss(0, 3),start_theta,0.01) for j in range(NUMBER_OF_PARTICLES)]
    #for k in range(1, 10):
        #for j in range(1,10):
            #particleCloud[(k-1)*10 + (j-1)] = (j * 19, k * 19, start_theta, 0.01)


def getMeanPosition():
    meanX = 0
    meanY = 0
    meanTheta = 0
    for i in range(NUMBER_OF_PARTICLES):
	xi, yi, thetai, weighti = particleCloud[i]
        meanX += xi * weighti
	meanY += yi * weighti
	meanTheta += thetai * weighti
    return (meanX, meanY, meanTheta)


def resample():
    global particleCloud
    length = len(particleCloud)
    cumalativeWeights = [0 for j in range(length)]
    for i in range(length):
        if(i == 0):
            cumalativeWeights[0] = particleCloud[0][3]
        else:
            cumalativeWeights[i] = cumalativeWeights[i-1] + particleCloud[i][3]

    newParticleCloud = [(0,0,0,0) for k in range(length)]

    for l in range(length):
        rnd = random.random()
        for m in range(length):
            x, y, theta , w = (0,0,0,0)
            if(rnd <= cumalativeWeights[m]):
                x, y, theta, w = particleCloud[m]
                break
        newParticle = (x, y, theta, 1.0 / length)
        newParticleCloud[l] = newParticle

    particleCloud = newParticleCloud


def updateLikelihoods(z):
    weightTotal = 0
    for i in range(len(particleCloud)):
        x, y, theta, w = particleCloud[i]
        likelihood = calculate_likelihood(x, y, theta, z)
        particleCloud[i] = (x, y, theta, likelihood)
        weightTotal += likelihood

    for i in range(len(particleCloud)):
        x, y, theta, w = particleCloud[i]
        w /= weightTotal
        particleCloud[i] = (x, y, theta, w)


def calculate_likelihood(x, y, theta, z):
    m = getExpectedDistance(x, y, theta)
    numerator = -((z-m) ** 2)
    denominator = 2 * (SONAR_SIGMA ** 2)
    power = numerator / denominator
    probability = math.exp(power)
    likelihood = K + probability
    return likelihood


def getExpectedDistance(x1, y1, theta):
#assumes horizontal / vertical walls
#assumes 0 < theta < 2 * pi
    distance = 300
    in_range = False
    for i in range(len(simpleWalls)):
        const, horizontal, start, finish = simpleWalls[i]
        if horizontal:
            if theta == 0 or theta == math.pi: continue
            y2 = const
            x2 = (y2 -y1) / math.tan(theta) + x1
            if start<x2<finish:
                in_range = True
        else:
            if theta == math.pi / 2 or theta == 1.5 * math.pi: continue
            x2 = const
            y2 = math.tan(theta) * (x2 - x1) + y1
            if start<y2<finish:
                in_range = True
        in_front = False

        if theta < -0.5 * math.pi:
            in_front = x1 > x2 and y1 > y2
        elif theta < 0:
            in_front = x1 < x2 and y1 > y2
        elif theta < math.pi / 2:
            in_front = x1 < x2 and y1 < y2
        elif theta < math.pi:
            in_front = x1 > x2 and y1 < y2

        if in_front and in_range:
            distance = min(distance, math.sqrt((x2 - x1)**2 + (y2 - y1)**2))
 #           print math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

