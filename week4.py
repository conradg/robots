import math

WAYPOINT_TOLERANCE = 2 #so the robot doesn't try to get infinitely nearer to the waypoint
SONAR_SIGMA = 3.0 #TODO this needs a real value
NUMBER_OF_PARTICLES = 100

particleCloud = [(0,0,0,1/NUMBER_OF_PARTICLES) for j in range(NUMBER_OF_PARTICLES)]
simpleWalls = [(0, True), (168, False), (84, True), (210, False), (168, True), (84, False), (210, True), (0, False)]

#from conrad, not used yet
point_map = {'O': (0,0), \
'A': (0,168),\
'B': (84,168),\
'C': (84,126),\
'D': (84,120),\
'E': (168,210),\
'F': (168,84),\
'G': (210,84),\
'H': (210,0)}

def resetParticleCloud():
    particleCloud = [(0,0,0,1/NUMBER_OF_PARTICLES) for j in range(NUMBER_OF_PARTICLES)]

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
    len = len(particleCloud)
    cumalativeWeights = [0 for j in range(len)]
    for i in range(len):
        if(i == 0):
            cumalativeWeights = particecloud[0]
        else:
            cumalativeWeights = cumalativeWeights[i-1] + particleCloud[i]

    newparticleCloud = [0 for k in range(len)]
    for l in range(len):
        rnd = random.random()
        for m in range(len):
            if(rnd < cumalativeWeights[m]):
                x, y, theta, w = particleCloud[m]
                break
        newParticle = (x, y, theta, 1 / len)
        newparticleCloud[l] = newParticle

    return newparticleCloud

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
        particleCloud[i] = (x, y, theta, z)

def calculate_likelihood(x, y, theta, z):
    m = getExpectedDistance(x, y, theta)
    numerator = (z-m) ** 2
    denominator = 2 * (SONAR_SIGMA ** 2)
    power = numerator / denominator
    probability = -math.exp(power)
    likelihood = 0.005 + probability
    #TODO maybe the likelihood magic number should be a global
    return likelihood

def getExpectedDistance(x1, y1, theta):
#assumes horizontal / vertical walls
#assumes 0 < theta < 2 * pi
    distance = 300

    for i in range(len(simpleWalls)):
        const, horizontal = simpleWalls[i]
        if horizontal:
            if theta == 0 or theta == math.pi: continue
            y2 = const
            x2 = (1 / math.tan(theta)) * (y2 - y1) - x1
        else:
            if theta == math.pi / 2 or theta == 1.5 * math.pi: continue
            x2 = const
            y2 = math.tan(theta) * (x2 - x1) - y1

        infront = False

        if 0 <= theta and theta < math.pi / 2:
            infront = x1 < x2 and y1 < y2
        elif math.pi / 2 <= theta and theta < math.pi:
            infront = x1 > x2 and y1 < y2
        elif math.pi <= theta and theta < 1.5 * math.pi:
            infront = x1 > x2 and y1 > y2
        elif 1.5  * math.pi <= theta and theta < 2 * math.pi:
            infront = x1 < x2 and y1 > y2

        if infront:
            distance = min(distance, math.sqrt((x2 - x1)**2 + (x2 - x1)**2))

    return distance

