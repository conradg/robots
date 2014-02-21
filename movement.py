from BrickPi import *
import math
import random
from week3 import *
from particleDataStructures import *

#############################
#########Constants###########
#############################

SPEED_TO_MOTO_MAGIC_NUMBER = 100 #voltage?
CARPET_SLIP = 1.08
TABLE_SLIP = 1
SLIPPING_MAGIC_NUMBER = CARPET_SLIP
FLIP_MOTORS = 1
WHEEL_SPACING = 13.5
WHEEL_DIAMETER  = 5.6
WHEEL_CIRC = WHEEL_DIAMETER*math.pi
ROT_CIRCLE_CIRCUM = WHEEL_SPACING*math.pi
ROT_CIRCLE_ENC = ROT_CIRCLE_CIRCUM/WHEEL_CIRC*720*1
LEFT = PORT_B
RIGHT = PORT_A
DEFAULT_SPEED = 1
PATH_THRESHHOLD = 2  #determines how far the bot can stray from it's path before it's corrected
ROTATION_TIME = 10 # seconds per rotation
ROTATION_SPEED = DEFAULT_SPEED #ROT_CIRCLE_CIRCUM/ROTATION_TIME
STOP_TOLERANCE = 0
WALL_STOP = 10 #if you get this close to a wall in front of you then stop 
SONAR_SIGMA = 3.0 #TODO this needs a real value
DIST_BEFORE_LOCO = 20 #the distance to travel before localising positions
                      #20 is suggested in the spec but may be too short for our tolerances?
WAYPOINT_TOLERANCE = 2 #so the robot doesn't try to get infinitely nearer to the waypoint
############################
############################
NUMBER_OF_PARTICLES = 100
pointcloud = [(0,0,0,1/NUMBER_OF_PARTICLES) for j in range(NUMBER_OF_PARTICLES)]

BrickPiSetup()

BrickPi.MotorEnable[LEFT] = 1
BrickPi.MotorEnable[RIGHT] = 1
def resetPointcloud():
    pointcloud = [(0,0,0,1/NUMBER_OF_PARTICLES) for j in range(NUMBER_OF_PARTICLES)]

#sets a wheel speed in centimetres/second
def setMotorSpeed(speed,port):
	BrickPi.MotorSpeed[port] = int(speed*SPEED_TO_MOTO_MAGIC_NUMBER)

#determines the number of cm each wheel has to travel to rotate by deg
def getRotationDistance(deg):
    rads = deg*math.pi/180
    return rads*WHEEL_SPACING/2

def calibrateTolerance():
    diff = 999
    global STOP_TOLERANCE
    STOP_TOLERANCE = 0
    targetEnc = -ROT_CIRCLE_ENC/4
    while(True):
        BrickPiUpdateValues()
        encL = BrickPi.Encoder[LEFT]
        encR = BrickPi.Encoder[RIGHT]
        turn_cw(90)
        BrickPiUpdateValues()
        encLNew = BrickPi.Encoder[LEFT]
        encRNew = BrickPi.Encoder[RIGHT]
        print "target", targetEnc
        print "actual", encLNew - encL
        print "right", encRNew - encR
        diff = targetEnc - (encLNew - encL)
        print "Diff", diff

        if math.fabs(diff) < 10: break

        STOP_TOLERANCE += diff
        print "STOP_TOLERANCE", STOP_TOLERANCE
    f = open("tolerance","w")
    f.write(str(STOP_TOLERANCE))
    f.close()

def loadTolerance():
    f = open("tolerance","r")
    for line in f:
        s = line.split()
        if s:
            tol = s[0]
            global STOP_TOLERANCE
            STOP_TOLERANCE = float(tol)
    f.close()
    print "STOP Tolerance:", STOP_TOLERANCE

def turn_cw(deg):
    deg = deg*SLIPPING_MAGIC_NUMBER
    BrickPiUpdateValues()
    dist_to_rotate = ROT_CIRCLE_CIRCUM*(deg/360.0)
    if deg<0: print "Turning left"
    else: print "Turning right"

    straight_drive_loop(dist_to_rotate, True)

    stopMotors()



def stopMotors():
    setMotorSpeeds(0)
    time.sleep(1)

def setMotorSpeeds(speed):
    setMotorSpeed(speed, LEFT)
    setMotorSpeed(speed, RIGHT)

def getMotorSpeed(port):
    return BrickPi.MotorSpeed[port]/SPEED_TO_MOTO_MAGIC_NUMBER

def straight_drive_loop(dist, turn = False):
    dist*=FLIP_MOTORS
    global pointcloud

    if turn: leftFlip = -1
    else: leftFlip = 1
    driftMode = 0


    if dist > 0 : forwardFlip = 1
    else : forwardFlip = -1


    distEncs = dist_to_enc(dist)

    BrickPiUpdateValues()
    encStartL = BrickPi.Encoder[LEFT]
    encStartR = BrickPi.Encoder[RIGHT]
    encL = encStartL #0
    encR = encStartR #0
    targetSpeedMax = ROTATION_SPEED
    targetSpeed = targetSpeedMax
    encLTarget = encStartL + leftFlip * (distEncs-STOP_TOLERANCE*forwardFlip)
    encRTarget = encStartR +            (distEncs-STOP_TOLERANCE*forwardFlip)
    increasing = encLTarget>encL #May not work on small enc values
    power_mult_orig = 1.1
    power_mult_cap = 1.5
    power_mult = power_mult_orig
    min_speed = 0.45

    while True:
        # get distance travelled
        BrickPiUpdateValues()

        encsTravelledL = BrickPi.Encoder[LEFT] -  encL
        encsTravelledR = BrickPi.Encoder[RIGHT] - encR

        #update previous enc value
        encL = BrickPi.Encoder[LEFT]
        encR = BrickPi.Encoder[RIGHT]

        if not turn:
            d = encs_to_dist((encsTravelledL + encsTravelledR)/2)
            pointcloud = drawNewPointCloud(pointcloud, d, 0)


        # adjust for drift
        encLRel = math.fabs(encL-encStartL)
        encRRel = math.fabs(encR-encStartR)

        targetSpeed = min((math.fabs(distEncs) - encRRel)/400 + min_speed, targetSpeedMax) * forwardFlip


        if math.fabs(encLRel-encRRel) > PATH_THRESHHOLD : #if we get off track, increase the motorSpeed of the slower side to compensate
            if encLRel > encRRel :
                if driftMode!=1: power_mult=power_mult_orig
                driftMode=1
                setMotorSpeed(targetSpeed*power_mult, RIGHT)
                setMotorSpeed(targetSpeed*leftFlip, LEFT)

            else :
                if driftMode!=-1: power_mult=power_mult_orig
                driftMode=-1
                setMotorSpeed(targetSpeed*power_mult*leftFlip, LEFT)
                setMotorSpeed(targetSpeed, RIGHT)

            if power_mult < power_mult_cap: power_mult += 0.03

        else : #if we get back on track, reset the two speeds to the target speed
            driftMode=0
            setMotorSpeed(targetSpeed*leftFlip, LEFT)
            setMotorSpeed(targetSpeed, RIGHT)
            power_mult = power_mult_orig



        # break conditions
        if increasing and encL >= encLTarget:
            break
        if not increasing and encL <= encLTarget:
            break

        time.sleep(.02)
    #time.sleep(1)
    BrickPiUpdateValues()
    encL = BrickPi.Encoder[LEFT] - encStartL
    encR = BrickPi.Encoder[RIGHT] - encStartR
    angle = - encs_to_angle((encR - encL)/2)
    if (turn):
        pointcloud = drawNewPointCloud(pointcloud, 0, angle)
    print encL, encR

def go(distance):
    straight_drive_loop(distance)
    stopMotors()

def dist_to_enc(distance):
    return 720*distance/WHEEL_CIRC

def encs_to_dist(encs):
    return WHEEL_CIRC * (encs / 720.0)

def encs_to_angle(encs):
    return 2*math.pi * (encs_to_dist(encs)/ROT_CIRCLE_CIRCUM)

def rotateWheel(wheel,deg):
    BrickPiUpdateValues()
    targetEnc = BrickPi.Encoder[wheel] + deg*2
    BrickPi.MotorSpeed[wheel] = 2000
    while (BrickPi.Encoder[wheel] < targetEnc):
        BrickPiUpdateValues()
        print (targetEnc - BrickPi.Encoder[wheel])
        time.sleep(0.01)
    stopMotors()
    BrickPiUpdateValues()


def rotateWheel2(wheel,deg):
    BrickPiUpdateValues()
    targetEnc = BrickPi.Encoder[wheel] + deg*2
    BrickPi.MotorSpeed[wheel] = -40
    while (BrickPi.Encoder[wheel] > targetEnc):
        BrickPiUpdateValues()
        print (targetEnc - BrickPi.Encoder[wheel])
        time.sleep(0.01)
    stopMotors()
    BrickPiUpdateValues()

def go40():
    go(40)

def square(distance = 40):
    resetPointcloud()

    squaremap = Map()
    squaremap.add_wall((0,0,40,0))
    squaremap.add_wall((40,0,40,40))
    squaremap.add_wall((40,40,0,40))
    squaremap.add_wall((0,40,0,0))
    squaremap.draw()

    go(distance)
    turn_cw(-90)
    go(distance)
    turn_cw(-90)
    go(distance)
    turn_cw(-90)
    go(distance)
    turn_cw(-90)

def square40():
    square(40)

x = 0
y = 0
theta = 0

def getMeanPosition(pointcloud):
    meanX = 0
    meanY = 0
    meanTheta = 0
    for i in range(NUMBER_OF_PARTICLES):
	(xi, yi, thetai, weighti) = pointcloud[i]
        meanX += xi * weighti
	meanY += yi * weighti
	meanTheta += thetai * weigthi
    return (meanX, meanY, meanTheta)

def goTo (xnew,ynew):
    (x, y, theta) = getMeanPosition(pointcloud)

    while not maths.abs(xnew - x) < WAYPOINT_TOLERANCE or not maths.abs(ynew - y) < WAYPOINT_TOLERANCE:
        xdiff = xnew - x
        ydiff = ynew - y
        angle  = math.atan2(ydiff,xdiff)
        anglediff = angle - theta

        # Modulo pi retaining sign
        anglediff %= math.pi * (-1 if anglediff < 0 else 1)
    
        distance  = math.sqrt(xdiff**2 + ydiff**2) * 100 # *100 to convert to cm
        turn_cw(-anglediff/(2*math.pi*360))
        go(min(DIST_BEFORE_LOCO, distance))

def getExpectedDist(x, y, theta):
    return 0

def calculate_likelihood(x, y, theta, z):
    m = getExpectedDist(x, y, theta)
    likelihood = 0.005 + exp((-(z-m) ** 2) / (2 * SONAR_SIGMA ** 2))
    #TODO maybe the likelihood magic number should be a global	
    return likelihood

def updateLikelihoods(z):
    weightTotal = 0
    for i in range(len(particles)):
        x, y, theta = particles[i]
        likelihood = calculate_likelihood(x, y, theta, z)
        particles[i] = (x, y, theta, likelihood)
        weightTotal += likelihood
    
    for i in range(len(particles)):
        x, y, theta, w = particles[i]
        w /= weightTotal
        particles[i] = (x, y, theta, z)

def resample(particlecloud):
    len = len(particlecloud)
    cumalativeWeights = [0 for j in range(len)]
    for i in range(len):
        if(i == 0):
            cumalativeWeights = particecloud[0]
        else:
            cumalativeWeights = cumalativeWeights[i-1] + particleclous[i]
    
    newParticles = [0 for k in range(len)]
    for l in range(len):
        rnd = random.random()
        for m in range(len):
            if(rnd < cumalativeWeights[m]):
                x, y, theta, w = particlecloud[m]
                break
        newParticle = (x, y, theta, 1 / len)
        newParticles[l] = newParticle
    
    return newParticles

simpleWalls = [(0, True), (168, False), (84, True), (210, False), (168, True), (84, False), (210, True), (0, False)]


def getExpectedDistance(x1, y2, theta):
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

def localise():
#assumes sensors already set up
    z = BrickPi.Sensor[PORT_1] 
    updateLikelihoods(z)
    particlecloud = resample(particlecloud)

def doTheMonteCarlo():
    goTo(84, 30)
    goTo(180, 30)
    goTo(180, 54)
    goTo(126, 54)
    goTo(126, 168)
    goTo(126, 126)
    goTo(30, 54)
    goTo(84, 54)
    goTo(84, 30)
	

square(40)

