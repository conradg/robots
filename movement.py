from BrickPi import *
import math
import random
from week3 import *
from wee4 import *
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

    stopMotor()



def stopMotor():
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
        targetSpeed = min(math.fabs(distEncs - math.fabs(encRRel))/400 + min_speed, targetSpeedMax)


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
    stopMotor()

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
    stopMotor()
    BrickPiUpdateValues()


def rotateWheel2(wheel,deg):
    BrickPiUpdateValues()
    targetEnc = BrickPi.Encoder[wheel] + deg*2
    BrickPi.MotorSpeed[wheel] = -40
    while (BrickPi.Encoder[wheel] > targetEnc):
        BrickPiUpdateValues()
        print (targetEnc - BrickPi.Encoder[wheel])
        time.sleep(0.01)
    stopMotor()
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

    xdiff = xnew - x
    ydiff = ynew - y
    angle  = math.atan2(ydiff,xdiff)
    anglediff = angle - theta
    # Modulo pi retaining sign
    anglediff %= math.pi * (-1 if anglediff < 0 else 1)

    distance  = math.sqrt(xdiff**2 + ydiff**2) * 100 # *100 to convert to cm
    turn_cw(-anglediff/(2*math.pi*360))
    go(distance)

def getExpectedDist(x, y, theta):
    return 0

sonarSigma = 3.0

def calculate_likelihood(x, y, theta, z):
    m = getExpectedDist(x, y, theta)
    likelihood = exp((-(z-m) ** 2) / (2 * sonarSigma ** 2))	
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

def localise():
#assumes sensors already set up
    z = BrickPi.Sensor[PORT_1] 
    updateLikelihoods(z)
    particlecloud = resample(particlecloud)

def doTheMonteCarlo():
    while(true):
        localise()
        go(20)
        #TODO add something to stop it running into walls

square(40)

