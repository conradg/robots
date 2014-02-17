from BrickPi import *
import math
import time
import random
#############################
#########Constants###########
#############################

SPEED_TO_MOTO_MAGIC_NUMBER = 100 #voltage?
SLIPPING_MAGIC_NUMBER = 1.08
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
pointcloud = []
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

BrickPiSetup()

BrickPi.MotorEnable[LEFT] = 1
BrickPi.MotorEnable[RIGHT] = 1

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
    if deg>0: print "Turning left"
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
    targetSpeed = ROTATION_SPEED * forwardFlip # change this
    encLTarget = encStartL + leftFlip * (distEncs-STOP_TOLERANCE*forwardFlip)
    encRTarget = encStartR +            (distEncs-STOP_TOLERANCE*forwardFlip)
    increasing = encLTarget>encL #May not work on small enc values
    power_mult_orig = 1.1
    power_mult_cap = 1.5
    power_mult = power_mult_orig




    while True:
        # get distance travelled
        BrickPiUpdateValues()
        encsTravelledL = BrickPi.Encoder[LEFT]
        encsTravelledR = BrickPi.Encoder[RIGHT]

        if not turn:
            d = encs_to_dist((encsTravelledL + encsTravelledR)/2)
            pointcloud = drawNewPointCloud(pointcloud, d, 0)

        encL = BrickPi.Encoder[LEFT]
        encR = BrickPi.Encoder[RIGHT]

        # adjust for drift
        encLRel = math.fabs(encL-encStartL)
        encRRel = math.fabs(encR-encStartR)

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
    angle = encs_to_angle((encR - encL)/2)
    if (turn):
        pointcloud = recalculatePointCloud(pointcloud, 0, angle)
    print encL, encR

def go(distance):
    straight_drive_loop(distance)
    stopMotor()

def dist_to_enc(distance):
    return 720*distance/WHEEL_CIRC

def encs_to_dist(encs):
    return WHEEL_CIRC * encs / 720

def encs_to_angle(encs):
    return 2*math.pi * (encs_to_dist(envs)/ROT_CIRCLE_CIRCUM)

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

    global pointcloud
    global PHYSICAL_SQUARE_SIDE

    PHYSICAL_SQUARE_SIDE = distance

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

    go(distance)
    turn_cw(90)
    go(distance)
    turn_cw(90)
    go(distance)
    turn_cw(90)
    go(distance)
    turn_cw(90)

def square40():
    square(40)


def getRandomErrorDist():
    return random.gauss(mu, sigmaDist)

def getRandomErrorAngle():
    return random.gauss(mu, sigmaAngle)

def getRandomErrorTurn():
    return random.gauss(mu, sigmaTurn)

def recalculatePointCloud(particles, d, dtheta):
    out = []
    d = DISPLAY_SQUARE_SIDE * d / PHYSICAL_SQUARE_SIDE
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

def drawNewPointCloud(pointcloud, d, dtheta):
    pointcloud = recalculatePointCloud(pointcloud, d, dtheta)
    print "drawParticles:"  + str(pointcloud)
    time.sleep(0.1)
    return pointcloud


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

go(40)
