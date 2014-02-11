from BrickPi import *
import math

#############################
#########Constants###########
#############################

SPEED_TO_MOTO_MAGIC_NUMBER = 60 #voltage?
SLIPPING_MAGIC_NUMBER = 1.9
FLIP_MOTORS = -1
WHEEL_SPACING = 13.8
WHEEL_DIAMETER  = 3.2
WHEEL_CIRC = WHEEL_DIAMETER*math.pi
ROT_CIRCLE_CIRCUM = WHEEL_SPACING*math.pi
ROT_CIRCLE_ENC = ROT_CIRCLE_CIRCUM/WHEEL_CIRC*720*1
LEFT = PORT_B
RIGHT = PORT_A
DEFAULT_SPEED = 0.7
PATH_THRESHHOLD = 2  #determines how far the bot can stray from it's path before it's corrected
ROTATION_TIME = 10 # seconds per rotation
ROTATION_SPEED = DEFAULT_SPEED #ROT_CIRCLE_CIRCUM/ROTATION_TIME
STOP_TOLERANCE = 0
############################
############################

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
    print encL, encR


def go(distance):
    straight_drive_loop(distance)
    stopMotor()


def dist_to_enc(distance):
    return 720*distance/WHEEL_CIRC


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
    loadTolerance()
    go(40)

def square(distance):
    go(distance)
    turn_cw(90)
    go(distance)
    turn_cw(90)
    go(distance)
    turn_cw(90)
    go(distance)
    turn_cw(90)

def square40():
    loadTolerance()
    square(40)

#calibrateTolerance()
#go40()
#square40()
