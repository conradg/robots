from BrickPi import *
import math
import random
from setup import *
from week3 import *
from particleDataStructures import *

#############################
#########Constants###########
#############################
SPEED_TO_MOTO_MAGIC_NUMBER = 200 #voltage?
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
DIST_BEFORE_LOCO = 20 #the distance to travel before localising positions
                      #20 is suggested in the spec but may be too short for our tolerances?
############################
############################
BrickPiSetup()

BrickPi.MotorEnable[LEFT] = 1
BrickPi.MotorEnable[RIGHT] = 1
BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPiSetupSensors()

#sets a wheel speed in centimetres/second
def setMotorSpeed(speed,port):
	BrickPi.MotorSpeed[port] = int(speed*SPEED_TO_MOTO_MAGIC_NUMBER)

#determines the number of cm each wheel has to travel to rotate by deg
def getRotationDistance(deg):
    rads = deg*math.pi/180
    return rads*WHEEL_SPACING/2

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

def stopMotors():
    setMotorSpeeds(0)
    time.sleep(1)

def setMotorSpeeds(speed):
    setMotorSpeed(speed, LEFT)
    setMotorSpeed(speed, RIGHT)

def getMotorSpeed(port):
    return BrickPi.MotorSpeed[port]/SPEED_TO_MOTO_MAGIC_NUMBER

def dist_to_enc(distance):
    return 720*distance/WHEEL_CIRC

def encs_to_dist(encs):
    return WHEEL_CIRC * (encs / 720.0)

def encs_to_angle(encs):
    return 2*math.pi * (encs_to_dist(encs)/ROT_CIRCLE_CIRCUM)

