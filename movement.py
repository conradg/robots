from BrickPi import *
import math
import random
from setup import *
from week4 import *
from week3 import *
from particleDataStructures import *

def goTo (xnew,ynew):
    (x, y, theta) = getMeanPosition()

    while not math.fabs(xnew - x) < WAYPOINT_TOLERANCE or not math.fabs(ynew - y) < WAYPOINT_TOLERANCE:
        localise()
        xdiff = xnew - x
        ydiff = ynew - y
        angle  = math.atan2(ydiff,xdiff)
        anglediff = angle - theta

        # Modulo pi retaining sign
        anglediff %= math.pi * (-1 if anglediff < 0 else 1)
    
        distance  = math.sqrt(xdiff**2 + ydiff**2) * 100 # *100 to convert to cm
        turn_acw(-anglediff/(2*math.pi*360))
        go(min(DIST_BEFORE_LOCO, distance))

def localise():
#assumes sensors already set up
    result = BrickPiUpdateValues()
    z = 300
    if not result:
        z = BrickPi.Sensor[PORT_1] 
    updateLikelihoods(z)
    particlecloud = resample()

def go(distance):
    straight_drive_loop(distance)
    stopMotors()

def turn_acw(deg):
    deg = deg*SLIPPING_MAGIC_NUMBER
    BrickPiUpdateValues()
    dist_to_rotate = ROT_CIRCLE_CIRCUM*(deg/360.0)
    #if deg<0: print "Turning left"
    #else: print "Turning right"

    straight_drive_loop(dist_to_rotate, True)

    stopMotors()

def straight_drive_loop(dist, turn = False):
    dist*=FLIP_MOTORS
    global particleCloud

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
            particleCloud = drawNewParticleCloud(particleCloud, d, 0)


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
        particleCloud = drawNewParticleCloud(particleCloud, 0, angle)
    #print encL, encR

