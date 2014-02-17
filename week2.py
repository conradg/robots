from movement import *
import sys

def bumperRun():
    BrickPiSetup()  # setup the serial port for communication
    BrickPi.SensorType[PORT_3] = TYPE_SENSOR_TOUCH   #Set the type of sensor at PORT_3
    BrickPi.SensorType[PORT_4] = TYPE_SENSOR_TOUCH   #Set the type of sensor at PORT_4
    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi
    setMotorSpeeds(DEFAULT_SPEED)
    bumperL =  BrickPi.Sensor[PORT_3]
    bumperR =  BrickPi.Sensor[PORT_4]
    while True:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors
        if not result :
            bumperL =  BrickPi.Sensor[PORT_3]
            bumperR =  BrickPi.Sensor[PORT_4]
            print bumperL, bumperR
            if bumperL and bumperR:
                avoid(150)
            elif bumperL:
                avoid(-90)
            elif bumperR:
                avoid(90)
        setMotorSpeeds(DEFAULT_SPEED)
        time.sleep(.1)     # sleep for 10 ms

def avoid(deg):
    go(-10)
    #turn_cw(deg)
    BrickPiUpdateValues()
    startEncsL = BrickPi.Encoder[LEFT]
    startEncsR = BrickPi.Encoder[RIGHT]
    if(deg < 0):
        dir = -1
    else:
        dir = 1
    setMotorSpeed(DEFAULT_SPEED * dir, LEFT)
    setMotorSpeed(-DEFAULT_SPEED * dir, RIGHT)
    dist = deg * 8
    if(deg>0):
        while(BrickPi.Encoder[LEFT] < startEncsL + dist):
            BrickPiUpdateValues()
    else:
        while(BrickPi.Encoder[RIGHT] < startEncsR - dist):
            BrickPiUpdateValues()


def sonicStick():

    BrickPiSetup()  # setup the serial port for communication

    BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1

    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    setMotorSpeeds(DEFAULT_SPEED)

    while True:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        if not result :
            dist = BrickPi.Sensor[PORT_1]
            print dist, getMotorSpeed(LEFT), getMotorSpeed(RIGHT)
            setMotorSpeeds(0.5*(30-dist))
        time.sleep(.01)     # sleep for 10 ms

def wallStick():

    BrickPiSetup()  # setup the serial port for communication

    BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1

    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    speed = DEFAULT_SPEED
    setMotorSpeeds(speed)

    while True:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        if not result :
            dist = BrickPi.Sensor[PORT_1]
            print dist
            wheelDiff = 0.5*(30-dist)
            speedL = getMotorSpeed(LEFT)
            speedR = getMotorSpeed(RIGHT)
            setMotorSpeed(speed - wheelDiff, LEFT)
            setMotorSpeed(speed + wheelDiff, RIGHT)
            print speedL, speedR, wheelDiff
        time.sleep(0.001)     # sleep for 10 ms

