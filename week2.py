from BrickPi import *   #import BrickPi.py file to use BrickPi operations
import week1

def bumperRun():
    BrickPiSetup()  # setup the serial port for communication

    BrickPi.SensorType[PORT_3] = TYPE_SENSOR_TOUCH   #Set the type of sensor at PORT_3
    BrickPi.SensorType[PORT_4] = TYPE_SENSOR_TOUCH   #Set the type of sensor at PORT_4

    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    week1.startMotor(-25)

    bumperL =  BrickPi.Sensor[PORT_3]
    bumperR =  BrickPi.Sensor[PORT_4]

    while True:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors
        if not result :
            bumperL =  BrickPi.Sensor[PORT_3]
            bumperR =  BrickPi.Sensor[PORT_4]
            print bumperL, bumperR
            if bumperL and bumperR:
                avoid(180)
            elif bumperL:
                avoid(-90)
            elif bumperR:
                avoid(90)
        time.sleep(.01)     # sleep for 10 ms

def avoid(deg):
    week1.startMotor(12)
    week1.turn_acw(deg)
    week1.startMotor(-25)

def sonicStick():

    BrickPiSetup()  # setup the serial port for communication

    BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1

    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    week1.startMotor(-25)

    while True:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        if not result :
            dist = BrickPi.Sensor[PORT_1]
            print dist
            week1.startMotor(1*(20-dist))
        time.sleep(.01)     # sleep for 10 ms

def wallStick():

    BrickPiSetup()  # setup the serial port for communication

    BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1

    BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

    week1.startMotor(-40)

    while True:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        if not result :
            dist = BrickPi.Sensor[PORT_1]
            print dist
            wheelDiff = 1.5*(30-dist)
            speedL = week1.getMotorSpeed(week1.LEFT)
            speedR = week1.getMotorSpeed(week1.RIGHT)
            week1.setMotorSpeed(-40 + wheelDiff, week1.LEFT)
            week1.setMotorSpeed(-40 - wheelDiff, week1.RIGHT)
            print speedL, speedR, wheelDiff
        time.sleep(.1)     # sleep for 10 ms


#bumperRun()
#sonicStick()
wallStick()
