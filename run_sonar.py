
from week2 import *

BrickPiSetup()  # setup the serial port for communication
BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

while True:
    result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
    print BrickPi.Sensor[PORT_1]
    time.sleep(.05)     # sleep for 10 ms
