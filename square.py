from movement import *
from particleDataStructures import *
from week4 import *

def square(distance = 40):
    resetParticleCloud()

    squaremap = Map()
    squaremap.add_wall((0,0,40,0))
    squaremap.add_wall((40,0,40,40))
    squaremap.add_wall((40,40,0,40))
    squaremap.add_wall((0,40,0,0))
    squaremap.draw()

    go(distance)
    turn_acw(90)
    go(distance)
    turn_acw(90)
    go(distance)
    turn_acw(90)
    go(distance)
    turn_acw(90)


square()
