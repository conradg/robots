from movement import *


def calibrateTolerance():
    diff = 999
    global STOP_TOLERANCE
    STOP_TOLERANCE = 0
    targetEnc = -ROT_CIRCLE_ENC/4
    while(True):
        BrickPiUpdateValues()
        encL = BrickPi.Encoder[LEFT]
        encR = BrickPi.Encoder[RIGHT]
        turn_acw(90)
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

