from pybricks.parameters import Port

import math

#robot-specific constrains, which can be measured / searched up on google:
#       - W = width of the robot, measured with a ruler 
#       - L = length of the robot. These 2 values would've been useful in finding the center
#         of the bot, but for now aren't used anywere. Out of habbit from FTC :D
#       - WHEEL_RADIUS = pretty self explanatory (in cm)
#       - GEAR_RATIO = geartrain from the output of the motor to the wheel. If the wheel is attached
#         to the motor, it's called a -direct drive- and the gear ratio is 1 (because there aren't any
#         external gears, so wheel spins as fast as the motor). Otherwise, this value is set to the ratio
#         of the number of teeth of input gear (the one who can spin without being attached to the other)
#         and output gear (the other gear) (in / out). Didn't need that here, but it's useful to be aware of it

#cm
W = 15
L = 20
WHEEL_RADIUS = 4.3 / 2
GEAR_RATIO = 1 # in / out
TICKS_PER_REVOLUTION = 360
MAX_TICKS_PER_SECOND = 1020

MAX_VEL = 0 #cm / s
MAX_ANGULAR_VEL = 0 #deg / s


distanceBetweenWheels = 9.97
dW = distanceBetweenWheels / 2

generalCurveMultiplier = 3





#gain constants for a custom-implemented PID controller (check this explanation out:
#       https://www.integrasources.com/blog/basics-of-pid-controllers-design-applications/ )
#
#combined with values for a friction-based feedforward controller (basically you account for the low 
#       values you give a motor, which in theory should make the robot move, but in reality the static 
#       friction wins and it stays in place (which you don't want))
kP_head = 1.97 * 1.65 #tune this
kD_head = 4.75 * 11.35
kS_head = 1

kP_forw = 5.5 * 1.2
kS_forw = 10
kP_correction_agresive = 24
kP_correction_mild = 4
kD_correction = 0

forward_threshold = 18 #cm




#'timeToTurn' 360 deg, to calculate the time for a set amount of degrees. Basically making it more efficient
#       in case of something going wrong. 'failSwitchTime' is just a generic variable to be used as threshold
timeToTurn = 6 #sec
failSwitchTime = 0
run = 1



maxVoltage = 7.9 #volts
targetAngleValidation = 17

#keeps account on which programs have been played before. Also if you're allowed to play them more than 1 time
upDone, leftDone, rightDone, downDone, middleUpDone, middleLeftDone, middleRightDone, middleDownDone = (
    False, False, False, False, False, False, False, False)

zeroBeforeEveryMove, zeroBeforeEveryRun, zeroBeforeEveryTask, zeroBeforeMotors = False, True, False, False
oneTimeUse = False

#these contain the respective ports. If any wire management changes, change this ->
ltPort, rtPort, ldPort, rdPort, gyroPort, lcPort, rcPort = (
    Port.B, Port.A, Port.C, Port.D, Port.S3, Port.S4, Port.S2
)