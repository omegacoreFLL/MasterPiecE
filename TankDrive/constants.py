from pybricks.parameters import Port
import math

W = 15
L = 20
WHEEL_RADIUS = 4.3 / 2
GEAR_RATIO = 1 # in / out
TICKS_PER_REVOLUTION = 360
MAX_TICKS_PER_SECOND = 1020


distanceBetweenWheels = 9.97
dW = distanceBetweenWheels / 2

generalCurveMultiplier = 3
secondsToMilliseconds = 1000

kP_head = 1.97 * 1.25 
kD_head = 4.75 * 10.35
kS_head = 1

kP_fwd = 5.5 * 1.2
kS_fwd = 20
kP_correction_agresive = 20
kP_correction_mild = 4
kD_correction = 2

kP_interpolating = 4.2 #for pure pursuit, go to pose without turning first (curve motion)

forward_threshold = 18 #cm
minimum_power = 5

turn_rate = 0.85 #line squaring
left_on_line = 9
right_on_line = 9

on_edge = 20 #line following
kP_head_lf = 5
kD_head_lf = 2
kP_line_follow = 6
kD_line_follow = 0
kI_line_follow = 0.0

timeToTurn = 6 #sec
failSwitchTime = 0


acceleration_dc = 20
acceleration_interval = 0.3 #sec

maxVoltage = 7.9 #volts
targetAngleValidation = 17

zeroBeforeEveryMove, zeroBeforeEveryRun, zeroBeforeEveryTask, zeroBeforeMotors = False, True, False, False
oneTimeUse = False

takeHandsOff = True
time_to_take_hands_off = 0.6 #sec
do_runs_in_order = False

ltPort, rtPort, ldPort, rdPort, gyroPort, lcPort, rcPort = (
    Port.B, Port.A, Port.C, Port.D, Port.S3, Port.S4, Port.S2
)