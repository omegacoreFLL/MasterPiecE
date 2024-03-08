from TankDrive.constants import *
from BetterClasses.MathEx import *
from pybricks.tools import StopWatch
from pybricks.parameters import Stop

#calculates time for completing a turn
def normalizeTimeToTurn(deg):
    return timeToTurn * deg / 360


def turnRad(rad, robot, threshold = 0.1, sensitivity = 1):
    turnDeg(toDegrees(normalizeRadians(rad)), robot, threshold, sensitivity)

#turning [0, 360]
def turnDeg(deg, robot, threshold = 0.1, sensitivity = 1):

    isBusy = True

    deg = normalizeDegrees(deg)
    robot.localizer.update()
    pose = robot.localizer.getPoseEstimate()

    #find shortest path to target
    if (abs(pose.head - deg) < 360 - abs(pose.head - deg)):
        error = pose.head - deg
    else: error = signum(pose.head - deg) * 360 - (pose.head - deg)




    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()
    loopsInTarget = 0

    #calculating max time accorded
    robot.resetFailSwitch(normalizeTimeToTurn(abs(error)) / sensitivity)

    #loop
    while isBusy:
        robot.localizer.update()
        pose = robot.localizer.getPoseEstimate()

        if (abs(pose.head - deg) <= 360 - abs(pose.head - deg)):
            error = pose.head - deg
        else: error = -1 * (signum(pose.head - deg) * 360 - (pose.head - deg))

        currentTime = derivativeTimer.time()
        #check if you reached the target -> stop
        if (abs(error) <= threshold and abs(pastError) <= threshold) or robot.failSwitchStop():
            loopsInTarget = loopsInTarget + 1
        else: 
            #else calculate motor powers
            loopsInTarget = 0

            d = (error - pastError) / (currentTime - pastTime)
            power = robot.normalizeVoltage(error * kP_head + d * kD_head + signum(error) * kS_head)

            #to turn in place, motor should spin opposite from one another
            robot.setWheelPowers(left = -power, right = power, sensitivity = sensitivity)

        pastTime = currentTime
        pastError = error

        if loopsInTarget == targetAngleValidation:
            isBusy = False
    
    #stop motors when done
    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)

def inLineCM(cm, robot, threshold = 0.1, sensitivity = 1, correctHeading = True, specified_angle = 0.2):

    isBusy = True

    robot.update()
    pose = robot.localizer.getPoseEstimate()

    if specified_angle == 0.2:
        deg = normalizeDegrees(pose.head)
    else: deg = normalizeDegrees(specified_angle)

    turnDeg(deg, robot)

    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()

    robot.localizer.zeroDistance()

    #loop
    while isBusy: 
        robot.update()
        pose = robot.localizer.getPoseEstimate()

        error = robot.localizer.distance - cm

        correction = 0
        if correctHeading:
            if (abs(pose.head - deg) <= 360 - abs(pose.head - deg)):
                headError = pose.head - deg
            else: headError = -1 * (signum(pose.head - deg) * 360 - (pose.head - deg))

            currentTime = derivativeTimer.time()

            #if heading is off, calculate the correction power
            if abs(headError) > threshold:
                d = (headError - pastError) / (currentTime - pastTime)
                if abs(error) < forward_threshold:
                    kP = kP_correction_mild
                else: kP = kP_correction_agresive
                correction = robot.normalizeVoltage(headError * kP + d * kD_correction)
            
            pastTime = currentTime
            pastError = headError

        if abs(error) <= threshold:
            isBusy = False
        else:
            power = robot.normalizeVoltage(-error * kP_forw) + signum(cm) * kS_forw
            robot.setWheelPowers(left = power - correction, right = power + correction, sensitivity = sensitivity)

    
    #stop motors when done
    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)