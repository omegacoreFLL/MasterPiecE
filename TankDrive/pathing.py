from TankDrive.constants import *
from BetterClasses.MathEx import *
from BetterClasses.ErrorEx import *
from Controllers.PIDController import *
from pybricks.tools import StopWatch, wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Stop
from robot import *

#calculates time for completing a turn
def __normalizeTimeToTurn(deg):
    return abs(timeToTurn * deg / 360)

def turnRad(rad, robot, threshold = 0.1, sensitivity = 1):
    turnDeg(toDegrees(normalizeRadians(rad)), robot, threshold, sensitivity)

def turnDeg(deg, robot, 
            threshold = 0.1, 
            sensitivity = 1):


    ErrorEx.isType(deg, "deg", [int, float])
    ErrorEx.isType(robot, "robot", Robot)
    ErrorEx.isType(threshold, "threshold", [int, float])
    ErrorEx.isType(sensitivity, "sensitivity", [int, float])

    threshold = abs(threshold)
    sensitivity = abs(sensitivity)
    robot.update(isBusy = True)
    pose = robot.localizer.getPoseEstimate()


    deg = normalizeDegrees(deg)
    head_error = findShortestPath(pose.head, deg)
    robot.resetFailSwitch(__normalizeTimeToTurn(abs(head_error)) / sensitivity)


    head_controller = PIDController(kP = kP_head, kD = kD_head, kI = 0)
    loopsInTarget = 0
    isBusy = True

    while isBusy:
        robot.update(isBusy)
        pose = robot.localizer.getPoseEstimate()


        head_error = findShortestPath(pose.head, deg)
        turn = head_controller.calculate(head_error) + signum(head_error) * kS_head


        if abs(head_error) <= threshold or robot.failSwitchStop():
            loopsInTarget = loopsInTarget + 1
        else: 
            loopsInTarget = 0
            robot.setWheelPowers(left = -turn, right = turn, sensitivity = sensitivity)

        if loopsInTarget == targetAngleValidation:
            isBusy = False
    

    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)

def inLineCM(cm, robot, 
            threshold = 0.1, 
            sensitivity = 1, 
            correctHeading = True, turnTangential = True, tangential_angle = None,
            interpolating = False, accelerating = False,
            listOfCommands = None):


    ErrorEx.isType(cm, "cm", [int, float])
    ErrorEx.isType(robot, "robot", Robot)
    ErrorEx.isType(threshold, "threshold", [int, float])
    ErrorEx.isType(sensitivity, "sensitivity", [int, float])
    ErrorEx.isType(correctHeading, "correctHeading", bool)
    ErrorEx.isType(turnTangential, "turnTangential", [int, float])
    ErrorEx.isType(tangential_angle, "tangential_angle", [int, float])
    ErrorEx.isType(interpolating, "interpolating", bool)
    ErrorEx.isType(accelerating, "accelerating", bool)

    threshold = abs(threshold)
    sensitivity = abs(sensitivity)
    robot.update(isBusy = True)
    pose = robot.localizer.getPoseEstimate()

    if not tangential_angle == None:
        ErrorEx.isType(tangential_angle, "tangential_angle", [float, int, None])
        facing_angle = normalizeDegrees(tangential_angle)
    else: facing_angle = pose.head
    

    queuedCommands = not (listOfCommands == None)

    if queuedCommands:
        for command in listOfCommands:
            ErrorEx.isType(command, "command", Command)

            command.startPercent = command.startPercent / 100 * cm
            command.endPercent = command.endPercent / 100 * cm
    

    if turnTangential and abs(facing_angle - pose.head) > 2:
        turnDeg(facing_angle, robot)
        turnDeg(facing_angle, robot)

    
    head_controller = PIDController(kP = kP_correction_agresive, kD = kD_correction, kI = 0)
    fwd_controller = PIDController(kP = kP_fwd, kD = 0, kI = 0)
    robot.localizer.zeroDistance()
    isBusy = True

    while isBusy: 
        robot.update(isBusy)
        pose = robot.localizer.getPoseEstimate()


        fwd_error = cm - robot.localizer.distance
        forward = fwd_controller.calculate(fwd_error) + signum(cm) * kS_fwd


        if correctHeading:

            if interpolating:
                kP = kP_interpolating
            elif abs(fwd_error) < forward_threshold:
                kP = kP_correction_mild
            else: kP = kP_correction_agresive

            head_controller.setCoefficients(kP = kP)
            head_error = findShortestPath(pose.head, facing_angle)
            correction = head_controller.calculate(head_error)
        
        else: correction = 0


        if abs(fwd_error) <= threshold:
            isBusy = False
        else: robot.setWheelPowers(left = forward - correction, right = forward + correction, 
                            sensitivity = sensitivity, accelerating = accelerating)
        

        if queuedCommands:
            for command in listOfCommands:
                command.update(robot.localizer.distance)


    if abs(head_error) > 2 and turnTangential:
        turnDeg(facing_angle, robot)
        turnDeg(facing_angle)
    
    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)

def toPosition(target, robot, 
            threshold = 0.1, headThreshold = 0.1, 
            sensitivity = 1, headSensitivity = 1,
            keepHeading = False, correctHeading = True, 
            forwards = True, interpolating = False, accelerating = False,
            listOfCommands = None):
    

    ErrorEx.isType(target, "target", Pose)
    ErrorEx.isType(robot, "robot", Robot)
    ErrorEx.isType(threshold, "threshold", [int, float])
    ErrorEx.isType(headThreshold, "headThreshold", [int, float])
    ErrorEx.isType(sensitivity, "sensitivity", [int, float])
    ErrorEx.isType(headSensitivity, "headSensitivity", [int, float])
    ErrorEx.isType(keepHeading, "keepHeading", bool)
    ErrorEx.isType(correctHeading, "correctHeading", bool)
    ErrorEx.isType(forwards, "forwards", bool)
    ErrorEx.isType(interpolating, "interpolating", bool)
    ErrorEx.isType(accelerating, "accelerating", bool)
    
    
    threshold = abs(threshold)
    headThreshold = abs(headThreshold)
    sensitivity = abs(sensitivity)
    headSensitivity = abs(headSensitivity)
    robot.update(isBusy = True)
    pose = robot.localizer.getPoseEstimate()
    

    if forwards:
        direction_sign = 1
    else: direction_sign = -1

    yError = target.x - pose.x 
    xError = target.y - pose.y 
    pointError = rotateMatrix(yError, xError, toRadians(90 * direction_sign))

    needToTravelDistance = hypot(yError, xError)
    facing_angle = toDegrees(-math.atan2(pointError.x, pointError.y))


    queuedCommands = not (listOfCommands == None)

    if queuedCommands:
        for command in listOfCommands:
            ErrorEx.isType(command, "command", Command)

            command.startPercent = command.startPercent / 100 * needToTravelDistance
            command.endPercent = command.endPercent / 100 * needToTravelDistance


    turnDeg(facing_angle, robot)
    turnDeg(facing_angle, robot)


    head_controller = PIDController(kP = kP_correction_agresive, kD = kD_correction, kI = 0)
    fwd_controller = PIDController(kP = kP_fwd, kD = 0, kI = 0)
    robot.localizer.zeroDistance()
    isBusy = True

    while isBusy:
        robot.update(isBusy)
        pose = robot.localizer.getPoseEstimate()


        fwd_error = direction_sign * needToTravelDistance - robot.localizer.distance
        forward = fwd_controller.calculate(fwd_error) + direction_sign * kS_fwd


        if correctHeading:

            if interpolating:
                kP = kP_interpolating
            elif abs(fwd_error) < forward_threshold:
                kP = kP_correction_mild
            else: kP = kP_correction_agresive

            head_controller.setCoefficients(kP = kP)
            head_error = findShortestPath(pose.head, facing_angle)
            correction = head_controller.calculate(head_error)

        else: correction = 0


        if abs(fwd_error) <= threshold:
            isBusy = False
        else: robot.setWheelPowers(left = forward - correction, right = forward + correction, 
                            sensitivity = sensitivity, accelerating = accelerating)


        if queuedCommands:
            for command in listOfCommands:
                command.update(robot.localizer.distance)

        
    if not keepHeading:
        turnDeg(target.head, robot, sensitivity = headSensitivity, threshold = headThreshold)
        turnDeg(target.head, robot, sensitivity = headSensitivity, threshold = headThreshold)
    elif abs(head_error) > 2:
        turnDeg(facing_angle, robot)
        turnDeg(facing_angle, robot)

    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)

def lineSquare(robot,
                sensitivity = 1,
                backing_distance = 0.7, success_threshold = 1,
                forwards = True, accelerating = False,
                time_threshold = None):

    if not isinstance(robot, Robot):
        raise Exception("not a robot instance")
    if not isinstance(sensitivity, float) and not isinstance(sensitivity, int):
        raise Exception("not a valid 'sensitivity' ----- needed type: float / int")
    if not isinstance(backing_distance, float) and not isinstance(backing_distance, int):
        raise Exception("not a valid 'backing_distance' ----- needed type: float / int")
    if not isinstance(success_threshold, int):
        raise Exception("not a valid 'success_threshold' ----- needed type: int")
    if not isinstance(forwards, bool):
        raise Exception("not a valid 'forwards' ----- needed type: bool")
    if not isinstance(accelerating, bool):
        raise Exception("not a valid 'accelerating' ----- needed type: bool")
    if not isinstance(time_threshold, float) and not isinstance(time_threshold, int) and not success_threshold == None:
        raise Exception("not a valid 'time_threshold' ----- needed type: float")
    
    robot.update()
    pose = robot.localizer.getPoseEstimate()
    deg = normalizeDegrees(pose.head)
    
    sensitivity = abs(sensitivity)
    success_threshold = abs(success_threshold)
    time_threshold = abs(time_threshold)
    backing_distance = abs(backing_distance)

    timer = StopWatch()
    derivativeTimer = StopWatch()
    pastError = 0
    pastTime = 0

    if forwards:
        direction = 1
    else: direction = -1

    exitByTime = False
    exitBySuccess = False

    times_reached = 0
    reached_line = False
    correction = 0

    timer.reset()
    while not exitByTime and not exitBySuccess:
        robot.update()
        pose = robot.localizer.getPoseEstimate()

        if times_reached > 0:
            forward_sensitivity = 0.7
        else: forward_sensitivity = sensitivity

        if not reached_line:
            if (abs(pose.head - deg) <= 360 - abs(pose.head - deg)):
                headError = pose.head - deg
            else: headError = -1 * (signum(pose.head - deg) * 360 - (pose.head - deg))

            currentTime = derivativeTimer.time()

            if abs(headError) > 0.1:
                d = (headError - pastError) / (currentTime - pastTime)

                correction = robot.normalizeVoltage(headError * kP_head_lf + d * kD_head_lf)
        else: correction = 0

        robot.setWheelPowers(100 * direction - correction, 100 * direction + correction, 
                            sensitivity = forward_sensitivity, accelerating = accelerating)

        left_reading = robot.leftColor.reflection()
        right_reading = robot.rightColor.reflection()

        actual_turn_rate = abs(left_reading - right_reading) * turn_rate

        if left_reading >= left_on_line and right_reading <= right_on_line:
            inLineCM(cm = (- abs(backing_distance) - 1) * direction, robot = robot, threshold = 1, 
                    sensitivity = 0.7, turnTangential = False)
            turnDeg(robot.localizer.getPoseEstimate().head + direction * abs(actual_turn_rate), robot)

            if not reached_line:
                timer.reset()
                reached_line = True


        elif left_reading <= left_on_line and right_reading >= right_on_line:
            inLineCM(cm = (- abs(backing_distance) - 1) * direction, robot = robot, threshold = 1, 
                    sensitivity = 0.7, turnTangential = False)
            turnDeg(robot.localizer.getPoseEstimate().head - direction * abs(actual_turn_rate), robot)

            if not reached_line:
                timer.reset()
                reached_line = True
    

        elif left_reading <= left_on_line and right_reading <= right_on_line:
            times_reached = times_reached + 1

            if times_reached < success_threshold:
                inLineCM(cm = (- abs(backing_distance) - 1) * direction, robot = robot,  threshold = 1, 
                        sensitivity = 0.7, turnTangential = False)
            else: 
                exitBySuccess = True
                print("exit by success")

            if not reached_line:
                timer.reset()
                reached_line = True


        if not time_threshold == None and reached_line:
            if msToS(timer.time()) > time_threshold:
                exitByTime = True
                print("exit by time")
    

    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)
        
def lineFollow(robot, sensor,
                sensitivity = 1, time = 10,
                forwards = True, left_curve = True):
    
    if not isinstance(robot, Robot):
        raise Exception("not an instance of Robot")
    if not isinstance(sensor, ColorSensor):
        raise Exception("not an instance of ColorSensor")
    if not isinstance(sensitivity, float) and not isinstance(sensitivity, int):
        raise Exception("not a valid 'sensitivity' ----- needed type: float / int")
    if not isinstance(time, float) and not isinstance(time, int):
        raise Exception("not a valid 'time' ----- needed type: float / int")
    if not isinstance(forwards, bool):
        raise Exception("not a valid 'forwards' ----- needed type: bool")
    if not isinstance(left_curve, bool):
        raise Exception("not a valid 'left_curve' ----- needed type: bool")
    
    sensitivity = abs(sensitivity)
    time = abs(time)

    timer = StopWatch()

    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()
    integral = 0

    if forwards:
        forward_direction = 1
    else: forward_direction = -1

    if left_curve:
        turn_direction = 1
    else: turn_direction = -1

    forwardSpeed = 60 * forward_direction * sensitivity

    exitByTime = False
    timer.reset()
    while not exitByTime:
        robot.update()
        reading = sensor.reflection()

        error = reading - on_edge
        integral = integral + error

        currentTime = derivativeTimer.time()
        d = (error - pastError) / (currentTime - pastTime)
  
        pastTime = currentTime
        pastError = error

        turnSpeed = (error * kP_line_follow + d * kD_line_follow + integral * kI_line_follow) * turn_direction 
        print(turnSpeed)

        robot.setWheelPowers(left = forwardSpeed - turnSpeed, right = forwardSpeed + turnSpeed)

        if msToS(timer.time()) > time:
            exitByTime = True
    

    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)
        


        


