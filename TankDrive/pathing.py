from TankDrive.constants import *
from BetterClasses.MathEx import *
from BetterClasses.ErrorEx import *
from BetterClasses.ColorSensorEx import *
from BetterClasses.EdgeDetectorEx import *
from Controllers.PIDController import *
from pybricks.tools import StopWatch, wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Stop
from robot import *

#calculates time for completing a turn
def __normalizeTimeToTurn(deg):
    return abs(timeToTurn * deg / 360)

def turnRad(rad, robot, 
            threshold = 0.1, 
            sensitivity = 1):
    turnDeg(toDegrees(normalizeRadians(rad)), robot, threshold, sensitivity)

def turnDeg(deg, robot, 
            threshold = 0.1, 
            sensitivity = 1):


    isType([deg, robot, threshold, sensitivity], 
            ["deg", "robot", "threshold", "sensitivity"], 
            [[int, float], Robot, [int, float], [int, float]])


    threshold = abs(threshold)
    sensitivity = abs(sensitivity)
    robot.updateOdometry()
    pose = robot.localizer.getPoseEstimate()


    deg = normalizeDegrees(deg)
    head_error = findShortestPath(pose.head, deg)
    robot.resetFailSwitch(__normalizeTimeToTurn(head_error) / sensitivity)


    head_controller = PIDController(kP = kP_head, kD = kD_head, kI = 0)
    loopsInTarget = 0
    isBusy = True

    while isBusy:
        robot.updateOdometry()
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

def inLineCM(cm, robot, inOtherFunction = False,
            threshold = 0.1, 
            sensitivity = 1, 
            correctHeading = True, tangential_angle = None,
            interpolating = False, accelerating = False,
            listOfCommands = None):

    if not inOtherFunction:
        isType([cm, robot, threshold, sensitivity, correctHeading, interpolating, accelerating],
                ["cm", "robot", "threshold", "sensitivity", "correctHeading", "interpolating", "accelerating"],
                [[int, float], Robot, [int, float], [int, float], bool, bool, bool])


        threshold = abs(threshold)
        sensitivity = abs(sensitivity)
        robot.updateOdometry()
        

    pose = robot.localizer.getPoseEstimate()
    if not tangential_angle == None:
        isType([tangential_angle], ["tangential_angle"], [[float, int]])
        facing_angle = normalizeDegrees(tangential_angle)
    else: facing_angle = pose.head
        

    queuedCommands = not (listOfCommands == None)

    if queuedCommands:
        for command in listOfCommands:
            isType([command], ["command"], [Command])
            command.calculate(cm)
    

    if not interpolating and abs(facing_angle - pose.head) > 0.4:
        turnDeg(facing_angle, robot)
        turnDeg(facing_angle, robot)

    
    head_controller = PIDController(kP = kP_correction_agresive, kD = kD_correction, kI = 0)
    fwd_controller = PIDController(kP = kP_fwd, kD = 0, kI = 0)
    robot.localizer.zeroDistance()
    isBusy = True


    while isBusy: 
        robot.updateOdometry()
        pose = robot.localizer.getPoseEstimate()


        fwd_error = cm - robot.localizer.distance
        fwd_error_abs = abs(fwd_error)
        forward = fwd_controller.calculate(fwd_error) + signum(cm) * kS_fwd


        if correctHeading:

            if interpolating:
                kP = kP_interpolating
            elif fwd_error_abs < forward_threshold:
                kP = kP_correction_mild
            else: kP = kP_correction_agresive

            head_controller.setCoefficients(kP = kP)
            head_error = findShortestPath(pose.head, facing_angle)
            correction = head_controller.calculate(head_error)
        
        else: correction = 0


        if fwd_error_abs <= threshold:
            isBusy = False
        else: robot.setWheelPowers(left = forward - correction, right = forward + correction, 
                            sensitivity = sensitivity, accelerating = accelerating)
        

        if queuedCommands:
            for command in listOfCommands:
                command.update(robot.localizer.distance)

    if abs(head_error) > 0.4:
        turnDeg(facing_angle, robot)
        turnDeg(facing_angle, robot)

    elif not inOtherFunction:
        robot.setWheelPowers(0, 0)
        robot.setDriveTo(Stop.BRAKE)

def toPosition(target, robot, inOtherFunction = False,
            threshold = 0.1, headThreshold = 0.1, 
            sensitivity = 1, headSensitivity = 1,
            keepHeading = False, correctHeading = True, 
            forwards = True, interpolating = False, accelerating = False,
            listOfCommands = None):
    

    if not inOtherFunction:
        isType([target, robot, threshold, headThreshold, sensitivity, headSensitivity, keepHeading, correctHeading, forwards, interpolating, accelerating],
                ["target", "robot", "threshold", "headThreshold", "sensitivity", "headSensitivity", "keepHeading", "correctHeading", "forwards", "interpolating", "accelerating"], 
                [Pose, Robot, [int, float], [int, float], [int, float], [int, float], bool, bool, bool, bool, bool])
        
        
        threshold = abs(threshold)
        headThreshold = abs(headThreshold)
        sensitivity = abs(sensitivity)
        headSensitivity = abs(headSensitivity)
        robot.updateOdometry()
        pose = robot.localizer.getPoseEstimate()
    

    if forwards:
        direction_sign = 1
    else: direction_sign = -1

    yError = target.x - pose.x 
    xError = target.y - pose.y 
    pointError = rotateMatrix(yError, xError, toRadians(90 * direction_sign))

    needToTravelDistance = hypot(yError, xError)
    facing_angle = toDegrees(-math.atan2(pointError.x, pointError.y))


    inLineCM(cm = direction_sign * needToTravelDistance, robot = robot, inOtherFunction = True,
                sensitivity = sensitivity, correctHeading = correctHeading,
                tangential_angle = facing_angle, interpolating = interpolating, 
                accelerating = accelerating, listOfCommands = listOfCommands)
        

    if not keepHeading:
        turnDeg(target.head, robot, sensitivity = headSensitivity, threshold = headThreshold)
        turnDeg(target.head, robot, sensitivity = headSensitivity, threshold = headThreshold)

    elif not inOtherFunction:
        robot.setWheelPowers(0, 0)
        robot.setDriveTo(Stop.BRAKE)
    
def lineSquare(robot,
                sensitivity = 1,
                backing_distance = 1.5, success_threshold = 1,
                forwards = True, accelerating = False,
                time_threshold = None):


    isType([robot, sensitivity, backing_distance, success_threshold, forwards, accelerating],
            ["robot", "sensitivity", "backing_distance", "success_threshold", "forwards", "acceelerating"],
            [Robot, [int, float], [int, float], [int, float], bool, bool])


    sensitivity = abs(sensitivity)
    success_threshold = abs(success_threshold)
    backing_distance = abs(backing_distance)
    robot.updateOdometry()
    pose = robot.localizer.getPoseEstimate()
    facing_angle = pose.head

    left_color = ColorSensorEx(robot.leftColor, target_reflection = left_on_line)
    right_color = ColorSensorEx(robot.rightColor, target_reflection = right_on_line)


    if forwards:
        direction_sign = 1
    else: direction_sign = -1


    exitByTime = False
    exitBySuccess = False
    reached_line_detector = EdgeDetectorEx()
    times_reached = 0

    head_controller = PIDController(kP = kP_head_lf, kD = kD_head_lf, kI = 0)
    turn_direction = 0
    isBusy = True

    
    if not time_threshold == None:
        time_threshold = abs(time_threshold)
        exit_timer = StopWatch()
        time_exit = True
    else: time_exit = False
    loopTime = 0
    timer = StopWatch()
    
    while isBusy:
        robot.updateOdometry()
        pose = robot.localizer.getPoseEstimate()
        left_color.update()
        right_color.update()

        left_on_color = left_color.onColor()
        right_on_color = right_color.onColor()
        left_reading = left_color.reading
        right_reading = right_color.reading

        actual_turn_rate = abs(left_reading - right_reading) * turn_rate


        if not left_on_color and right_on_color:
            reached_line_detector.set(True)
            times_reached = 0
            turn_direction = 1

        elif left_on_color and not right_on_color:
            reached_line_detector.set(True)
            times_reached = 0
            turn_direction = -1

        elif left_on_color and right_on_color:
            reached_line_detector.set(True)
            times_reached += 1
            turn_direction = 0


        reached_line_detector.update()

        if reached_line_detector.high:
            forward_sensitivity = 0.7
            correction = 0
    
            if time_exit:
                if msToS(exit_timer.time()) > time_threshold:
                    exitByTime = True
                    robot.setWheelPowers(0, 0)
                    robot.setDriveTo(Stop.BRAKE)
                    print("exit by time")
            
            if times_reached >= success_threshold:
                exitBySuccess = True
                robot.setWheelPowers(0, 0)
                robot.setDriveTo(Stop.BRAKE)
                print("exit by success")

        elif reached_line_detector.low:
            forward_sensitivity = sensitivity
            head_error = findShortestPath(pose.head, facing_angle)
            correction = head_controller.calculate(head_error)

        elif reached_line_detector.rising:
            if time_exit:
                exit_timer.reset()
        

        isBusy = not exitByTime and not exitBySuccess
        

        if (left_color.detector.rising or right_color.detector.rising) and isBusy:
            inLineCM(cm = (backing_distance + 7) * -direction_sign, robot = robot, inOtherFunction = True, threshold = 7, 
                        sensitivity = 0.7, turnTangential = False)
            turnDeg(robot.localizer.getPoseEstimate().head + turn_direction * direction_sign * actual_turn_rate, robot)

        robot.setWheelPowers(100 * direction_sign - correction, 100 * direction_sign + correction, 
                    sensitivity = forward_sensitivity, accelerating = accelerating)
        


        endLoopTime = timer.time()
        print("freq: ", 1000 / (endLoopTime - loopTime))
        loopTime = endLoopTime
    

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
        robot.updateOdometry()
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
        