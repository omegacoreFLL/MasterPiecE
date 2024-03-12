from TankDrive.constants import *
from BetterClasses.MathEx import *
from pybricks.tools import StopWatch, wait
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Stop
from robot import *

#calculates time for completing a turn
def normalizeTimeToTurn(deg):
    return abs(timeToTurn * deg / 360)

def turnRad(rad, robot, threshold = 0.1, sensitivity = 1):
    turnDeg(toDegrees(normalizeRadians(rad)), robot, threshold, sensitivity)

def turnDeg(deg, robot, 
            threshold = 0.1, 
            sensitivity = 1):

    if not isinstance(deg, float) and not isinstance(deg, int):
        raise Exception("not a valid 'deg' ----- needed type: float / int")
    if not isinstance(robot, Robot):
        raise Exception("not a Robot instance")
    if not isinstance(threshold, float) and not isinstance(threshold, int):
        raise Exception("not a valid 'threshold' ----- needed type: float / int")
    if not isinstance(sensitivity, float) and not isinstance(sensitivity, int):
        raise Exception("not a valid 'sensitivity' ----- needed type: float / int")
    
    threshold = abs(threshold)
    sensitivity = abs(sensitivity)

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

def inLineCM(cm, robot, 
            threshold = 0.1, 
            sensitivity = 1, 
            correctHeading = True, turnTangential = True, 
            interpolating = False, accelerating = False,
            tangential_angle = None, #default val
            listOfCommands = None):
    
    if not isinstance(cm, int) and not isinstance(cm, float):
        raise Exception("not a valid 'cm' ----- needed type: float / int")
    if not isinstance(robot, Robot):
        raise Exception("not an instance of Robot")
    if not isinstance(threshold, float) and not isinstance(threshold, int):
        raise Exception("not a valid 'threshold' ----- needed type: float / int")
    if not isinstance(sensitivity, float) and not isinstance(sensitivity, int):
        raise Exception("not a valid 'sensitivity' ----- needed type: float / int")
    if not isinstance(correctHeading, bool):
        raise Exception("not a valid 'correctHeading' ----- needed type: bool")
    if not isinstance(turnTangential, bool):
        raise Exception("not a valid 'turnTangential' ----- needed type: bool")
    if not isinstance(interpolating, bool):
        raise Exception("not a valid 'interpolating' ----- needed type: bool")
    if not isinstance(accelerating, bool):
        raise Exception("not a valid 'accelerating' ----- needed type: bool")
    if not isinstance(tangential_angle, float) and not isinstance(tangential_angle, int) and not tangential_angle == None:
        raise Exception("not a valid 'tangential_angle' ----- needed type: float / int")
    
    threshold = abs(threshold)
    sensitivity = abs(sensitivity)

    queuedCommands = not (listOfCommands == None)
    isBusy = True

    robot.update()
    pose = robot.localizer.getPoseEstimate()

    if tangential_angle == None:
        deg = normalizeDegrees(pose.head)
    else: 
        deg = normalizeDegrees(tangential_angle)


    if queuedCommands:
        for command in listOfCommands:
            if not isinstance(command, Command):
                raise Exception("not a Command instance")

            command.startPercent = command.startPercent / 100 * cm
            command.endPercent = command.endPercent / 100 * cm
    
    if (abs(abs(pose.head) - abs(deg)) > 2 and turnTangential):
        turnDeg(deg, robot)
        turnDeg(deg, robot)

    pastError = 0
    pastTime = 0
    headError = 0
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

                if interpolating:
                    kP = kP_interpolating
                elif abs(error) < forward_threshold:
                    kP = kP_correction_mild
                else: kP = kP_correction_agresive
                correction = robot.normalizeVoltage(headError * kP + d * kD_correction)
            
            pastTime = currentTime
            pastError = headError

        if abs(error) <= threshold:
            isBusy = False
        else:
            power = robot.normalizeVoltage(-error * kP_forw) + signum(cm) * kS_forw
            robot.setWheelPowers(left = power - correction, right = power + correction, 
                                sensitivity = sensitivity, accelerating = accelerating)
        
        if queuedCommands:
            for command in listOfCommands:
                if abs(abs(robot.localizer.distance) - abs(command.startPercent)) < 1:
                    command.start()
                elif abs(abs(robot.localizer.distance) - abs(command.endPercent)) < 1:
                    command.stop()

    if abs(headError) > 2 and turnTangential:
        turnDeg(deg, robot)
    
    #stop motors when done
    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)

def toPosition(target, robot, 
            threshold = 0.1, headThreshold = 0.1, 
            sensitivity = 1, headSensitivity = 1,
            keepHeading = False, forwards = True, correctHeading = True, 
            interpolating = False, accelerating = False,
            listOfCommands = None):
    

    if not isinstance(target, Pose):
        raise Exception("not an instance of Pose")
    if not isinstance(robot, Robot):
        raise Exception("not an instance of Robot")
    if not isinstance(threshold, float) and not isinstance(threshold, int):
        raise Exception("not a valid 'threshold' ----- needed type: float / int")
    if not isinstance(headThreshold, float) and not isinstance(headThreshold, int):
        raise Exception("not a valid 'headThreshold' ----- needed type: float / int")
    if not isinstance(sensitivity, float) and not isinstance(sensitivity, int):
        raise Exception("not a valid 'sensitivity' ----- needed type: float / int")
    if not isinstance(headSensitivity, float) and not isinstance(headSensitivity, int):
        raise Exception("not a valid 'headSensitivity' ----- needed type: float / int")
    if not isinstance(keepHeading, bool):
        raise Exception("not a valid 'keepHeading' ----- needed type: bool")
    if not isinstance(forwards, bool):
        raise Exception("not a valid 'forwards' ----- needed type: bool")
    if not isinstance(correctHeading, bool):
        raise Exception("not a valid 'correctHeading' ----- needed type: bool")
    if not isinstance(interpolating, bool):
        raise Exception("not a valid 'interpolating' ----- needed type: bool")
    if not isinstance(accelerating, bool):
        raise Exception("not a valid 'acccelerating' ----- needed type: bool")
    
    threshold = abs(threshold)
    headThreshold = abs(headThreshold)
    sensitivity = abs(sensitivity)
    headSensitivity = abs(headSensitivity)
    
    #multitasking?
    queuedCommands = not (listOfCommands == None)
    robot.update()
    pose = robot.localizer.getPoseEstimate()

    yError = target.x - pose.x 
    xError = target.y - pose.y 
    
    if forwards:
        rotationAngle = 90
        sign = -1
    else:
        rotationAngle = -90
        sign = 1
    

    pointError = rotateMatrix(yError, xError, toRadians(rotationAngle))

    needToTravelDist = hypot(yError, xError)
    turnAngle = -math.atan2(pointError.x, pointError.y)
    deg = toDegrees(turnAngle)

    #relate command intervals to distance intervals
    if queuedCommands:
        for command in listOfCommands:
            if not isinstance(command, Command):
                raise Exception("not a Command instance")

            command.startPercent = command.startPercent / 100 * needToTravelDist
            command.endPercent = command.endPercent / 100 * needToTravelDist

    #twice for better accuracy
    turnDeg(deg, robot)
    wait(100)
    turnDeg(deg, robot)

    isBusy = True 
    robot.localizer.zeroDistance()

    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()


    while isBusy:
        robot.update()
        pose = robot.localizer.getPoseEstimate()

        error = abs(robot.localizer.distance) - needToTravelDist

        correction = 0
        if correctHeading:
            if (abs(pose.head - deg) <= 360 - abs(pose.head - deg)):
                headError = pose.head - deg
            else: headError = -1 * (signum(pose.head - deg) * 360 - (pose.head - deg))
            currentTime = derivativeTimer.time()
            
            if abs(headError) > threshold:
                d = (headError - pastError) / (currentTime - pastTime)

                if interpolating:
                    kP = kP_interpolating
                elif abs(error) < forward_threshold:
                    kP = kP_correction_mild
                else: kP = kP_correction_agresive
                correction = robot.normalizeVoltage(headError * kP + d * kD_correction)

        if abs(error) <= threshold:
            isBusy = False
        else:
            power = -sign * (-error * kP_forw - signum(error) * kS_forw)
            robot.setWheelPowers(left = power - correction, right = power + correction, 
                                sensitivity = sensitivity, accelerating = accelerating)

        if queuedCommands:
            for command in listOfCommands:
                if abs(abs(robot.localizer.distance) - abs(command.startPercent)) < 1:
                    command.start()
                elif abs(abs(robot.localizer.distance) - abs(command.endPercent)) < 1:
                    command.stop()

        
    if not keepHeading:
        turnDeg(target.head, robot, sensitivity = headSensitivity, threshold = headThreshold)
    elif abs(headError) > 2:
        turnDeg(deg, robot)

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
        


        


