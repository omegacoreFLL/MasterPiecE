from TankDrive.constants import *
from BetterClasses.MathEx import *
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

def turnDeg(deg, robot, 
            threshold = 0.1, 
            sensitivity = 1):


    threshold = abs(threshold)
    sensitivity = abs(sensitivity)

    heading = robot.localizer.getHeading()


    deg = normalizeDegrees(deg)
    head_error = findShortestPath(heading, deg)
    robot.resetFailSwitch(__normalizeTimeToTurn(head_error) / sensitivity)


    head_controller = PIDController(kP = kP_head, kD = kD_head, kI = 0)
    loopsInTarget = 0


    while not loopsInTarget == targetAngleValidation:
        heading = robot.localizer.getHeading()


        head_error = findShortestPath(heading, deg)
        turn = head_controller.calculate(head_error) + signum(head_error) * kS_head

        if abs(head_error) <= threshold or robot.failSwitchStop():
            loopsInTarget = loopsInTarget + 1
        else: 
            loopsInTarget = 0
            robot.setWheelPowers(left = -turn, right = turn, sensitivity = sensitivity)


    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)
      
def driveUntilOnColor(robot, left_on_color, right_on_color,
                        sensitivity = 1, threshold = 1,
                        forwards = True,
                        time = None):



    sensitivity = abs(sensitivity)
    

    if forwards:
        direction_sign = 1
    else: direction_sign = -1
    

    if not time == None:
        timeExit = True
        time = abs(time)
    else: timeExit = False

    facing_angle = robot.getHeading()

    left_color = ColorSensorEx(robot.leftColor, target_reflection = left_on_color, threshold = threshold)
    right_color = ColorSensorEx(robot.rightColor, target_reflection = right_on_color, threshold = threshold)
    head_controller = PIDController(kP = kP_head_lf, kD = kD_head_lf, kI = 0)
    reached_line_detector = EdgeDetectorEx()
    exit_timer = StopWatch()


    exitByTime = False
    exitBySuccess = False
    isBusy = True


    while isBusy:
        heading = robot..getHeading()
        left_color.update()
        right_color.update()

        head_error = findShortestPath(heading, facing_angle)
        correction = head_controller.calculate(head_error)

        if left_color.onColor() or right_color.onColor():
            reached_line_detector.set(True)
        

        reached_line_detector.update()

        if reached_line_detector.rising:
            exitBySuccess = True
            isBusy = False
        
        if timeExit:
            if msToS(exit_timer.time()) > time:
                exitByTime = True
                isBusy = False
        
        robot.setWheelPowers(100 * direction_sign - correction, 100 * direction_sign + correction, 
                    sensitivity = sensitivity)
    

    robot.setWheelPowers(0, 0)
    robot.setDriveTo(Stop.BRAKE)


