from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch

from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from BetterClasses.MotorEx import *
from TankDrive.constants import *
from TankDrive.pathing import *
from Controllers.RunController import *
from robot import *



core = Robot()
oneTimeUse = True




def start_run():
    global core

    if zeroBeforeEveryRun:
        core.localizer.zero()
    
    if takeHandsOff:
        core.led_control.take_your_hands_off()
        wait(sToMs(time_to_take_hands_off))
    
    core.led_control.in_progress()

def stop_run():
    global core

    core.led_control.not_started()
    core.brick.screen.clear()
    #core.showcaseOptions()

def run1():
    #skater
    inLineCM(cm = 90, robot = core, threshold = 65)
    turnDeg(2, core)
    inLineCM(cm = 9, robot = core, threshold = 2)
    core.rightTask.run_angle(220, rotation_angle = -180)
    turnDeg(3, core)

    inLineCM(cm = 150, robot = core, threshold = 125)


    core.leftTask.dc(100)
    wait(2050)
    core.leftTask.dc(0)


    
    inLineCM(cm = -10, robot = core, threshold = 3.2, sensitivity = 0.6)
    inLineCM(cm = -90,  robot = core, threshold = 67.69)
    turnDeg(146, core)

    inLineCM(cm = -140,  robot = core, threshold = 74.5)
    turnDeg(71, core, sensitivity = 0.4, threshold = 30)

    inLineCM(cm = 13, robot = core, threshold = 6.8, sensitivity = 0.5)

    turnDeg(40, robot = core, sensitivity = 0.4, threshold = 5)
    turnDeg(core.localizer.getPoseEstimate().head + 7, core)

    core.leftTask.dc(100)
    wait(2100)
    core.leftTask.dc(0)

    inLineCM(cm = -4, robot = core, threshold = 2)
    turnDeg(125, core)
    inLineCM(cm = -160, robot = core, threshold = 58)

    return 0

def run2():
    inLineCM(cm = 85, robot = core, correctHeading = True, threshold = 51)
    turnDeg(-4, core)

    core.leftTask.run_until_stalled(500, then = Stop.HOLD)

    inLineCM(cm = -61, robot = core, threshold = 55.5)
    turnDeg(-70, core, threshold = 5)
    turnDeg(-42, core, threshold = 5)

    core.leftTask.run_angle(-300, rotation_angle = 100)
    
    turnDeg(-5, core, threshold = 2)
    inLineCM(cm = -75, robot = core, threshold = 38)

    return 0

def run3():
    inLineCM(cm = -45, robot = core, threshold = 41)
    turnDeg(36, core, threshold = 1)
    turnDeg(36, core, threshold = 1)

    inLineCM(cm = -50, robot = core, correctHeading = True, threshold = 12.6)
    inLineCM(cm = 1.5, robot = core, correctHeading = False, threshold = 1.2)
    core.localizer.setPoseEstimate(Pose(0,0,0))

    turnDeg(9, core, threshold = 2)
    wait(500)
    turnDeg(30.5, core, sensitivity = 0.5, threshold = 9.5)

    inLineCM(cm = 90, robot = core, correctHeading = False, threshold = 43)
    return 0

def run4():
    inLineCM(cm = 30, robot = core, correctHeading = True, threshold = 28, sensitivity = 1)
    turnDeg(-10, core)
    inLineCM(cm = 97, robot = core, correctHeading = True, threshold = 64)
    turnDeg(30, core, threshold = 28 )

    inLineCM(cm = -20, robot = core, correctHeading = False, threshold = 10)

    return 0

def run5():

    #skateboard
    inLineCM(cm = 50, robot = core, threshold = 20)
    turnDeg(-25, core)
    inLineCM(cm = 90, robot = core, threshold = 55)
    turnDeg(4, core)
    inLineCM(cm = 90, robot = core, threshold = 76)
    
    #back from skateboard
    inLineCM(cm = -90, robot = core, threshold = 86)
    turnDeg(82.5, core)

    #scene change
    inLineCM(cm = -90, robot = core, threshold = 76)
    inLineCM(cm = 90, robot = core, threshold = 82)
    turnDeg(49, core)

    #going to museum
    inLineCM(cm = 90, robot = core, threshold = 38)
    turnDeg(-30, core)
    inLineCM(cm = 90, robot = core, threshold = 60)
    inLineCM(cm = -90, robot = core, threshold = 88)

    #in museum stuff
    turnDeg(15, core, sensitivity = 0.5, threshold = 1.5)
    core.rightTask.run_angle(350, rotation_angle = -140)

    #back from museum
    inLineCM(cm = -90, robot = core, threshold = 69)
    turnDeg(-50, core, threshold = 2)

    #pannels up
    inLineCM(cm = 90, robot = core, threshold = 50)

    #going for flower
    core.localizer.setPoseEstimate(Pose(0,0,0))
    inLineCM(cm = -90, robot = core, threshold = 80)
    turnDeg(65.5, core)
    inLineCM(cm = 120, robot = core, threshold = 55)

    #doing the flower
    inLineCM(cm = -12, robot = core, threshold = 2.7)
    turnDeg(109.8, robot = core, sensitivity = 0.8, threshold = 3)
    turnDeg(100, core)
    inLineCM(cm = 90, robot = core, threshold = 78)
    turnDeg(50, core)
    turnDeg(100, core)
    
    return 0

def run6():
    inLineCM(cm = 90, robot = core, correctHeading = False, threshold = 49)
    inLineCM(cm = -60, robot = core, correctHeading = False, threshold = 43)
    return 0

def run7():
    inLineCM(cm = 30, robot = core, correctHeading = True, threshold = 28, sensitivity = 1)
    turnDeg(7.9, core)
    inLineCM(cm = 130, robot = core, correctHeading = True, threshold = 43)

    turnDeg(-24, core)
    turnDeg(-24, core)
    return 0

def dummy():
    wait(1000)
    return 0

def loop():
    core.update()



run_list = [Run(Button.UP, function = dummy, oneTimeUse = oneTimeUse),
            Run(Button.LEFT, function = dummy, oneTimeUse = oneTimeUse),
            Run(Button.RIGHT, function = dummy, oneTimeUse = oneTimeUse),
            Run(Button.LEFT, function = dummy, oneTimeUse = oneTimeUse)]

core.run_control.addRunList(run_list)
core.run_control.addBeforeEveryRun(function = start_run)
core.run_control.addAfterEveryRun(function = stop_run)

