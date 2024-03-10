#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math

from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from BetterClasses.MotorEx import *
from TankDrive.constants import *
from TankDrive.pathing import *
from robot import *

import threading

omega = Robot()
loopTimer = StopWatch()
startTime = 0
endTime = 0


def every_loop():
    omega.update()

def start_program():
    global omega

    if zeroBeforeEveryRun:
        omega.localizer.zero()
    
    if takeHandsOff:
        omega.led_control.take_your_hands_off()
        wait(sToMs(time_to_take_hands_off))
    
    omega.showcaseInProgress()
    omega.led_control.in_progress()

def stop_program(run_boolean):
    global omega

    run_boolean = True
    run += 1
    omega.led_control.not_started()
    omega.showcaseOptions()


while True:
    startTime = endTime
    omega.update()
    omega.led_control.not_started()
    

    if omega.gamepad.wasJustPressed(Button.UP):
        omega.localizer.zero()
        omega.led_control.take_your_hands_off()

        wait(600)

        omega.led_control.in_progress()

        inLineCM(60, omega, turnTangential = False, interpolating = True, tangential_angle = 90, sensitivity = 0.6,
                    listOfCommands = [Command(motor = omega.leftTask, runType = "DC", speed = 100,
                                        startPercent = 30, endPercent = 70),
                                      Command(motor = omega.rightTask, runType = "DC", speed = 70,
                                        startPercent = 10, endPercent = 50)])
        omega.printPose()
        toPosition(Pose(0, 0, 0), omega)

        omega.setDriveTo(Stop.COAST)
        omega.setDriveTo(Stop.COAST)

        omega.getPose()

    if omega.gamepad.wasJustPressed(Button.DOWN):
        omega.localizer.zero()
        omega.led_control.take_your_hands_off()

        wait(600)

        omega.led_control.in_progress()

        lineSquare(omega, time_threshold = 7, forwards = True)

    
    if omega.gamepad.wasJustPressed(Button.LEFT):
        omega.localizer.zero()
        omega.led_control.take_your_hands_off()

        wait(600)

        omega.led_control.in_progress()

        lineFollow(omega, omega.leftColor, left_curve = True)
    
        omega.setDriveTo(Stop.COAST)
        omega.setDriveTo(Stop.COAST)
    
    if omega.gamepad.wasJustPressed(Button.RIGHT):
        omega.localizer.zero()
        omega.led_control.take_your_hands_off()

        wait(600)

        omega.led_control.in_progress()

        inLineCM(cm = 70, robot = omega, accelerating = True)
        toPosition(Pose(0, 0, 0), omega, accelerating = True)
    
        omega.setDriveTo(Stop.COAST)
        omega.setDriveTo(Stop.COAST)
    
    if omega.gamepad.wasJustPressed(Button.CENTER):
        omega.led_control.take_your_hands_off()

        wait(600)

        omega.led_control.in_progress()

        toPosition(Pose(0, 0, 0), omega, accelerating = True)
    
        omega.setDriveTo(Stop.COAST)
        omega.setDriveTo(Stop.COAST)
    
    endTime = loopTimer.time()
    print("loop time", secondsToMilliseconds / (endTime - startTime))

        
    


'''
def inCurve(target, keepHeading = False, left = False,
                listOfCommands = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
    global botPose, dW, leftDrive, rightDrive, isBusy, distance
    queuedCommands = not (len(listOfCommands) > 10)

    updateAll()

    errorX = target.x
    errorY = target.y

    if errorX <= 0:
        signX = -1
        errorX *= -1
    else:
        signX = 1

    theta = normalizeRadians(2 * (math.atan2(errorY, errorX) - toRadians(botPose.head)))
    if theta > 2 * math.pi:
        theta = 2 * math.pi - theta
    d = hypot(errorX, errorY)

    radius = d / (2 * math.sin(theta / 2))
    
    arcD = theta * radius

    dR = (radius + dW) * theta
    dL = (radius - dW) * theta

    sec = arcD / MAX_TICKS_PER_SECOND

    velR = dR / sec
    velL = dL / sec

    if velR > velL:
        velL = 100 * velL / velR
        velR = 100
    else:
        velR = 100 * velR / velL
        velL = 100
    
    if queuedCommands:
        for com in listOfCommands:
            com.startPercent = com.startPercent / 100 * arcD
            com.endPercent = com.endPercent / 100 * arcD

    isBusy = True
    zeroDistance()

    while isBusy:
        updateAll()

        leftDrive.dc(clipMotor(velR * signX))
        rightDrive.dc(clipMotor(velL * signX))

        if abs(abs(distance) - abs(arcD)) < 1:
            isBusy = False
        
        if queuedCommands:
            for com in listOfCommands:
                if abs(abs(distance) - abs(com.startPercent)) < 1:
                    com.start()
                elif abs(abs(distance) - abs(com.endPercent)) < 1:
                    com.stop()
    

    if not keepHeading:
        turnDeg(target.head)
        turnDeg(target.head)

    leftDrive.dc(0)
    rightDrive.dc(0)
    leftDrive.brake()
    rightDrive.brake()

def run1():
    #skater
    inLineCM(cm = 90, threshold = 65)
    turnDeg(2)
    inLineCM(cm = 9, threshold = 2)
    rightTask.run_angle(220, rotation_angle = -180)
    turnDeg(3)

    inLineCM(cm = 150, threshold = 125)


    leftTask.dc(100)
    wait(2050)
    leftTask.dc(0)


    
    inLineCM(cm = -10, threshold = 3.2, sensitivity = 0.6)
    inLineCM(cm = -90, threshold = 67.69)
    turnDeg(146)

    inLineCM(cm = -140, threshold = 74.5)
    turnDeg(71, sensitivity = 0.4, threshold = 30)

    inLineCM(cm = 13, threshold = 6.8, sensitivity = 0.5)

    turnDeg(40, sensitivity = 0.4, threshold = 5)
    turnDeg(botPose.head + 7)

    leftTask.dc(100)
    wait(2100)
    leftTask.dc(0)

    inLineCM(cm = -4, threshold = 2)
    turnDeg(125)
    inLineCM(cm = -160, threshold = 58)

    return 0

def run2():
    inLineCM(cm = 85, correctHeading = True, threshold = 51)
    turnDeg(-4)

    leftTask.run_until_stalled(500, then = Stop.HOLD)

    inLineCM(cm = -61, threshold = 55.5)
    turnDeg(-70, threshold = 5)
    turnDeg(-42, threshold = 5)

    leftTask.run_angle(-300, rotation_angle = 100)
    
    turnDeg(-5, threshold = 2)
    inLineCM(cm = -75, threshold = 38)

    return 0

def run3():
    inLineCM(cm = -45, threshold = 41)
    turnDeg(36, threshold = 1)
    turnDeg(36, threshold = 1)

    inLineCM(cm = -50, correctHeading = True, threshold = 12.6)
    inLineCM(cm = 1.5, correctHeading = False, threshold = 1.2)
    setPoseEstimate(Pose(0,0,0))

    turnDeg(9, threshold = 2)
    wait(500)
    turnDeg(30.5, sensitivity = 0.5, threshold = 9.5)

    inLineCM(cm = 90, correctHeading = False, threshold = 43)
    return 0

def run4():
    inLineCM(cm = 30, correctHeading = True, threshold = 28, sensitivity = 1)
    turnDeg(-10)
    inLineCM(cm = 97, correctHeading = True, threshold = 64)
    turnDeg(30, threshold = 28 )

    inLineCM(cm = -20, correctHeading = False, threshold = 10)

    return 0

def run5():

    #skateboard
    inLineCM(cm = 50, threshold = 20)
    turnDeg(-25)
    inLineCM(cm = 90, threshold = 55)
    turnDeg(4)
    inLineCM(cm = 90, threshold = 76)
    
    #back from skateboard
    inLineCM(cm = -90, threshold = 86)
    turnDeg(82.5)

    #scene change
    inLineCM(cm = -90, threshold = 76)
    inLineCM(cm = 90, threshold = 82)
    turnDeg(49)

    #going to museum
    inLineCM(cm = 90, threshold = 38)
    turnDeg(-30)
    inLineCM(cm = 90, threshold = 60)
    inLineCM(cm = -90, threshold = 88)

    #in museum stuff
    turnDeg(15, sensitivity = 0.5, threshold = 1.5)
    rightTask.run_angle(350, rotation_angle = -140)

    #back from museum
    inLineCM(cm = -90, threshold = 69)
    turnDeg(-50, threshold = 2)

    #pannels up
    inLineCM(cm = 90, threshold = 50)

    #going for flower
    setPoseEstimate(Pose(0,0,0))
    inLineCM(cm = -90, threshold = 80)
    turnDeg(65.5)
    inLineCM(cm = 120, threshold = 55)

    #doing the flower
    inLineCM(cm = -12, threshold = 2.7)
    turnDeg(109.8, sensitivity = 0.8, threshold = 3)
    turnDeg(100)
    inLineCM(cm = 90, threshold = 78)
    turnDeg(50)
    turnDeg(100)
    
    return 0

def run6():
    inLineCM(cm = 90, correctHeading = False, threshold = 49)
    inLineCM(cm = -60, correctHeading = False, threshold = 43)
    return 0

def run7():
    inLineCM(cm = 30, correctHeading = True, threshold = 28, sensitivity = 1)
    turnDeg(7.9)
    inLineCM(cm = 130, correctHeading = True, threshold = 43)

    turnDeg(-24)
    turnDeg(-24)
    return 0

def loop():
    global upDone, leftDone, rightDone, downDone, middleUpDone, middleLeftDone, middleRightDone, middleDownDone
    global oneTimeUse, run
    zero()

    showcaseOptions()
    while True:
        
        updateAll()
        gamepad.updateButtons()
        
        

        #run 1
        if gamepad.wasJustPressed(Button.UP):
            if not oneTimeUse or (not upDone and run == 1):
                if zeroBeforeEveryRun:
                    zero()
                brick.light.on(Color.GREEN)
                printInProgress()

                run1()

                brick.light.on(Color.RED)
                upDone = True
                run += 1
                showcaseOptions()
                afterEveryRun()




        #run 2
        elif gamepad.wasJustPressed(Button.LEFT):
            if not oneTimeUse or (not leftDone and run == 2):  
                if zeroBeforeEveryRun:
                    zero()
                brick.light.on(Color.GREEN)
                printInProgress()

                run2()

                brick.light.on(Color.RED)
                leftDone = True
                run += 1
                showcaseOptions()
                afterEveryRun()





        #run 3
        elif gamepad.wasJustPressed(Button.RIGHT):
            if not oneTimeUse or (not rightDone and run == 3):
                if zeroBeforeEveryRun:
                    zero()
                brick.light.on(Color.GREEN)
                printInProgress()

                run3()

                brick.light.on(Color.RED)
                rightDone = True
                run += 1
                showcaseOptions()
                afterEveryRun()




        #run 4
        elif gamepad.wasJustPressed(Button.DOWN):
            if not oneTimeUse or (not downDone and run == 4):
                if zeroBeforeEveryRun:
                    zero()
                brick.light.on(Color.GREEN)
                printInProgress()

                run4()

                brick.light.on(Color.RED)
                downDone = True
                run += 1
                showcaseOptions()
                afterEveryRun()



            
        elif gamepad.wasJustPressed(Button.CENTER):
            exit = False
            brick.screen.clear()
            brick.screen.print('middle: ENTERED')
            showcaseOptions(clear = False)

            while not exit:
                updateAll()
                gamepad.updateButtons()

                if gamepad.wasJustPressed(Button.CENTER):
                    exit = True


                #run 5
                if gamepad.wasJustPressed(Button.UP):
                    if not oneTimeUse or (not middleUpDone and run == 5):
                        if zeroBeforeEveryRun:
                            zero()
                        brick.light.on(Color.GREEN)
                        printInProgress()

                        run5()

                        brick.light.on(Color.RED)
                        middleUpDone = True
                        exit = True
                        run += 1
                        showcaseOptions()
                        afterEveryRun()
                        



                #run 6
                elif gamepad.wasJustPressed(Button.LEFT):
                    if not oneTimeUse or (not middleLeftDone and run == 6):  
                        if zeroBeforeEveryRun:
                            zero()
                        brick.light.on(Color.GREEN)
                        printInProgress()

                        run6()

                        brick.light.on(Color.RED)
                        middleLeftDone = True
                        exit = True
                        run += 1
                        showcaseOptions()
                        afterEveryRun()





                #run 7
                elif gamepad.wasJustPressed(Button.RIGHT):
                    if not oneTimeUse or (not middleRightDone and run == 7):
            
                        if zeroBeforeEveryRun:
                            zero()
                        brick.light.on(Color.GREEN)
                        printInProgress()

                        run7()

                        brick.light.on(Color.RED)
                        middleRightDone = True
                        exit = True
                        run += 1
                        showcaseOptions()
                        afterEveryRun()
            

            brick.screen.clear()
            brick.screen.print('middle: EXITED')
            showcaseOptions(clear = False)
'''

