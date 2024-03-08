#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from TankDrive.constants import *
import math


angle, pastAngle, deltaAngle, time, pastTime, deltaTime = 0, 0, 0, 0, 0, 0

sec = 5
maxAngularVelocity = 0.0
angularVel = 0.0

runTimer = StopWatch()
gamepad = ButtonEx()

brick = EV3Brick()
brick.light.on(Color.RED)

leftDrive = Motor(ldPort)
rightDrive = Motor(rdPort)

gyro = GyroSensor(gyroPort)


def updateDeltas():
    global angle, time
    global pastAngle, pastTime
    global deltaAngle, deltaTime
    global angularVel

    angle = gyro.angle()
    time = runTimer.time()

    deltaAngle = angle - pastAngle
    deltaTime = time - pastTime

    angularVel = deltaAngle / deltaTime

    pastAngle = angle
    pastTime = time


print('Robot will turn right at full speed for', sec, 'seconds')
print('Press CENTER button to start: ')

hasFinished = False
hasStarted = False
while not hasFinished:
    gamepad.updateButtons()
    
    if gamepad.wasJustPressed(Button.CENTER) and not hasStarted:
        hasStarted = True
        runTimer.reset()
        gyro.reset_angle(0)


        leftDrive.dc(100)
        rightDrive.dc(-100)

    if hasStarted:
        updateDeltas()
        maxAngularVelocity = max(angularVel, maxAngularVelocity)


    if hasStarted and msToS(runTimer.time()) > sec:
        hasFinished = True

        leftDrive.brake()
        rightDrive.brake()


print('             ')
print('Max Angular Velocity is:', maxAngularVelocity, 'deg / s')
print('                        ', toRadians(maxAngularVelocity), 'rad / s')