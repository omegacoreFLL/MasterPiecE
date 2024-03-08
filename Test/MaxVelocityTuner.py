#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from TankDrive.constants import *
import math


posL, posR, pastPosL, pastPosR, deltaL, deltaR = 0, 0, 0, 0, 0, 0
time, pastTime, deltaTime = 0, 0, 0

sec = 5
maxVelocity = 0.0
vel = 0.0

runTimer = StopWatch()
gamepad = ButtonEx()

brick = EV3Brick()
brick.light.on(Color.RED)

leftDrive = Motor(ldPort)
rightDrive = Motor(rdPort)


def updateDeltas():
    global posL, posR, time
    global pastPosL, pastPosR, pastTime
    global deltaL, deltaR, deltaTime
    global vel

    posL = encoderTicksToCM(leftDrive.angle())
    posR = encoderTicksToCM(rightDrive.angle())
    time = runTimer.time()

    deltaL = posL - pastPosL
    deltaR = posR - pastPosR
    deltaTime = time - pastTime

    deltaReference = (deltaL + deltaR) / 2
    vel = deltaReference / deltaTime

    pastPosL = posL
    pastPosR = posR
    pastTime = time


print('Robot will move forwards at full speed for', sec, 'seconds')
print('Press CENTER button to start: ')

hasFinished = False
hasStarted = False
while not hasFinished:
    gamepad.updateButtons()

    if gamepad.wasJustPressed(Button.CENTER) and not hasStarted:
        hasStarted = True
        runTimer.reset()


        leftDrive.dc(100)
        rightDrive.dc(100)
    
    if hasStarted:
        updateDeltas()
        maxVelocity = max(vel, maxVelocity)


    if hasStarted and msToS(runTimer.time()) > sec:
        hasFinished = True

        leftDrive.brake()
        rightDrive.brake()


print('             ')
print('Max Velocity is:', maxVelocity, 'cm / s')