#!/usr/bin/env pybricks-micropython

#importing all the pybricks stuff

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

#importing the python-predefined class 'math', with usefull functions like sin() & cos()
import math
#also importing my own file with transformation functions (ex: degrees to radians) +
#       Point & Pose classes, a compact way to reference position in 2D, respectively 3D space
from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from BetterClasses.MotorEx import *
from TankDrive.constants import *


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


#remembering current and past values for:
#         - LEFT & RIGHT drive motor encoders
#         - robot orientation aka gyro-given angle
#         - reference = a conventional notation I chose for the position of the middle of the 
#           distance between wheels, needed for further calculation
#
#with those values, we can calculate deltas, to sum up. You may have seen this notation from 
#       physics: Δt = t1 - t0
posL, posR, angle, pastPosL, pastPosR, pastAngle, deltaL, deltaR, deltaAngle, deltaReference, distance = (
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
)


#the -Pose- type variables containg info about the current & previous robot poses (past one isn't used, yet)
botPose, pastBotPose = Pose(0, 0, 0), Pose(0, 0, 0)

voltage = 0
currentThreshold = 0


#variable which dictates when the robot has reached the target (if it's moving on the field is True)
isBusy = False


#you know that v = d/t, which you can also write with deltas: v = Δd / Δt . We already defined Δd, so here's
#       the rest
#
#'Fail switch' is a concept used a lot in FTC autonomus movements, which accounts for known or logical
#       issues with the system. For ex, if you know that you should have a clear way in front, and the 
#       motors are constrained, you know you've hit something and you stop. This is usually referred to as  
#       current sensing. I made a general timer to maybe use it in more situations, like the one above, but
#       for know is's purpose is to break out the turning function if it turns for too much time (it's obvious
#       that you don't need 5 second to turn 90deg lol)
velTimer = StopWatch()
failSwitchTimer = StopWatch()
time, vel, pastTime, pastVel, deltaTime, deltaVel = 0, 0, 0, 0, 0, 0


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


#all hardware HERE:
brick = EV3Brick()
brick.light.on(Color.RED)

leftTask = Motor(ltPort, positive_direction = Direction.COUNTERCLOCKWISE)
leftDrive = Motor(ldPort)
leftColor = ColorSensor(lcPort)

rightTask = Motor(rtPort, positive_direction = Direction.COUNTERCLOCKWISE)
rightDrive = Motor(rdPort)
rightColor = ColorSensor(rcPort)

gyro = GyroSensor(gyroPort)

gamepad = ButtonEx()


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


#Math


#dictates if you should stop or not based on the elapsed time since the start of the movement
def failSwitchStop():
    if msToS(failSwitchTimer.time()) > failSwitchTime:
        return True 
    return False


def normalizeVoltage(power):
    return power * maxVoltage / voltage




#resetting the timer and also setting the time provided to complete a movement
def resetFailSwitch(fst):
    global failSwitchTime, failSwitchTimer

    failSwitchTime = fst
    failSwitchTimer.reset()




#the calculations for odometry. Odometry is a concept which basically describes the process of finding the
#       position relative to a reference system, using some input, usually motor encoder ticks or speed
#
#check these very well-explained calculations to see where the formulas came from:
#       https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
def updateDeltas():
    global posL, posR, angle
    global pastPosL, pastPosR, pastAngle
    global deltaL, deltaR, deltaAngle, deltaReference

    global time, vel
    global pastTime, pastVel
    global deltaTime

    posL = encoderTicksToCM(leftDrive.angle())
    posR = encoderTicksToCM(rightDrive.angle())
    angle = normalizeDegrees(deg = -gyro.angle())
    time = velTimer.time()

    deltaL = posL - pastPosL
    deltaR = posR - pastPosR
    deltaAngle = angle - pastAngle
    deltaTime = time - pastTime

    deltaReference = (deltaL + deltaR) / 2

    pastPosL = posL
    pastPosR = posR
    pastAngle = angle
    pastTime = time
    pastVel = vel

    vel = (deltaL + deltaR) / deltaTime * 100


#here you update the actual position of the robot. The new X and Y coordinates can be easely calculated
#       by getting the sin & cos values of the relative distance already calculated. We then add this to the
#       past value. The new heading is basically the current heading, as the gyro provides a more accurate
#       value than calculating it with the odometry. We then sum all of these to the past values at a fast 
#       loop frequency
def updatePose():
    global distance 
    
    rad = normalizeRadians(toRadians(pastAngle + deltaAngle / 2))

    newX = botPose.x + deltaReference * math.cos(rad)
    newY = botPose.y + deltaReference * math.sin(rad)
    newHead = normalizeDegrees(angle)

    distance += deltaReference

    botPose.set(newX, newY, newHead)






#combines all methods in just one method call, for simplicity, also getting the battery voltage
def updateAll(): 
    global voltage

    updateDeltas()
    updatePose()

    voltage = brick.battery.voltage() / 1000


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


#Telemetry (print things on ev3's screen, mostly for debugging)
def showcaseOptions(clear = True):
    global run
    if clear:
        brick.screen.clear()
        brick.screen.print('               ')

    if run < 9:
        brick.screen.print(' next run:', run)
        brick.screen.print('               ')
    else:
        brick.screen.print('                ')
        brick.screen.print('                ')
        brick.screen.print('      DONE      ')
        brick.screen.print('                ')
        brick.screen.print('                ')

    if not upDone:
        brick.screen.print(' UP ')
    elif upDone and not leftDone:    
        brick.screen.print(' LEFT ')
    elif leftDone and not rightDone:    
        brick.screen.print(' RIGHT ')
    elif rightDone and not downDone:    
        brick.screen.print(' DOWN ')
    elif downDone and not middleUpDone:   
        brick.screen.print(' MIDDLE + UP ')
    elif middleUpDone and not middleLeftDone:
        brick.screen.print(' MIDDLE + LEFT ')
    elif middleLeftDone and not middleRightDone:
        brick.screen.print(' MIDDLE + RIGHT ')
    elif middleRightDone and not middleDownDone:
        brick.screen.print(' MIDDLE + DOWN ')



def showcaseDeltas():
    brick.screen.print('delta L: ', deltaL)  
    brick.screen.print('delta R: ', deltaR)  
    brick.screen.print('delta angle: ', deltaAngle)

def getAngle():
    brick.screen.print('angle (deg): ', angle)

def getVel():
    brick.screen.print('vel: ', vel)

def getVoltage():
    brick.screen.print('V: ', voltage)

def getPose():
    brick.screen.print('x: ', botPose.x)
    brick.screen.print('y: ', botPose.y)
    brick.screen.print('deg: ', botPose.head)

def printPose():
    print('x: ', botPose.x)
    print('y: ', botPose.y)
    print('deg: ', botPose.head)

def printInProgress():
    global run
    brick.screen.clear()
    brick.screen.print('                          ')
    brick.screen.print('                          ')
    brick.screen.print('run', run)
    brick.screen.print('  in progress...')



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

#ignore these 2, no actual use
def setOneTimeUse(otu):
    global oneTimeUse
    oneTimeUse = otu

def initFont(size, bold):
    coolFont = Font(size, bold)
    brick.screen.set_font(coolFont)

def setPoseEstimate(pose):
    global leftDrive, rightDrive, offset
    global botPose, posL, posR, angle, pastPosL, pastPosR, pastAngle 
    
    leftDrive.reset_angle(angle = 0)
    rightDrive.reset_angle(angle = 0)

    gyro.reset_angle(-normalizeDegrees(pose.head))
    botPose.set(pose.x, pose.y, normalizeDegrees(pose.head))

    posL, posR, angle, pastPosL, pastPosR, pastAngle = (
        0, 0, 0, 0, 0, 0
    )


#resetting all robot values to 0
def zero(zeroTaskMotors = False):
    global leftTask, rightTask, leftDrive, rightDrive
    global botPose, posL, posR, angle, pastPosL, pastPosR, pastAngle

    if zeroTaskMotors:
        leftTask.reset_angle(angle = 0)
        rightTask.reset_angle(angle = 0)

    leftDrive.reset_angle(angle = 0)
    rightDrive.reset_angle(angle = 0)

    gyro.reset_angle(0)
    botPose.set(0, 0, 0)
    posL, posR, angle, pastPosL, pastPosR, pastAngle = (
        0, 0, 0, 0, 0, 0
    )




#resetting just the drive encoders to 0
def zeroOdometry():
    global leftDrive, rightDrive

    leftDrive.reset_angle(angle = 0)
    rightDrive.reset_angle(angle = 0)


def zeroDistance():
    global distance
    
    distance = 0

   
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

#turning a specified number of radians (same as turning in degrees, so we made a conversion)
def turnRad(rad, threshold = 0.1):
    turnDeg(toDegrees(normalizeRadians(rad)), threshold)




#turning any number of degrees [0, 360] (or [0, 2π] in radians)
def turnDeg(deg, threshold = 0.1, sensitivity = 1):
    global botPose, isBusy, gyro
    #starts movement
    isBusy = True


    #saving the sign of desired posion, because we then normalize it in [0, 360] range (ex: -90 --> 270)
    sign = signum(deg)
    deg = normalizeDegrees(deg)


    #we first find the initial error (distance between position and target) to know how many seconds to give
    #for this action
    updateAll()


    #find shortest path to target. If you are a 0° and want to go to 270° right, it's faster to go 90° left
    if (abs(botPose.head - deg) < 360 - abs(botPose.head - deg)):
        error = botPose.head - deg
    else: error = signum(botPose.head - deg) * 360 - (botPose.head - deg)


    #calculating max time accorded
    resetFailSwitch(normalizeTimeToTurn(abs(error)) / sensitivity)


    #derivative term local variables: D = Δe / Δt, where e = error and t = time)
    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()
    loopsInTarget = 0


    #loop
    while isBusy:
        updateAll()

        #copy-paste from above
        if (abs(botPose.head - deg) <= 360 - abs(botPose.head - deg)):
            error = botPose.head - deg
        else: error = -1 * (signum(botPose.head - deg) * 360 - (botPose.head - deg))

        currentTime = derivativeTimer.time()

        #check if you reached the target -> stop, else calculate motor powers
        if (abs(error) <= threshold and abs(pastError) <= threshold) or failSwitchStop():
            loopsInTarget = loopsInTarget + 1
        else: 
            loopsInTarget = 0

            #power = proportional * kP + derivative * kD
            d = (error - pastError) / (currentTime - pastTime)
            power = normalizeVoltage(error * kP_head + d * kD_head + signum(error) * kS_head)

            #to turn in place, motor should spin opposite from one another
            leftDrive.dc(-power * sensitivity)
            rightDrive.dc(power * sensitivity)

        #update values
        pastTime = currentTime
        pastError = error

        if loopsInTarget == 17:
            isBusy = False
    
    #stop motors when done
    leftDrive.dc(0)
    rightDrive.dc(0)
    leftDrive.brake()
    rightDrive.brake()





def inLineCM(cm, threshold = 0.1, sensitivity = 1, correctHeading = True):
    global botPose, isBusy, gyro, angle, distance
    #starts movement
    isBusy = True


    #defining local variables
    updateAll()
    deg = botPose.head
    deg = normalizeDegrees(deg)

    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()

    zeroDistance()

    #loop for reaching the target state
    while isBusy: 
        updateAll()


        #separate value for heading correction, same logic as in -turnDeg- method
        correction = 0
        if correctHeading:
            if (abs(botPose.head - deg) <= 360 - abs(botPose.head - deg)):
                headError = botPose.head - deg
            else: headError = -1 * (signum(botPose.head - deg) * 360 - (botPose.head - deg))

            currentTime = derivativeTimer.time()

            #this time don't break out of the loop if heading is right. just to nothing
            #tho if heading is off, calculate the correction power
            if (abs(headError) > threshold and abs(pastError) <= threshold) or failSwitchStop():
                d = (headError - pastError) / (currentTime - pastTime)
                correction = normalizeVoltage(headError * kP_head + d * kD_head)
            
            pastTime = currentTime
            pastError = headError
        

        #the p controller for going straight this time
        error = distance - cm

        #if target is reached, stop
        if abs(error) <= threshold:
            isBusy = False
        else:
            #the power for going straight calculated by the controller, accounting for static friction too
            #total power is the sum of the forward power and correction
            power = normalizeVoltage(-error * kP_forw) + signum(cm) * kS_forw
            leftDrive.dc(clipMotor(normalizeVoltage(power - correction) * sensitivity))
            rightDrive.dc(clipMotor(normalizeVoltage(power + correction) * sensitivity))

    
    #stop motors when done
    leftDrive.dc(0)
    rightDrive.dc(0)
    leftDrive.brake()
    rightDrive.brake()




'''
COORDINATE REFERENCE FIELD:
               Λ +X
               |
               |
               |
               |
               |
               |             +Y
---------------o-------------->
               |
               |
               |
               |
               |
               |   

'''
def toPosition(target, threshold = 0.1, sensitivity = 1, headSensitivity = 1, headThreshold = 0.1,
            keepHeading = False, forwards = True, correctHeading = True,
            listOfCommands = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
    global botPose, leftDrive, rightDrive, isBusy, distance

    queuedCommands = not (len(listOfCommands) > 10)
    updateAll()

    yError = target.x - botPose.x 
    xError = target.y - botPose.y 
    
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

    if queuedCommands:
        for com in listOfCommands:
            com.startPercent = com.startPercent / 100 * needToTravelDist
            com.endPercent = com.endPercent / 100 * needToTravelDist

    #twice for better accuracy
    turnDeg(deg)
    wait(100)
    turnDeg(deg)

    isBusy = True 
    isFirstLoop = True
    zeroDistance()

    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()


    while isBusy:
        updateAll()

        #separate value for heading correction, same logic as in -turnDeg- method
        correction = 0
        if correctHeading:
            if (abs(botPose.head - deg) <= 360 - abs(botPose.head - deg)):
                headError = botPose.head - deg
            else: headError = -1 * (signum(botPose.head - deg) * 360 - (botPose.head - deg))

            currentTime = derivativeTimer.time()

            #this time don't break out of the loop if heading is right. just to nothing
            #tho if heading is off, calculate the correction power
            if abs(headError) > threshold and abs(pastError) <= threshold:
                d = (headError - pastError) / (currentTime - pastTime)
                correction = normalizeVoltage(headError * kP_head + d * kD_head)


        error = abs(distance) - needToTravelDist

        #if target is reached, stop
        if abs(error) <= threshold:
            isBusy = False
        else:
            #the power for going straight calculated by the controller, accounting for static friction too
            #total power is the sum of the forward power and correction
            power = -sign * (normalizeVoltage(-error * kP_forw) - signum(error) * kS_forw)
        
            leftDrive.dc(clipMotor(normalizeVoltage(power - correction) * sensitivity))
            rightDrive.dc(clipMotor(normalizeVoltage(power + correction) * sensitivity))


        if queuedCommands:
            for com in listOfCommands:
                if abs(abs(distance) - abs(com.startPercent)) < 1:
                    com.start()
                elif abs(abs(distance) - abs(com.endPercent)) < 1:
                    com.stop()

        
    if not keepHeading:
        turnDeg(target.head, sensitivity = headSensitivity, threshold = headThreshold)
        wait(100)
        turnDeg(target.head, sensitivity = headSensitivity, threshold = headThreshold)

    leftDrive.dc(0)
    rightDrive.dc(0)
    leftDrive.brake()
    rightDrive.brake()

       




def slewRateLimiter(motor: Motor, speed: float):
    return 0





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



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



def run1():
    inLineCM(cm = 4.2, correctHeading = True, threshold = 2) 
    turnDeg(-48)
    inLineCM(cm = 28.9, correctHeading = True, threshold = 3)
    rightTask.run_angle(500, rotation_angle = 200)
    turnDeg(-48, sensitivity = 0.65)
    inLineCM(cm = 22.5,  correctHeading = True, threshold = 5)
    turnDeg(botPose.head - 6, sensitivity = 0.4, threshold = 0.2)
    
    leftTask.dc(-100)
    wait(1700)
    leftTask.dc(0)

    inLineCM(cm = -17,  correctHeading = True, threshold = 13)
    turnDeg(-52.7)
    inLineCM(cm = -17, threshold = 8)
    turnDeg(63)
    inLineCM(cm = -21,  correctHeading = True, threshold = 2)
    printPose()

    setPoseEstimate(Pose(0, 0, botPose.head))
    toPosition(Pose(12.9, -95.69, 90), headSensitivity = 9, forwards = False, threshold = 33, keepHeading = True)
    turnDeg(70)
    inLineCM(cm = 16, correctHeading = False, threshold = 3)
    inLineCM(cm = -5, correctHeading = False, threshold = 1.2)
    leftTask.run_angle(500, rotation_angle = 1199)

    turnDeg(70)
    turnDeg(70)
    inLineCM(cm = - 87, threshold = 4)
    
    return 0

def run2():
    inLineCM(cm = 85, correctHeading = True, threshold = 51)

    leftTask.run_until_stalled(500, then = Stop.HOLD)

    inLineCM(cm = -61, threshold = 55.5)
    turnDeg(-70, threshold = 5)
    turnDeg(-42, threshold = 5)

    leftTask.run_angle(-500, rotation_angle = 100)
    
    turnDeg(-5, threshold = 2)
    inLineCM(cm = -75, threshold = 44)

    return 0

def run3():
    inLineCM(cm = -45, threshold = 41)
    turnDeg(37.5, threshold = 1)
    #turnDeg(34.5, threshold = 1)
    print('deg', botPose.head)
    inLineCM(cm = -50, correctHeading = True, threshold = 11.8)
    inLineCM(cm = 2.5, correctHeading = False, threshold = 2)
    setPoseEstimate(Pose(0,0,0))

    turnDeg(30.5, sensitivity = 0.4, threshold = 9.5)

    inLineCM(cm = 90, correctHeading = False, threshold = 43)
    return 0

def run4():
    inLineCM(cm = 30, correctHeading = True, threshold = 28, sensitivity = 1)
    turnDeg(-10)
    inLineCM(cm = 97, correctHeading = True, threshold = 64)
    turnDeg(30, threshold = 28 )
    #inLineCM(cm = -47, threshold = 44)
    #inLineCM(cm = 57, threshold = 46)

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
    rightTask.run_angle(500, rotation_angle = -140)

    #back from museum
    inLineCM(cm = -90, threshold = 69)
    turnDeg(-43, threshold = 2)

    #pannels up
    inLineCM(cm = 90, threshold = 50)
    #inLineCM(cm = -90, threshold = 85)
    #inLineCM(cm = 90, threshold = 69)

    #going for flower
    setPoseEstimate(Pose(0,0,0))
    inLineCM(cm = -90, threshold = 75)
    turnDeg(56)
    inLineCM(cm = 120, threshold = 55)

    #doing the flower
    inLineCM(cm = -10, threshold = 2.5)
    turnDeg(110, sensitivity = 0.8, threshold = 3)
    inLineCM(cm = 90, threshold = 80)
    turnDeg(50)
    
    return 0

def run6():
    inLineCM(cm = 90, correctHeading = False, threshold = 49)
    inLineCM(cm = -60, correctHeading = False, threshold = 43)
    return 0

def run7():
    inLineCM(cm = 30, correctHeading = True, threshold = 28, sensitivity = 1)
    turnDeg(-7)
    inLineCM(cm = 130, correctHeading = True, threshold = 43)

    turnDeg(-24)
    turnDeg(-24)
    return 0

def run8():

   return 0




# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



#main loop, which exits only if the 'back' button is pressed on the brick
#we check what button was pressed to know which program should run. Right now used only for testing
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
            if not upDone or (not oneTimeUse and run == 1):
                if zeroBeforeEveryRun:
                    zero()
                brick.light.on(Color.GREEN)
                printInProgress()

                run1()

                brick.light.on(Color.RED)
                upDone = True
                run += 1
                showcaseOptions()




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




                #run 8
                elif gamepad.wasJustPressed(Button.DOWN):
                    if not oneTimeUse or (not middleDownDone and run == 8):
                        if zeroBeforeEveryRun:
                            zero()
                        brick.light.on(Color.GREEN)
                        printInProgress()

                        run8()

                        brick.light.on(Color.RED)
                        middleDownDone = True
                        exit = True
                        run += 1
                        showcaseOptions()
            
            brick.screen.clear()
            brick.screen.print('middle: EXITED')
            showcaseOptions(clear = False)

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


#Actual code goes here ----> (asta-i sageata in jos lmao)
loop() 
