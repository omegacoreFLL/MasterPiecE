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

#some more boolean (True/False) variables, often referenced as 'flags', because they flag(show the user)
#       if something should be done or not. It's a method of modularizing the code, which means the ability
#       to change features really fast (in this case, whith a variable).
#
#I use this concept a lot, but in this case, those variables determine when all encoders, gyro value
#       and position calculations should be reset to 0, for better accuracy
zeroBeforeEveryMove, zeroBeforeEveryRun, zeroBeforeEveryTask, zeroBeforeMotors = False, True, False, False

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
def showcaseOptions():
    if not upDone:
        brick.screen.print(' UP ')
    elif upDone and not leftDone:    
        brick.screen.print(' LEFT ')
    elif leftDone and not middleDone:   
        brick.screen.print(' MIDDLE ')
    elif middleDone and not rightDone:    
        brick.screen.print(' RIGHT ')
    elif rightDone and not downleftDone:    
        brick.screen.print(' DOWN + LEFT ')
    elif downleftDone and not downrightDone:
        brick.screen.print(' DOWN + RIGHT ')

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



# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

#ignore these 2, no actual use
def setOneTimeUse(otu):
    oneTimeUse = otu

def initFont(size, bold):
    coolFont = Font(size, bold)
    brick.screen.set_font(coolFont)

def setPoseEstimate(pose):
    global leftDrive, rightDrive
    global botPose, posL, posR, angle, pastPosL, pastPosR, pastAngle 
    
    leftDrive.reset_angle(angle = 0)
    rightDrive.reset_angle(angle = 0)

    gyro.reset_angle(pose.head)
    botPose.set(pose.x, pose.y, pose.head)

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
    resetFailSwitch(normalizeTimeToTurn(abs(error)))


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
    leftDrive.brake()
    rightDrive.brake()





def inLineCM(cm, threshold = 0.1, sensitivity = 1, correctHeading = False):
    global botPose, isBusy, gyro, angle, distance
    #starts movement
    isBusy = True


    #defining local variables
    updateAll()
    deg = botPose.head
    sign = signum(deg)
    deg = normalizeDegrees(deg)

    pastError = 0
    pastTime = 0
    derivativeTimer = StopWatch()


    #loop for reaching the target state
    while isBusy: 
        updateAll()
        getAngle()


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
def toPositon(target, threshold = 0.1, sensitivity = 1,
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
        turnDeg(target.head)
        #wait(100)
        turnDeg(target.head)

    leftDrive.brake()
    rightDrive.brake()

       




def slewRateLimiter(motor: Motor, speed: float):
    return 0





def inCurveCM(target, keepHeading = False):
    global botPose, dW, leftDrive, rightDrive, isBusy, distance
    updateAll()

    errorX = target.x - botPose.x
    errorY = target.y - botPose.y

    if errorX <= 0:
        signX = -1
        errorX *= -1
    else:
        signX = 1
    
    if errorY <= 0:
        signY = 1
    else:
        signY = -1


    turnAngle = -math.atan2(errorX, errorY)
    theta = 2 * (math.atan2(errorY, errorX) - toRadians(botPose.head))
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
        
    #print('left: ', velL)
    #print('right: ', velR)

    print('arc', arcD)
    print('radius', radius)
    print('       ')
    printPose()
    print('       ')
    print('       ')

    

    #wait(2000)

    isBusy = True
    zeroDistance()

    while isBusy:
        updateAll()

        leftDrive.dc(velR * signX)
        rightDrive.dc(velL * signX)

        getPose()

        if abs(abs(distance) - abs(arcD)) < 1:
            isBusy = False
    

    if not keepHeading:
        turnDeg(target.head)
        turnDeg(target.head)

    leftDrive.brake()
    rightDrive.brake()

    print('       ')
    printPose()
    print('       ')
    print('       ')
        


    




# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



#main loop, which exits only if the 'back' button is pressed on the brick
#we check what button was pressed to know which program should run. Right now used only for testing
def loop():
    global upDone, leftDone, middleDone, rightDone, downleftDone, downrightDone
    global oneTimeUse

    while True:
        #showcaseOptions()
        updateAll()
        gamepad.updateButtons()

        

        


        if gamepad.wasJustPressed(Button.UP):
            if not upDone or not oneTimeUse:
                 #run 1
                if zeroBeforeEveryRun:
                    zero()

                toPositon(Pose(40, -1, 0))
                toPositon(Pose(0, -1, 0), forwards = False)
                
                upDone = True
                

        elif gamepad.wasJustPressed(Button.LEFT):
            if not oneTimeUse or not leftDone:
                #run 2  
                if zeroBeforeEveryRun:
                    zero()

                setPoseEstimate(Pose(0, 0, 0))
                toPositon(Pose(60, 10, 90), keepHeading = False, listOfCommands = [
                    Command(motor = leftTask, runType = 'RUN_ANGLE', speed = 300, value = 100, startPercent = 10, endPercent = 55),
                    Command(motor = leftTask, runType = 'RUN_ANGLE', speed = 300, value = -100, startPercent = 80, endPercent = 90)
                ])

                toPositon(Pose(60, 40, 0), keepHeading = False, forwards = False, listOfCommands = [
                    Command(motor = leftTask, runType = 'RUN_ANGLE', speed = 300, value = 100, startPercent = 10, endPercent = 55),
                    Command(motor = leftTask, runType = 'RUN_ANGLE', speed = 300, value = -100, startPercent = 80, endPercent = 90)
                ])


                leftDone = True;

        elif gamepad.wasJustPressed(Button.CENTER):
            if not oneTimeUse or not middleDone:
                #run 3
                if zeroBeforeEveryRun:
                    zero()

                zero()    
                toPositon(target = Pose(30, 0, 90), keepHeading = False, forwards = False)
                toPositon(target = Pose(30, 30, 180), keepHeading = False, forwards = True)
                toPositon(target = Pose(0, 30, -90), keepHeading = False, forwards = False)
                toPositon(target = Pose(0, 0, 90), keepHeading = False, forwards = True) 
                getPose()
                    

                middleDone = True;
            
        elif gamepad.wasJustPressed(Button.RIGHT):
            if not oneTimeUse or not rightDone:
                #run 4
                if zeroBeforeEveryRun:
                    zero()

                inCurveCM(Pose(40, 20, -90), keepHeading = True)
                #inCurveCM(Pose(20, 0, 0), keepHeading = False)
                #inCurveCM(Pose(10, 90, -90))

                rightDone = True
            
        elif gamepad.wasJustPressed(Button.DOWN):
            exit = False;

            while exit == False:
                updateAll()
                gamepad.updateButtons()


                if gamepad.wasJustPressed(Button.UP):
                    exit = True;

                elif gamepad.wasJustPressed(Button.LEFT):
                    if not oneTimeUse or not downleftDone:
                        #run 5
                        if zeroBeforeEveryRun:
                            zero()

                        leftTask.run_angle(500, -120)

                        downleftDone = True
                        exit = True
                        
                elif gamepad.wasJustPressed(Button.RIGHT):
                    if not oneTimeUse or not downrightDone:
                        #run 6
                        if zeroBeforeEveryRun:
                            zero()

                        leftTask.run_angle(500, 120)

                        downrightDone = True
                        exit = True  


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


#Actual code goes here ----> (asta-i sageata in jos lmao)
loop() 
