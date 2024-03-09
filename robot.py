#importing all the pybricks stuff

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile

import math

from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from BetterClasses.MotorEx import *
from BetterClasses.LedEx import *
from TankDrive.odometry import *
from TankDrive.constants import *


class robot:
    def __init__(self):
        self.brick = EV3Brick()

        self.leftTask = Motor(ltPort, positive_direction = Direction.COUNTERCLOCKWISE)
        self.leftDrive = Motor(ldPort)
        self.leftColor = ColorSensor(lcPort)

        self.rightTask = Motor(rtPort, positive_direction = Direction.COUNTERCLOCKWISE)
        self.rightDrive = Motor(rdPort)
        self.rightColor = ColorSensor(rcPort)

        self.gyro = GyroSensor(gyroPort)
        
        self.gamepad = ButtonEx()
        self.localizer = TwoWheelLocalizer(self.leftDrive, self.rightDrive, self.gyro, upside_down_gyro = True)

        self.led_control = LedEx(self.brick)
        self.led_control.build()


        self.failSwitchTimer = StopWatch()
        self.voltage = 0
        self.failSwitchTime = 0
    


    def normalizeVoltage(self, power):
        if self.voltage != 0:
            return power * maxVoltage / self.voltage
        return power
    
    def resetFailSwitch(self, fst):
        self.failSwitchTimer.reset()
        self.failSwitchTime = fst
    
    def failSwitchStop(self):
        if msToS(self.failSwitchTimer.time()) > self.failSwitchTime:
            return True 
        return False
    
    def telemetry(self, *message):
        self.brick.screen.print(message)
    
    def slewRateLimiter(self, targetSpeed):
        return 0



    def setWheelPowers(self, left, right, sensitivity = 1, accelerating = False):
        self.leftDrive.dc(clipMotor(self.normalizeVoltage(left) * sensitivity))
        self.rightDrive.dc(clipMotor(self.normalizeVoltage(right) * sensitivity))
    
    def setDriveTo(self,stop_type):
        if stop_type == Stop.COAST:        
            self.leftDrive.stop()
            self.rightDrive.stop()
        
        elif stop_type == Stop.BRAKE:
            self.leftDrive.brake()
            self.rightDrive.brake()
        
        elif stop_type == Stop.HOLD:
            self.leftDrive.hold()
            self.rightDrive.hold()



    def zeroTaskMotors(self, resetLeft = True, resetRight = True):
        if resetLeft:
            self.leftTask.reset_angle(0)
        if resetRight:
            self.rightTask.reset_angle(0)



    def update(self):
        self.voltage = self.brick.battery.voltage() / 1000
        self.localizer.update()
        self.gamepad.updateButtons()
        


    def getAngle(self):
        self.telemetry('angle (deg): ', self.localizer.angle)

    def getVel(self):
        self.telemetry('vel: ', self.localizer.getVelocity())

    def getVoltage(self):
        self.telemetry('V: ', self.voltage)

    def getPose(self):
        self.telemetry('x: ', self.localizer.getPoseEstimate().x)
        self.telemetry('y: ', self.localizer.getPoseEstimate().y)
        self.telemetry('deg: ', self.localizer.getPoseEstimate().head)

    def printPose(self):
        print('x: ', self.localizer.getPoseEstimate().x)
        print('y: ', self.localizer.getPoseEstimate().y)
        print('deg: ', self.localizer.getPoseEstimate().head)

    def showcaseInProgress(self):
        self.brick.screen.clear()
        self.telemetry('                          ')
        self.telemetry('                          ')
        self.telemetry('run', run)
        self.telemetry('  in progress...')
    
    def showcaseDeltas(self):
        self.telemetry('delta L: ', self.localizer.deltaL)  
        self.telemetry('delta R: ', self.localizer.deltaR)  
        self.telemetry('delta angle: ', self.localizer.deltaAngle)
    
    def showcaseOptions(self, clear = True):
        if clear:
            self.brick.screen.clear()
            self.telemetry('               ')

        if oneTimeUse:
            if run < 8:
                self.telemetry(' next run:', run)
                self.telemetry('               ')
            else:
                self.telemetry('                ')
                self.telemetry('                ')
                self.telemetry('      DONE      ')
                self.telemetry('                ')
                self.telemetry('                ')

            if not upDone:
                self.telemetry(' UP ')
            elif upDone and not leftDone:    
                self.telemetry(' LEFT ')
            elif leftDone and not rightDone:    
                self.telemetry(' RIGHT ')
            elif rightDone and not downDone:    
                self.telemetry(' DOWN ')
            elif downDone and not middleUpDone:   
                self.telemetry(' MIDDLE + UP ')
            elif middleUpDone and not middleLeftDone:
                self.telemetry(' MIDDLE + LEFT ')
            elif middleLeftDone and not middleRightDone:
                self.telemetry(' MIDDLE + RIGHT ')




        