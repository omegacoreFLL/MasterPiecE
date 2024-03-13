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
from BetterClasses.TelemetryEx import *
from TankDrive.odometry import *
from TankDrive.constants import *
from Controllers.RunController import *


class Robot:
    def __init__(self):
        self.brick = EV3Brick()

        self.leftTask = Motor(ltPort, positive_direction = Direction.COUNTERCLOCKWISE)
        self.leftDrive = Motor(ldPort)
        self.leftColor = ColorSensor(lcPort)

        self.rightTask = Motor(rtPort, positive_direction = Direction.COUNTERCLOCKWISE)
        self.rightDrive = Motor(rdPort)
        self.rightColor = ColorSensor(rcPort)

        self.gyro = GyroSensor(gyroPort)
        
        self.gamepad = ButtonEx(self.brick)
        self.telemetry = Telemetry(self.brick)
        self.run_control = RunController(self.gamepad, self.brick, self.telemetry)
        self.localizer = TwoWheelLocalizer(self.leftDrive, self.rightDrive, self.gyro, upside_down_gyro = True)

        self.led_control = LedEx(self.brick)
        self.led_control.addTakeYourHandsOffColor(None).build()

        self.failSwitchTimer = StopWatch()
        self.voltage = 0
        self.failSwitchTime = 0

        self.lastLeftSpeed = 0
        self.lastRightSpeed = 0
        self.is_stopped = True
        self.acceleration_timer = StopWatch()
    


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
    

        self.brick.screen.print(message)
    
    def slewRateLimiter(self, targetSpeed):
        if self.is_stopped:
            self.acceleration_timer.reset()
            self.is_stopped = False
        
        acceleration = (msToS(self.acceleration_timer.time()) / acceleration_interval) * acceleration_dc
        if abs(acceleration) < abs(targetSpeed):
            return acceleration * signum(targetSpeed)
        return targetSpeed
        



    def setWheelPowers(self, left, right, sensitivity = 1, accelerating = False):
        if accelerating:
            left = self.slewRateLimiter(left)
            right = self.slewRateLimiter(right)

        self.leftDrive.dc(clipMotor(self.normalizeVoltage(left) * sensitivity))
        self.rightDrive.dc(clipMotor(self.normalizeVoltage(right) * sensitivity))

        if left == 0 and right == 0:
            self.is_stopped = True
        else: self.is_stopped = False
    
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



    def update(self, isBusy = False):
        self.voltage = self.brick.battery.voltage() / 1000
        self.localizer.update()
        self.run_control.update()

        if isBusy:
            self.led_control.in_progress()
        elif self.run_control.entered_center:
            self.led_control.entered_center()
        elif self.run_control.done():
            self.led_control.off()
        else: self.led_control.not_started()
        


    def showcaseAngle(self):
        self.telemetry.addData('angle (deg): ', self.localizer.angle)

    def showcaseVel(self):
        self.telemetry.addData('vel: ', self.localizer.getVelocity())

    def showcaseVoltage(self):
        self.telemetry.addData('V: ', self.voltage)

    def showcasePose(self):
        self.telemetry.addData('x: ', self.localizer.getPoseEstimate().x)
        self.telemetry.addData('y: ', self.localizer.getPoseEstimate().y)
        self.telemetry.addData('deg: ', self.localizer.getPoseEstimate().head)

    def showcaseDeltas(self):
        self.telemetry.addData('delta L: ', self.localizer.deltaL)  
        self.telemetry.addData('delta R: ', self.localizer.deltaR)  
        self.telemetry.addData('delta angle: ', self.localizer.deltaAngle)
    
    def printPose(self):
        print('x: ', self.localizer.getPoseEstimate().x)
        print('y: ', self.localizer.getPoseEstimate().y)
        print('deg: ', self.localizer.getPoseEstimate().head)

    def printVel(self):
        print('velocity: ', self.localizer.getVelocity() * 100)
    




        