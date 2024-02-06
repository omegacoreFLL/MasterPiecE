from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

from BetterClasses.mathEx import * 
from BetterClasses.ButtonsEx import *
from TankDrive.constants import *

import math

class Action:
    def __init_(self, action):
        self.RUN = self.RUN_TIME = self.RUN_ANGLE = self.RUN_TARGET = self.RUN_UNTIL_STALLED = self.DC = False

        if action == 'RUN':
            self.RUN = True
        elif action == 'RUN_TIME':
            self.RUN_TIME = True
        elif action == 'RUN_ANGLE':
            self.RUN_ANGLE = True
        elif action == 'RUN_TARGET':
            self.RUN_TARGET = True
        elif action == 'RUN_UNTIL_STALLED':
            self.RUN_UNTIL_STALLED = True
        elif action == 'DC':
            self.DC = True


class Command:
    def __init__(self, motor: Motor, runType, speed, startPercent, value = 0, oneTimeUse = False, endPercent = 100):
        self.motor = motor
        self.runType = runType
        self.value = value
        self.speed = speed
        
        self.numberOfCalls = 0
        self.oneTimeUse = oneTimeUse
        
        self.startPercent = startPercent
        self.endPercent = endPercent
    
    
    def start(self):
        if not self.oneTimeUse or self.numberOfCalls < 1:

            if self.runType == 'RUN':
                self.motor.run(speed = self.speed)
            elif self.runType == 'RUN_TIME':
                self.motor.run_time(speed = self.speed, time = self.value, wait = False)
            elif self.runType == 'RUN_ANGLE':
                self.motor.run_angle(speed = self.speed, rotation_angle = self.value, wait = False)
            elif self.runType == 'RUN_TARGET':
                self.motor.run_target(speed = self.speed, target_angle = self.value, wait = False)
            elif self.runType == 'RUN_UNTIL_STALLED':
                self.motor.run_until_stalled(speed = self.speed, then = Stop.BRAKE)
            elif self.runType == 'DC':
                self.motor.dc(duty = self.speed)

        self.numberOfCalls += 1
    
    def stop(self, stopType = Stop.BRAKE):
        if self.runType == 'RUN' or self.runType == 'DC':
            if stopType == Stop.BRAKE:
                self.motor.brake()
            elif stopType == Stop.HOLD:
                self.motor.hold()
            elif stopType == Stop.COAST:
                self.motor.coast()

    
    
