from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Stop
from pybricks.tools import wait, StopWatch

from BetterClasses.mathEx import * 
from BetterClasses.ButtonsEx import *
from BetterClasses.ErrorEx import *
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
        else: raise Exception("""not a valid 'action'. Choose one of these options:
                            - RUN
                            - RUN_TIME
                            - RUN_ANGLE
                            - RUN_TARGET
                            - RUN_UNTIL_STALLED
                            - DC
        """)

class Command:
    def __init__(self, motor, run_type, speed, start_percent, value = 0, one_time_use = False, end_percent = 100):
        ErrorEx.isType(motor, "motor", Motor)
        ErrorEx.isType(run_type, "run_type", Action)
        ErrorEx.isType(speed, "speed", [int, float])
        ErrorEx.isType(start_percent, "start_percent", [int, float])
        ErrorEx.isType(value, "value", [int, float])
        ErrorEx.iaType(one_time_use, "one_time_use", bool)
        ErrorEx.isType(end_percent, "end_percent", [int, float])
        

        self.__motor = motor
        self.__run_type = run_type
        self.__value = value
        self.__speed = speed
        
        self.__numberOfCalls = 0
        self.__one_time_use = one_time_use
        
        self.__start_percent = start_percent
        self.__endPercent = end_percent

        self.__start_distance = 0
        self.__end_distance = 0
        self.__has_build = False
    
    def calculate(self, total_distance):
        ErrorEx.isType(total_distance, "total_distance", [int, float])

        self.__start_distance = self.__start_percent / 100 * total_distance
        self.__end_distance = self.__end_percent / 100 * total_distance
        self.__has_build = True
    

    
    def __start(self):
        if not self.__one_time_use or self.__number_of_calls < 1:

            if self.__run_type == 'RUN':
                self.__motor.run(speed = self.speed)

            elif self.__run_type == 'RUN_TIME':
                self.__motor.run_time(speed = self.speed, time = self.value, wait = False)

            elif self.__run_type == 'RUN_ANGLE':
                self.__motor.run_angle(speed = self.speed, rotation_angle = self.value, wait = False)

            elif self.__run_type == 'RUN_TARGET':
                self.__motor.run_target(speed = self.speed, target_angle = self.value, wait = False)

            elif self.__run_type == 'RUN_UNTIL_STALLED':
                self.__motor.run_until_stalled(speed = self.speed, then = Stop.BRAKE)

            elif self.__run_type == 'DC':
                self.__motor.dc(duty = self.speed)

        self.numberOfCalls += 1
    
    def __stop(self, stop_type = Stop.BRAKE):
        ErrorEx.isType(stop_type, "stop_type", Stop)
        if self.__run_type == 'RUN' or self.__run_type == 'DC':

            if stop_type == Stop.BRAKE:
                self.__motor.brake()

            elif stop_type == Stop.HOLD:
                self.__motor.hold()

            elif stop_type == Stop.COAST:
                self.__motor.coast()
    


    def update(self, distance):
        if not self.__has_build:
            __throw_initialization_error()
        ErrorEx.isType(distance, "distance", [int, float])

        if abs(abs(distance) - abs(self.__start_distance)) < 1:
            self.__start()
        elif abs(abs(distance) - abs(self.__stop_distance)) < 1:
            self.__stop()
    

    @staticmethod
    def __throw_initialization_error():
        raise Exception("NOT INITIALIZED!! ----- please use '.build()' function")
    
    
