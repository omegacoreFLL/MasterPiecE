
from pybricks.tools import StopWatch
from BetterClasses.ErrorEx import *

#generic PID controller
class PIDController():
    def __init__(self, kP = 0, kI = 0, kD = 0):
        isType([kP, kI, kD], ["kP", "kI", "kD"], [[int, float], [int, float], [int, float]])

        self.__kP = kP
        self.__kD = kD
        self.__kI = kI

        self.__proportional = 0
        self.__derivative = 0
        self.__integral = 0

        self.__current_error = 0
        self.__current_time = 0

        self.__past_error = 0
        self.__past_time = 0
    
        self.__derivative_timer = StopWatch()
    
    def setCoefficients(self, kP = None, kI = None, kD = None):
        if not kP == None:
            isType([kP], ["kP"], [[int, float]])
            self.__kP = kP
        if not kI == None:
            isType([kI], ["kI"], [[int, float]])
            self.__kI = kI
        if not kD == None:
            isType([kD], ["kD"], [[int, float]])
            self.__kD = kD

    def calculate(self, error):
        isType([error], ["error"], [[int, float]])

        self.__error = error
        self.__current_time = self.__derivative_timer.time()

        self.__proportional = error
        self.__derivative = (error - self.__past_error) / (self.__current_time - self.__past_time)
        self.__integral += error

        power = self.__proportional * self.__kP + self.__derivative * self.__kD + self.__integral * self.__kI 
  
        self.__past_time = self.__current_time
        self.__past_error = error

        return power

