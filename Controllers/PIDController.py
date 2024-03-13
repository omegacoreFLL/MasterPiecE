
from pybricks.tools import wait, StopWatch
import math

class PIDController():
    def __init__(self, kP = 0, kD = 0, kI = 0):
        self.kP = kP
        self.kD = kD
        self.kI = kI

        self.proportional = 0
        self.derivative = 0
        self.integral = 0
        self.error = 0
        self.currentTime = 0
        self.pastError = 0
        self.pastTime = 0
    
        self.derivativeTimer = StopWatch()
    
    def setCoefficients(self, kP = None, kD = None, kI = None):
        if not kP == None:
            self.kP = kP
        if not kD == None:
            self.kD = kD
        if not kI == None:
            self.kI = kI

    def calculate(self, error):
        self.error = error
        self.currentTime = self.derivativeTimer.time()

        self.proportional = error
        self.derivative = (error - self.pastError) / (self.currentTime - self.pastTime)
        self.integral += error

        power = self.proportional * self.kP + self.derivative * self.kD + self.integral * self.kI 
  
        self.pastTime = self.currentTime
        self.pastError = error

        return power

