from pybricks.ev3devices import ColorSensor
from TankDrive.constants import *
from BetterClasses.EdgeDetectorEx import *
from BetterClasses.ErrorEx import *

default_target_reflection = 0

class ColorSensorEx():
    def __init__(self, colorSensor, target_reflection = None, threshold = 1):
        ErrorEx.isType(colorSensor, "colorSensor", ColorSensor)
        ErrorEx.isType(threshold, "threshold", [int, float])
        self.__colorSensor = colorSensor

        self.__target_relfection = None
        if not target_reflection == None:
            self.setTargetReflection(target_reflection)
        else: self.setTargetReflection(default_target_reflection)

        self.detector = EdgeDetectorEx()
        self.__threshold = threshold
        self.__on_color = False
        self.error = 0
        self.reading = 0
    
    def setTargetReflection(self, target_reflection):
        ErrorEx.isType(target_reflection, "target_reflection", [int, float])
        self.__target_reflection = target_reflection
    
    def update(self):
        self.reading = self.__colorSensor.reflection()
        self.error = self.__target_reflection - self.reading

        self.detector.set(self.onColor())
        self.detector.update()
    
    def onColor(self):
        return abs(self.error) <= abs(self.__threshold)
    
