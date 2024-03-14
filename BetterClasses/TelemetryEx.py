from pybricks.hubs import EV3Brick
from BetterClasses.ErrorEx import *

class TelemetryEx():
    def __init__(self, brick):
        ErrorEx.isType(brick, "brick", EV3Brick)
        self.__brick = brick
    
    def addData(self, *message):
        self.__brick.screen.print(message)
    
    def clear(self):
        self.__brick.screen.clear()
