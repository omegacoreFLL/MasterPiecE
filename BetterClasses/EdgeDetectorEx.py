from BetterClasses.ErrorEx import *

class EdgeDetectorEx():
    def __init__(self):
        self.__current = False
        self.__past = False

        self.__rising = False
        self.__falling = False
        self.__high = False
        self.__low = False
    
    def set(self, current):
        ErrorEx.isType(current, "current", bool)
        self.__current = current

    def update(self):
        self.__rising = not self.__past and self.__current
        self.__falling = self.__past and not self.__current
        self.__high = self.__past and self.__current
        self.__low = not self.__past and not self.__current

        self.__past = self.__current
    
    def rising(self):
        return self.__rising
    
    def falling(self):
        return self.__falling
    
    def high(self):
        return self.__high

    def low(self):
        return self.__low
    
    
