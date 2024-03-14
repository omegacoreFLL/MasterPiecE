from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color
from BetterClasses.EdgeDetectorEx import *
from BetterClasses.ErrorEx import *

class ButtonEx:
    def __init__(self, brick):
        ErrorEx.isType(brick, "brick", EV3Brick)
        self.__brick = brick

        self.__button_list = [Button.LEFT, Button.RIGHT, Button.UP, Button.DOWN, Button.CENTER]
        self.__pressedButtons = []
        self.__detector_list = []

        for i in range(len(self.__button_list)):
            self.__detector_list.append(EdgeDetectorEx())

    def __updateGamepad(self):
        self.__pressedButtons = self.__brick.buttons.pressed()
        

        if len(self.__pressedButtons) > 0:
            for b in self.__pressedButtons:
                for i in range(len(self.__button_list)):
                    if b == self.__button_list[i]:
                        self.__detector_list[i].set(True)
                    else: self.__detector_list[i].set(False)
        else: 
            for i in range(len(self.__button_list)):
                self.__detector_list[i].set(False)

    def __updateDetectors(self):
        for detector in self.__detector_list:
            detector.update()

    def updateButtons(self):
        self.__updateGamepad()
        self.__updateDetectors()



    def wasJustPressed(self, b):
        ErrorEx.isType(b, "b", Button)

        for i in range(len(self.__button_list)):
            if b == self.__button_list[i]:
                return self.__detector_list[i].rising()

    def wasJustReleased(self, b):
        ErrorEx.isType(b, "b", Button)

        for i in range(len(self.__button_list)):
            if b == self.__button_list[i]:
                return self.__detector_list[i].falling()



    def isPressed(self, b):
        ErrorEx.isType(b, "b", Button)

        for i in range(len(self.__button_list)):
            if b == self.__button_list[i]:
                return self.__detector_list[i].high()

    def isReleased(self, b):
        ErrorEx.isType(b, "b", Button)

        for i in range(len(self.__button_list)):
            if b == self.__button_list[i]:
                return self.__detector_list[i].low()
