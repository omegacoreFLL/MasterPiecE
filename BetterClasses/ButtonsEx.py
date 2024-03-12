#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


class ButtonEx:
    def __init__(self, brick):
        self.brick = brick

        self.pressedButtons = []

        self.left = self.right = self.up = self.down = self.center = False
        self.pastLeft = self.pastRight = self.pastUp = self.pastDown = self.pastCenter = False

        self.riseLeft = self.riseRight = self.riseUp = self.riseDown = self.riseCenter = False
        self.fallLeft = self.fallRight = self.fallUp = self.fallDown = self.fallCenter = False
        self.isLeft = self.isRight = self.isUp = self.isDown = self.isCenter = False

    def __updateInitial(self):

        self.left = self.right = self.up = self.down = self.center = False
        self.pressedButtons = self.brick.buttons.pressed()

        if len(self.pressedButtons) > 0:
            for b in self.pressedButtons:
                if b == Button.LEFT:
                    self.left = True
                elif b == Button.RIGHT:
                    self.right = True
                elif b == Button.UP:
                    self.up = True
                elif b == Button.DOWN:
                    self.down = True
                elif b == Button.CENTER:
                    self.center = True

    def __updateRisingEdge(self):
        self.riseLeft = self.riseRight = self.riseUp = self.riseDown = self.riseCenter = False

        if self.left and not self.pastLeft:
            self.riseLeft = True
        
        if self.right and not self.pastRight:
            self.riseRight = True
        
        if self.up and not self.pastUp:
            self.riseUp = True
        
        if self.down and not self.pastDown:
            self.riseDown = True
        
        if self.center and not self.pastCenter:
            self.riseCenter = True

    def __updateFallingEdge(self):
        self.fallLeft = self.fallRight = self.fallUp = self.fallDown = self.fallCenter = False

        if not self.left and self.pastLeft:
            self.fallLeft = True
        
        if not self.right and self.pastRight:
            self.fallRight = True
        
        if not self.up and self.pastUp:
            self.fallUp = True
        
        if not self.down and self.pastDown:
            self.fallDown = True
        
        if not self.center and self.pastCenter:
            self.fallCenter = True

    def __updateIsPressed(self):
        self.isLeft = self.isRight = self.isUp = self.isDown = self.isCenter = False

        if self.left and self.pastLeft:
            self.isLeft = True
        
        if self.right and self.pastRight:
            self.isRight = True
        
        if self.up and self.pastUp:
            self.isUp = True
        
        if self.down and self.pastDown:
            self.isDown = True
        
        if self.center and self.pastCenter:
            self.isCenter = True

    def __updateFinal(self):
        self.pastLeft, self.pastRight, self.pastUp, self.pastDown, self.pastCenter = (
            self.left, self.right, self.up, self.down, self.center
        )



    def updateButtons(self):
        self.__updateInitial()
        self.__updateRisingEdge()
        self.__updateIsPressed()
        self.__updateFallingEdge()
        self.__updateFinal()



    def wasJustPressed(self, b):
        if b == Button.LEFT:
            return self.riseLeft
        
        if b == Button.RIGHT:
            return self.riseRight
        
        if b == Button.UP:
            return self.riseUp
        
        if b == Button.DOWN:
            return self.riseDown

        return self.riseCenter

    def wasJustReleased(self, b):
        if b == Button.LEFT:
            return self.fallLeft
        
        if b == Button.RIGHT:
            return self.fallRight
        
        if b == Button.UP:
            return self.fallUp
        
        if b == Button.DOWN:
            return self.fallDown

        return self.fallCenter

    def isPressed(self, b):
        if b == Button.LEFT:
            return self.isLeft
        
        if b == Button.RIGHT:
            return self.isRight
        
        if b == Button.UP:
            return self.isUp
        
        if b == Button.DOWN:
            return self.isDown

        return self.isCenter

    def isReleased(self, b):
        if b == Button.LEFT:
            return not self.isLeft
        
        if b == Button.RIGHT:
            return not self.isRight
        
        if b == Button.UP:
            return not self.isUp
        
        if b == Button.DOWN:
            return not self.isDown

        return not self.isCenter
