from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color

default_NOT_STARTED_color = Color.RED
default_TAKE_YOUR_HANDS_OFF_color = Color.ORANGE
default_IN_PROGRESS_color = Color.GREEN

class LedEx():
    def __init__(self, brick):
        self.brick = brick
        self.not_started_color = default_NOT_STARTED_color
        self.take_your_hands_off_color = default_TAKE_YOUR_HANDS_OFF_color
        self.in_progress_color = default_IN_PROGRESS_color

        self.hasBuild = False
    
    def addNotStartedColor(self, color):
        if isinstance(color, Color):
            self.not_started_color = color
        else: raise Exception("NOT A COLOR")

        return self
    
    def addTakeYourHandsOffColor(self, color):
        if isinstance(color, Color):
            self.take_your_hands_off_color = color
        else: raise Exception("NOT A COLOR")

        return self
    
    def addInProgressColor(self, color):
        if isinstance(color, Color):
            self.in_progress_color = color
        else: raise Exception("NOT A COLOR")

        return self
    
    def build(self):
        self.hasBuild = True
    
    def not_started(self):
        if self.hasBuild:
            self.brick.light.on(self.not_started_color)
    
    def take_your_hands_off(self):
        if self.hasBuild:
            self.brick.light.on(self.take_your_hands_off_color)

    def in_progress(self):
        if self.hasBuild:
            self.brick.light.on(self.in_progress_color)
    
    def off():
        if self.hasBuild:
            self.brick.light.off()