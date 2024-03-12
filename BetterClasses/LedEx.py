from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color

default_NOT_STARTED_color = Color.RED
default_TAKE_YOUR_HANDS_OFF_color = None
default_IN_PROGRESS_color = Color.GREEN
default_ENTERED_CENTER_color = Color.ORANGE

class LedEx():
    def __init__(self, brick):
        self.brick = brick
        self.not_started_color = default_NOT_STARTED_color
        self.take_your_hands_off_color = default_TAKE_YOUR_HANDS_OFF_color
        self.in_progress_color = default_IN_PROGRESS_color
        self.entered_center_color = default_ENTERED_CENTER_color

        self.hasBuild = False
    
    def addNotStartedColor(self, color):
        if isinstance(color, Color) or color == None:
            self.not_started_color = color
        else: __throw_color_error()

        return self
    
    def addTakeYourHandsOffColor(self, color):
        if isinstance(color, Color) or color == None:
            self.take_your_hands_off_color = color
        else: __throw_color_error()

        return self
    
    def addInProgressColor(self, color):
        if isinstance(color, Color) or color == None:
            self.in_progress_color = color
        else: __throw_color_error()

        return self
    
    def addEnteredCenter(self, color):
        if isinstance(color, Color) or color == None:
            self.in_entered_center = color
        else: __throw_color_error()

        return self
    
    def build(self):
        self.hasBuild = True
    
    def not_started(self):
        if self.hasBuild:
            if self.not_started_color == None:
                self.off()
            else: self.brick.light.on(self.not_started_color)
        else: __throw_initialization_error()
        
    
    def take_your_hands_off(self):
        if self.hasBuild:
            if self.take_your_hands_off_color == None:
                self.off()
            else: self.brick.light.on(self.take_your_hands_off_color)
        else: __throw_initialization_error()

    def in_progress(self):
        if self.hasBuild:
            if self.in_progress_color == None:
                self.off()
            else: self.brick.light.on(self.in_progress_color)
        else: __throw_initialization_error()
    
    def entered_center(self):
        if self.hasBuild:
            if self.entered_center == None:
                self.off()
            else: self.brick.light.on(self.entered_center_color)
        else: __throw_initialization_error()
    
    def off(self):
        if self.hasBuild:
            self.brick.light.off()
        else: __throw_initialization_error()
    



    @staticmethod
    def __throw_initialization_error():
        raise Exception("NOT INITIALIZED!! ----- please use '.build()' function")
    
    @staticmethod 
    def __throw_color_error():
        raise Exception("NOT A COLOR")