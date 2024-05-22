from BetterClasses.EdgeDetectorEx import *
from pybricks.parameters import Button

# edge detectors for each button on the brick

class ButtonEx:
    def __init__(self, brick):
        self.__brick = brick

        # dictionary for all EV3 buttons (excepting the exit button, we don't talk about that)
        self.__buttons = {
            Button.LEFT: EdgeDetectorEx(), 
            Button.RIGHT: EdgeDetectorEx(), 
            Button.UP: EdgeDetectorEx(), 
            Button.DOWN: EdgeDetectorEx(), 
            Button.CENTER: EdgeDetectorEx()
        }

        self.__button_list_len = 5
        self.__pressed_buttons = []

    def updateButtons(self):
        # get the current gamepad state
        self.__pressed_buttons = self.__brick.buttons.pressed()
        
        if not self.__pressed_buttons:
            # no button is pressed
            for detector in self.__buttons.values():
                detector.set(False)
                detector.update()
            
            return None
            
        # take only the first pressed button
        pressed_button = next(iter(self.__pressed_buttons))

        for button, detector in self.__buttons.items():
            if pressed_button == button:
                detector.set(True)
            else: detector.set(False)    
            detector.update()      



    def wasJustPressed(self, button):
        try: 
            return self.__buttons[button].value.rising
        except: print("\n\nthat's not a button??")

    def wasJustReleased(self, button):
        try:
            return self.__buttons[button].value.falling
        except: print("\n\nthat's not a button??")



    def isPressed(self, button):
        try:
            return self.__buttons[button].value.high
        except: print("\n\nthat's not a button??")

    def isReleased(self, button):
        try:
            return self.__buttons[button].value.low
        except: print("\n\nthat's not a button??")
