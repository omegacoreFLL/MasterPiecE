from pybricks.parameters import Button, Color
from TankDrive.constants import *
import math


#change generals here
start_run_button = Button.CENTER
run_colors = [Color.GREEN, Color.WHITE, Color.BROWN, Color.RED, Color.YELLOW, Color.BLACK, Color.ORANGE, Color.BLUE]

class Run():
    def __init__(self, button, function, run_number = None, color = None, oneTimeUse = True):
        self.button = button
        self.run_number = run_number
        self.runs = 0
        self.oneTimeUse = oneTimeUse

        self.running = False
        self.pastRunning = False

        self.function = function
        self.color = color
        
    
    def hasJustStarted(self):
        return not self.pastRunning and self.running
    
    def hasNotStarted(self):
        return not self.pastRunning and not self.running
    
    def runable(self):
        return not self.oneTimeUse or not self.runs > 0
    
    def update(self):
        self.pastRunning = self.running
    
    def start(self):
        if self.hasJustStarted():
            self.runs +=1
            self.function()
        self.running = False
    



class RunController():
    def __init__(self, gamepad, brick, telemetry, run_list = None):

        self.next_run = 1
        self.run_loops = 0
        self.all_oneTimeUse = True

        self.run_list = None
        if not self.run_list == None:
            self.addRunList(self.run_list)

        self.color_sensor_control = False
        self.color_sensor = None
        self.entered_center = False
        self.run = 1

        self.gamepad = gamepad
        self.brick = brick
        self.telemetry = telemetry

        self.beforeEveryRun = None
        self.afterEveryRun = None
    


    def addRunList(self, run_list):
        self.run_list = run_list
        self.total_runs = len(self.run_list)

        for run in range(self.total_runs):
            self.run_list[run].run_number = run + 1
            if not self.run_list[run].oneTimeUse:
                self.all_oneTimeUse = False
        
        self.__showcaseNextRun()
    
    def addBeforeEveryRun(self, function):
        self.beforeEveryRun = function
    
    def addAfterEveryRun(self, function):
        self.afterEveryRun = function

    def addColorSensor(self, sensor):
        self.color_sensor = sensor
        self.color_sensor_control = True
    


    def __shouldStartRun(self, run):
        run.update()
        temporary_start = False

        if self.__needToEnterCenter(run.run_number):
            if self.entered_center:
                if self.gamepad.wasJustPressed(run.button):
                    temporary_start = True
        else:
            if not self.entered_center:
                if self.gamepad.wasJustPressed(run.button):
                    temporary_start = True
        
        start = False

        if temporary_start:
            if run.oneTimeUse:
                if run.runs == 0:
                    start = True
            else: start = True
        
        if start: 
            run.running = True

    def __needToEnterCenter(self, run_number):
        return run_number > 4

    def __updateManual(self):
        self.gamepad.updateButtons()

        if self.gamepad.wasJustPressed(Button.CENTER):
            self.entered_center = not self.entered_center
        
        for run in range(self.total_runs):
            if not do_runs_in_order or run + 1 == self.next_run:
                self.__shouldStartRun(self.run_list[run])
                if self.run_list[run].hasJustStarted():
                    self.__start(self.run_list[run])
        
    def __updateNextRun(self, current_run):
        if current_run.run_number == self.next_run:
            this_loop = self.run_loops

            while not self.run_list[self.next_run - 1].runable() and self.run_loops == this_loop:
                if self.next_run >= self.total_runs:
                    self.next_run = 1
                    self.run_loops += 1
                else: self.next_run += 1
            
            if not self.run_loops == this_loop and self.next_run == current_run.run_number:
                if self.next_run >= self.total_runs:
                    self.next_run = 1
                    self.run_loops += 1
                else: self.next_run += 1

    #upcoming feature.
    def __updateAuto(self):
        return 0
    
    def update(self):
        if not self.color_sensor_control:
            self.__updateManual()
        else: self.__updateAuto()



    def __start(self, run):
        if not self.beforeEveryRun == None:
            self.beforeEveryRun()
        self.__showcaseInProgress(run.run_number)
        
        run.start()

        if not self.afterEveryRun == None:
            self.afterEveryRun()
        self.entered_center = False

        self.__updateNextRun(current_run = run)
        self.__showcaseNextRun()



    def __showcaseInProgress(self, run_number):
        self.telemetry.clear()
        self.telemetry.addData('                          ')
        self.telemetry.addData('                          ')
        self.telemetry.addData('run', run_number)
        self.telemetry.addData('  in progress...')
    
    def __showcaseNextRun(self):
        self.telemetry.clear()

        if self.done():
            self.telemetry.addData("                  ")
            self.telemetry.addData("    DONE    ")
            self.telemetry.addData("     <3         ")
            return 0
        
        next_run = self.run_list[self.next_run - 1]

        self.telemetry.addData("                         ")
        self.telemetry.addData("next run:", self.next_run)
        self.telemetry.addData("                         ")

        if next_run.run_number > 4:
            self.telemetry.addData("Button.CENTER +    ")
        self.telemetry.addData(next_run.button)

    def done(self):
        return self.all_oneTimeUse and self.run_loops > 0


