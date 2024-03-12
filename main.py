#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

import math

from BetterClasses.MathEx import * 
from BetterClasses.ButtonsEx import *
from BetterClasses.MotorEx import *
from TankDrive.constants import *
from TankDrive.pathing import *
from Controllers.RunController import *
from robot import *
from RunManager import *

import threading

core.led_control.not_started()
while True:
    loop()