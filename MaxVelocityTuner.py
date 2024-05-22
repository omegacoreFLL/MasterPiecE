
#!/usr/bin/env pybricks-micropython


from pybricks.tools import wait, StopWatch
from pybricks.parameters import Stop
from robot import *

#time going max speed (sec)
time = 2


ms = time * 1000
core = Robot()
timer = StopWatch()


while ms - timer.time() >= 0:
    core.setWheelPowers(100, 100)
    core.updateOdometry()


core.setDriveTo(Stop.BRAKE)

max_vel = core.localizer.distance / time
print("MAX VEL: {0} cm/sec".format(round(max_vel, 4)))






