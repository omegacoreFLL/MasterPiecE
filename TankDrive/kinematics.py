from TankDrive.constants import * 
from  MasterPiecE.BetterClasses.MathEx import *
import math



def forwardKinematics(wheel_velocities: list):
    (left, right) = wheel_velocities
    return Pose(
        (left + right) / 2,
        0,
        (-left + right) / 2 * dW
    )

def inverseKinematics(velocity: Pose):
    return (
        velocity.x - dW * velocity.head,
        velocity.x + dW * velocity.head
    )

