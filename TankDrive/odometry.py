from pybricks.tools import StopWatch
from BetterClasses.MathEx import *

import math

#odometry constants

WHEEL_RADIUS = 4.3 / 2
TRACK_WIDTH = 9.97

GEAR_RATIO = 1 # in / out
TICKS_PER_REVOLUTION = 360

dW = TRACK_WIDTH / 2

def encoderTicksToCM(ticks):
    return ticks * GEAR_RATIO * 2 * math.pi * WHEEL_RADIUS / TICKS_PER_REVOLUTION

class TwoWheelLocalizer:

    def __init__(self, left, right, gyroscope, upside_down_gyro = False):
        self.pose = Pose(0, 0, 0)
        self.pastPose = Pose(0, 0, 0)

        self.leftEncoder = left
        self.rightEncoder = right
        self.gyro = gyroscope
        self.timer = StopWatch()

        self.poseL, self.poseR, self.angle,     self.pastPoseL, self.pastPoseR, self.pastAngle,     self.deltaL, self.deltaR, self.deltaAngle = [
            0, 0, 0,     0, 0, 0,     0, 0, 0 ]

        self.time, self.vel, self.pastTime,     self.pastVel, self.deltaTime, self.deltaVel,    self.distance, self.pastDistance, self.deltaDistance = [
            0, 0, 0,     0, 0, 0,    0, 0, 0 ]
        
        self.gyro_offset = 0
        if upside_down_gyro:
            self.gyro_direction = -1
        else: self.gyro_direction = 1



    def setPoseEstimate(self, newPose):
        self.gyro_offset = newPose.head - self.pose.head
        self.pose = self.pastPose = Pose(newPose.x, newPose.y, normalizeDegrees(newPose.head))

        self.pastPoseL, self.pastPoseR, self.pastAngle, self.pastTime, self.pastDistance, self.pastVel = [
            self.poseL, self.poseR, self.angle, self.time, self.deltaDistance, self.vel ]



    def zeroTimer(self):
        self.timer.reset()
    
    def zeroEncoders(self):
        self.leftEncoder.reset_angle(0)
        self.rightEncoder.reset_angle(0)
    
    def zeroGyro(self):
        self.gyro.reset_angle(0)
    
    def zeroDistance(self):
        self.distance = 0
    
    def zeroPose(self):
        self.gyro_offset = -self.pose.head
        self.pose = self.pastPose = Pose(0, 0, 0)

        self.pastPoseL = self.pastPoseR = self.pastAngle = self.pastTime = self.pastDistance = self.pastVel = 0
        self.poseL = self.poseR = self.angle = self.time = self.deltaDistance = self.vel = 0

    def zero(self):
        self.zeroEncoders()
        self.zeroGyro()
        self.zeroDistance()
        self.zeroPose()
        self.zeroTimer()


    
    def updateDeltas(self):
        self.poseL = encoderTicksToCM(self.leftEncoder.angle())
        self.poseR = encoderTicksToCM(self.rightEncoder.angle())
        self.angle = normalizeDegrees(self.gyro.angle() * self.gyro_direction + self.gyro_offset)
        self.time = self.timer.time()

        self.deltaL = self.poseL - self.pastPoseL
        self.deltaR = self.poseR - self.pastPoseR
        self.deltaAngle = self.angle - self.pastAngle
        self.deltaTime = self.time - self.pastTime

        self.deltaDistance = (self.deltaL + self.deltaR) / 2
        self.vel = (self.deltaL + self.deltaR) / self.deltaTime

        self.pastPoseL, self.pastPoseR, self.pastAngle, self.pastTime, self.pastDistance, self.pastVel = [
            self.poseL, self.poseR, self.angle, self.time, self.deltaDistance, self.vel ]
    
    def updatePose(self):
        radians = normalizeRadians(toRadians(self.pastAngle + self.deltaAngle / 2))

        newX = self.pose.x + self.deltaDistance * math.cos(radians)
        newY = self.pose.y + self.deltaDistance * math.sin(radians)
        newHead = normalizeDegrees(self.angle)

        self.distance += self.deltaDistance
        self.pose.set(newX, newY, newHead)
    
    def update(self):
        self.updateDeltas()
        self.updatePose()
    


    def getDistance(self):
        return self.distance
    
    def getPoseEstimate(self):
        return self.pose
    
    def getRawEncoderTicks(self):
        return (self.leftEncoder.angle(), self.rightEncoder.angle())
    
    def getVelocity(self):
        return self.vel






