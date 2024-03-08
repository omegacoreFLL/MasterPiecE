import math

deg_2_rad = math.pi / 180
rad_2_deg = 180 / math.pi


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


class Point:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y  
    
    def set(self, x, y):
        self.x = x 
        self.y = y 
    
    def rotateMatrix(self, angle):
        copy = self.x
        self.x = self.x * math.cos(angle) - self.y * math.sin(angle)
        self.y = copy * math.sin(angle) - self.y * math.cos(angle)

        return self


class Pose(Point):
    def __init__(self, x = 0, y = 0, head = 0):
        super().__init__(x, y)
        self.head = head 

    def set(self, x, y, head):
        super().set(x, y)
        self.head = head 





def rotateMatrix(x, y, angle):
    rotated_x = x * math.cos(angle) - y * math.sin(angle)
    rotated_y = x * math.sin(angle) - y * math.cos(angle)

    return Point(rotated_x, rotated_y)

def normalizeDegrees(deg):
    while deg >= 360:
        deg -= 360
    while deg < 0:
        deg +=360

    return deg  

def normalizeRadians(rad):
    while rad >= 2 * math.pi:
        rad -= 2 * math.pi
    while rad < 0:
        rad += 2 * math.pi

    return rad   

def toRadians(deg):
   return deg * deg_2_rad

def toDegrees(rad):
    return rad * rad_2_deg

def hypot(x, y):
    return math.sqrt(x * x + y * y)

#finding the sign of a number ('signum' = sign + number, conventionally again)
def signum(x):
    if x < 0:
        return -1
    return 1

#milliseconds to second
def msToS(ms):
    return ms / 1000

def clipMotor(value):
    if value < -100:
        value = -100
    elif value > 100:
        value = 100
    return value




