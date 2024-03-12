
class Telemetry():
    def __init__(self, brick):
        self.brick = brick
    
    def addData(self, *message):
        self.brick.screen.print(message)
    
    def clear(self):
        self.brick.screen.clear()
