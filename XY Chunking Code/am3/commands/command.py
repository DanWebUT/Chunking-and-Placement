import math
import am3

class MoveCommand:
    def __init__(self, v, machine_speed=10, color=(0,0,0)):
        self.type = type
        self.location = v * am3.settings.MODEL_SCALE
        # machine_speed in cm/s
        self.machine_speed = machine_speed
        self.color = color

    def calculateTime(self, previousLocation):
        x1 = self.location[0]
        y1 = self.location[1]
        z1 = self.location[2]

        x2 = previousLocation[0]
        y2 = previousLocation[1]
        z2 = previousLocation[2]

        # Assuming Gcode coordinate correspond to 2cm (i.e. (1, 0, 0) is (2cm, 0cm, 0cm))
        distance = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

        time_in_seconds = distance * 2 / self.machine_speed
        return time_in_seconds

class ToolOnCommand:
    def __init__(self, params=None):
        pass

class ToolOffCommand:
    def __init__(self, params=None):
        pass

class ReturnCommand:
    def __init__(self):
        pass

class WaitCommand:
    def __init__(self, dependencies):
        self.dependencies = dependencies

class NotifyCommand:
    def __init__(self, number):
        self.number = number

class NewLayerCommand:
    def __init__(self):
        pass

class Command:
    def __init__(self, params):
        self.params = params
