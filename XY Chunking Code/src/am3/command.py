"""
Contains all the Command-like objects that are used to simulate. These correspond to G-code
commands, except that they do not specify an extrusion or a feedrate.
"""

from typing import Tuple
import math
import am3
from mathutils import Vector


class MoveCommand:
    """
    The MoveCommand encodes the position to which a robot should move and the color it should
    print with. The MoveCommand can also calculate how long it should take to execute given the
    robot's current location.
    """

    def __init__(self,
                 location: Vector,
                 robot_speed: float = 10,
                 color: Tuple[int, int, int] = (0, 0, 0)):
        """
        Constructs a MoveCommand object.

        :param location: the location to which this command refers. If a robot executes this
            command, ``location`` is where it will move to.
        :type location: Vector
        :param robot_speed: how quickly the robot moves, in mm/s
        :type robot_speed: float
        :param color: the color to print when executing this command, if any
        :type color: Tuple[int, int, int]
        """

        self.type = type
        self.location = location * am3.settings.MODEL_SCALE
        self.robot_speed = robot_speed
        self.color = color

    def calculate_time(self, current_location: Vector) -> float:
        """
        Calculates how long this command will take to execute starting from ``current_location``,
        in seconds. This is a known quantity given that ``self.robot_speed`` is a meaningful
        value.

        :param current_location: the current location of a Robot, used to determine how far from
            the target coordinate the Robot is.
        :type current_location: Vector
        :return: the time it will take to execute this command, in seconds
        :rtype: float
        """

        x_1 = self.location[0]
        y_1 = self.location[1]
        z_1 = self.location[2]

        x_2 = current_location[0]
        y_2 = current_location[1]
        z_2 = current_location[2]

        # Assuming Gcode coordinate correspond to mm (i.e. (1, 0, 0) is (1mm, 0mm, 0mm))
        distance = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2 + (z_1 - z_2)**2)

        time_in_seconds = distance / self.robot_speed
        return time_in_seconds


class ToolOnCommand:
    """
    An empty object used to represent a \"tool on\" command for a Robot. This will toggle the
    robot's extruder \"on\", letting it know to extrude material.
    """

    def __init__(self, params=None):
        """Do nothing"""
        pass


class ToolOffCommand:
    """
    An empty object used to represent a \"tool off\" command for a Robot. This will toggle the
    robot's extruder \"off\", letting it know to stop extruding material.
    """

    def __init__(self, params=None):
        """Do nothing"""
        pass


class NewLayerCommand:
    """
    An empty object used to represent the transition from one layer to another.
    """

    def __init__(self):
        """Do nothing"""
        pass
