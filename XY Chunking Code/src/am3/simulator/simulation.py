"""
Contains classes directly related to encoding and decoding JSON Simulations.
"""

from typing import List, Dict, Union, Tuple
from am3.robot import Robot
from am3.simulator.material import Material
from mathutils import Vector


def describe_point(point: Union[Vector, Tuple[float, float, float]]) -> List[float]:
    """
    Returns a pure Python representation of ``point``, with the coordinates rounded to 6 
    decimal places. ``point`` can either be a ``Vector`` or a ``Tuple[float, float, float]``.
    The returned value is a ``List`` of floats.

    :param point: a 3D point
    :type point: Vector or Tuple[float, float, float]
    :return: A JSON-ready representation of the point as a list
    :rtype: List[float]
    """

    return [round(point[0], 6), round(point[1], 6), round(point[2], 6)]


class SimulationRobot:
    """
    Represents a single Robot as it should be represented in a simulation, as encoded by a JSON
    Simulation. The only necessary information to store is the robot's number, location, and rotation.
    """

    number = 0
    """
    The number of this Robot

    :type: int
    """

    location = None
    """
    The location of this Robot, either at initialization or at a specific frame

    :type: List[float] or Vector
    """

    rotation = 0
    """
    The rotation of this Robot, either at initialization or at a specific frame

    :type: float
    """

    def __init__(self, robot: Robot = None, data: Dict = None):
        if data is None:
            assert robot is not None, 'You must supply either a Robot or a data object'

            self.number = robot.get_number()
            self.location = describe_point(robot.get_location())
            self.rotation = robot.get_rotation()
        elif data is not None:
            self.number = data['n']
            self.location = Vector(tuple(data['v']))
            self.rotation = data['r']

    def serialize(self) -> Dict:
        """
        Returns a pure Python representation of a Robot. The only important values for
        exporting to JSON are the robot's number, position, and rotation, which are represented
        with ``n``, ``v``, and ``r`` respectively.

        :return: a JSON-ready representation of the Robots
        :rtype: List[Dict[str, object]]
        """

        return {
            'n': self.number,  # n = robot number
            'v': describe_point(self.location), # v = robot position
            'r': round(self.rotation, 6) # r = rotation in radians
        }
        

class SimulationInitialization:
    """
    Represents the data needed to initialize a scene, as encoded by a JSON Simulation. This is
    simply initial positioning for the Robots
    """

    robots = []
    """
    The list of robots (i.e. the robots' positions).

    :type: List[SimulationRobot]
    """

    def __init__(self, robots: List[Robot] = None, data: List[Dict] = None):
        if data is None:
            assert robots is not None, 'You must supply either a List of Robots or a data object'
            self.robots = [SimulationRobot(robot=robot) for robot in robots]
        else:
            self.robots = [SimulationRobot(data=robot) for robot in data['machines']]

        self.robots.sort(key=lambda x: x.number)

    def serialize(self):
        return {
            'machines': [robot.serialize() for robot in self.robots]
        }


class SimulationMaterial:
    """
    Represents a single piece of Material as encoded by a JSON file. Each piece of material is, in
    essence, a list of points only.
    """

    points = []
    """
    The points representing this material. Each point is a ``List[float]`` with 3 items (x, y, z)
    coordinates.

    :type: List[List[float]]
    """

    def __init__(self, material: Material = None, data: Dict = None):
        if data is None:
            assert material != None, 'You must supply a Material object if not supplying raw data'
            self.points = material.points
        else:
            self.points = data

    def serialize(self):
        """
        Returns a pure Python representation of this SimulationMaterial. The only information needed
        from a Material object is the path (i.e., the list of points in the object). The result is
        a list of points, which are themselves each represented as a list of floats.

        :return: a JSON-ready representation of the Material object
        :rtype: List[List[float]]
        """
        return [describe_point(point) for point in self.points]


class SimulationFrame:
    """
    Represents a single frame in a JSON Simulation. Each frame needs to store the robot position of
    every robot, and all the material placed only in that frame.
    """

    robots = None
    """
    An array of the 'robots' that need to be moved in this frame. Each robot simply stores
    its number, location, and rotation

    :type: List[SimulationRobot]
    """

    materials = None
    """
    An array of all the material placed in this frame. Each piece of material is a list of points

    :type: List[List[List[float]]]
    """

    def __init__(self,
                 robots: List[Robot] = None,
                 materials: List[Material] = None,
                 data: Dict = None):
        if data is None:
            assert robots != None and materials != None, 'You must supply a List of Robots and a List of Materials'
        
            self.robots = [SimulationRobot(robot=robot) for robot in robots]
            self.materials = [SimulationMaterial(material=material) for material in materials]
        else:
            self.robots = [SimulationRobot(data=robot_data) for robot_data in data['machines']]
            self.materials = [SimulationMaterial(data=material_data) for material_data in data['printeds']]

        self.robots.sort(key=lambda x: x.number)

    def serialize(self):
        """
        Returns a pure Python representation of a scene state. The only information needed for
        a \"scene state\" are the List of robots and the List of Material objects extruded during
        the most recent frame.

        :return: a JSON-ready representation of this frame
        :rtype: Dict[str, object]
        """
        return {
            'machines': [robot.serialize() for robot in self.robots],
            'printeds': [material.serialize() for material in self.materials]
        }

class Simulation:
    """
    Represents a ``Simulation`` as encoded by a JSON file. A Simulation has two main properties,
    ``init`` and ``frames``.
    """

    init = None
    """
    Stores the initialization data, in this case, just the initial robot positions. 

    :type: SimulationInitialization
    """

    frames = []
    """
    Stores the frame data. In this case, each frame stores the robot positions and the material
    placed in that frame.

    :type: List[SimulationFrame]
    """

    def __init__(self, robots: List[Robot] = None, data: Dict = None):
        assert robots != None or data != None, 'You must supply either a Robot or data object'

        if data is None:
            assert robots != None, 'You must supply either a List of Robots or a data object'
            self.init = SimulationInitialization(robots=robots)
            self.frames = []
        else:
            self.init = SimulationInitialization(data=data['init'])
            self.frames = [SimulationFrame(data=frame_data) for frame_data in data['frames']]

    def add_frame(self, robots: List[Robot], materials: List[Material]):
        """
        Creates a new frame given the robots in their current state and all the material placed in
        the current frame.

        :param robots: a List of Robots
        :type robots: List[Robot]
        :param material: a List of Materials
        :type materials: List[Material]
        """
        self.frames.append(SimulationFrame(robots=robots, materials=materials))

    def robot_count(self) -> int:
        """
        :return: the number of robots that exist in this Simulation
        :rtype: int
        """
        return len(self.init.robots)

    def frame_count(self) -> int:
        """
        :return: the number of frames in this Simulation
        :rtype: int
        """
        return len(self.frames)

    def serialize(self) -> Dict:
        """
        Returns a pure Python (i.e. JSON-ready) representation of this ``Simulation`` object.

        :return: a JSON-ready Dict
        :rtype: Dict
        """
        return {
            'init': self.init.serialize(),
            'frames': [frame.serialize() for frame in self.frames]
        }

    