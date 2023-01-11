"""
Contains the classes used to represent a Robot, and corresponding logic.
"""

from typing import Tuple, List
import math
import bpy

from mathutils import Vector

import am3
from am3.simulator.material import Material
from am3.util import SimulatorMath
from am3.command import MoveCommand, ToolOnCommand, ToolOffCommand, NewLayerCommand
from am3.model import Chunk


class ChunkFrame:
    """
    Represents the video frames needed to render a chunk.
    ``self.location`` specifies where a robot printing this chunk should move to for this frame,
    while ``self.materials`` defines a List of all the extruded material that needs to be placed
    in this frame.
    """

    location = None
    """
    The location to move the respective Robot to

    :type: Vector
    """

    materials = []
    """
    A List of all Material objects to be placed in this frame.

    :type: List[Material]
    """

    def __init__(self, location: Vector, materials: List[Material] = []):
        self.location = location
        self.materials = materials

    def has_material(self) -> bool:
        """
        Determines whether or not this ChunkFrame has any material to render

        :return: whether there is any material in this frame
        :rtype: bool
        """
        return self.materials is not None and len(self.materials) > 0


class RobotParameters:
    """
    Encapsulate numerous parameters that define a physical robot.
    """

    def __init__(self,
                 build_depth: float = 0,
                 printhead_slope: float = 0,
                 width: float = 0,
                 printhead_depth: float = 0,
                 number: int = 0,
                 speed: float = 10):
        """
        Constructs a RobotParameters object

        :param build_depth: the robot's maximum print distance, in mm (default ``0``)
        :type build_depth: float
        :param printhead_slope: the slope of the printhead nozzle, in degrees (default ``0``)
        :type printhead_slope: float
        :param width: the width of this robot, in mm (default ``0``)
        :type width: float
        :param printhead_depth: the length of the printhead past the nozzle, in mm (default ``0``)
        :type printhead_depth: float
        :param number: the number of this robot. Should be unique from other robots
            (default ``0``)
        :type number: int
        :param speed: the speed this robot can move, in mm/s (default ``10``)
        :type speed: float
        """

        self.build_depth = build_depth
        self.printhead_slope = abs(printhead_slope)
        self.width = width
        self.printhead_depth = printhead_depth
        self.number = number
        self.speed = speed

    def copy(self) -> 'RobotParameters':
        """
        :return: a unique copy of ``self``.
        :rtype: RobotParameters
        """
        new_robot_parameters = RobotParameters()
        new_robot_parameters.build_depth = self.build_depth
        new_robot_parameters.printhead_slope = self.printhead_slope
        new_robot_parameters.width = self.width
        new_robot_parameters.number = self.number + 1
        new_robot_parameters.speed = self.speed

        return new_robot_parameters


class Robot:
    """
    Represents a real-world robot. Robots know their parameters, location, Blender
    representations, and chunks. Robots also know how to convert sliced Chunks into
    commands, and how to convert commands into frame data.
    """

    parameters = None
    """
    The parameters that define the physical characteristics of this robot.

    :type: RobotParameters
    """

    body_model = None
    """
    The Blender object used as the \"body\", or \"base\" model of this robot.

    :type: bpy.types.Object
    """

    printhead_model = None
    """
    The Blender object used as the \"printhead\" or \"extruder\" of this robot.

    :type: bpy.types.Object
    """

    last_location = Vector((0, 0, 0))
    """
    The last location this robot received in :func:`~am3.robot.Robot.set_location`.

    :type: Vector
    """

    chunks = []
    """
    The chunks owned by this robot (i.e., the chunks this robot is responsible for printing).

    :type: List[Chunk]
    """

    def __init__(self, parameters: RobotParameters):
        """
        Constructs a new Robot.

        :param parameters: the RobotParameters that define this robot
        :type parameters: RobotParameters
        """

        self.parameters = parameters

        self.body_model = None
        self.printhead_model = None
        self.last_location = None
        self.chunks = []


    def copy(self, flip_direction: bool = False) -> 'Robot':
        """
        Returns a unique copy of this Robot. Copies of this robot also receive new, dedicated
        Blender objects.

        :param flip_direction: Whether or not to rotate the robot 180° after copying
            (default ``False``)
        :type flip_direction: bool
        :return: a unique copy of ``self``
        :rtype: Robot
        """

        new_robot_parameters = self.parameters.copy()
        new_robot = Robot(new_robot_parameters)

        # duplicate object
        scene = bpy.context.scene

        new_body_model = self.body_model.copy()
        new_body_model.data = self.body_model.data.copy()
        new_body_model.animation_data_clear()
        scene.objects.link(new_body_model)
        new_body_model.name = 'Robot Body {}'.format(new_robot.get_number())
        new_robot.body_model = new_body_model

        new_printhead_model = self.printhead_model.copy()
        new_printhead_model.data = self.printhead_model.data.copy()
        new_printhead_model.animation_data_clear()
        scene.objects.link(new_printhead_model)
        new_printhead_model.name = 'Robot Printhead {}'.format(new_robot.get_number())
        new_robot.printhead_model = new_printhead_model

        if flip_direction:
            new_robot.flip()

        return new_robot

    def set_location(self, location: Vector):
        """
        Sets the location of this robot to ``location``. This updates the Blender models as well.

        :param location: the location to set this robot to
        :type location: Vector
        """

        self.last_location = location
        self.body_model.location[0] = location[0]
        self.body_model.location[1] = location[1]

        self.printhead_model.location[0] = location[0]
        self.printhead_model.location[1] = location[1]
        self.printhead_model.location[2] = location[2]

    def set_last_location(self):
        """
        Updates the robot's location to its last location. This is useful when the robot is
        moving along an animation path and you want it to reset to the last location it was told to
        go to.
        """

        self.set_location(self.last_location)

    def get_number(self):
        return self.parameters.number

    def get_speed(self):
        return self.parameters.speed

    def set_number(self, number: int):
        """
        Sets the number of this robot to ``number``. Also updates the Blender model names
        appropriately.

        :param number: the new number for this robot. Should be unique from other robots.
        :type number: int
        """

        self.parameters.number = number
        self.body_model.name = 'Robot Body {}'.format(number)
        self.printhead_model.name = 'Robot Printhead {}'.format(number)

    def set_hidden(self, hidden: bool):
        """
        Sets the robot's state to \"hidden\". Hidden robots don't appear in the scene. Useful for
        debug purposes.

        :param hidden: the state to set the robot's hidden state
        :type hidden: bool
        """

        self.body_model.hide = hidden
        self.printhead_model.hide = hidden

    def rotate(self, angle: float):
        """
        Rotates the robot in the Blender scene by `angle`.

        :param angle: the angle, in radians, to rotate the robot.
        :type angle: float
        """

        self.body_model.rotation_euler.z += angle
        self.printhead_model.rotation_euler.z += angle

    def set_rotation(self, angle: float):
        """
        Sets the absolute Euler rotation of the robot in the Blender scene to `angle`.

        :param angle: the angle, in radians, to set the robot's rotation to.
        :type angle: float
        """

        self.body_model.rotation_euler.z = angle
        self.printhead_model.rotation_euler.z = angle

    def get_rotation(self) -> float:
        """
        :return: the current rotation of the robot
        :rtype: float
        """

        return self.body_model.rotation_euler.z

    def flip(self):
        """
        Flips the robot 180°
        """

        self.rotate(math.pi)

    def set_keyframe(self):
        """
        Adds a Blender animation keyframe for the location of the robot body and printhead
        """

        self.body_model.keyframe_insert(data_path="location")
        self.printhead_model.keyframe_insert(data_path="location")

    def get_location(self) -> Vector:
        """
        :return: The robot's current location in the Blender scene.
        :rtype: Vector
        """

        return self.printhead_model.location

    def clear_animation_data(self):
        """
        Removes all animation keyframes. Useful for regenerating a simulation or resetting a scene.
        """

        self.body_model.animation_data_clear()
        self.printhead_model.animation_data_clear()

    def get_path_between_points(self, point1: Vector, point2: Vector) -> List[MoveCommand]:
        """
        Calculates the robot's movements between two points. This movement is characterized by a
        slight upward movement from the robot's current position (``point1``), a horizontal
        movement to ``point2``'s Z coordinate, then a vertical movement to ``point2``.

        :param point1: the start point from which the robot needs to move.
        :type point1: Vector
        :param point2: the end point to which the robot needs to move.
        :type point2: Vector
        :return: a short list of MoveCommands to move from ``point1`` to ``point2``
        :rtype: List[MoveCommand]
        """

        commands = []
        next_location = Vector((0, 0, 0.05)) + point1
        commands.append(MoveCommand(next_location, self.get_speed()))
        next_location = Vector((point2[0], point2[1], point1[2] + 0.05))
        commands.append(MoveCommand(next_location, self.get_speed()))
        next_location = point2
        commands.append(MoveCommand(next_location, self.get_speed()))

        return commands

    def generate_visualization(self):
        """
        Generates the per-chunk commands needed to print the sliced chunks in ``self.chunks``. This
        process merely involves creating MoveCommands for every point in the sliced chunks, whilst
        toggling the tool on and off when moving between Paths. The robot must also make extra
        movements when moving between chunks. These movements are defined by
        :func:`~am3.robot.Robot.get_path_between_points`.
        """
        last_location = self.get_location()
        transitioning = False

        for chunk in self.chunks:
            chunk.commands.append(ToolOffCommand())

            for chunk_slice in chunk.slices:
                for path in chunk_slice.paths:
                    if not path.vertices:
                        continue
                    if transitioning:
                        path_commands = self.get_path_between_points(
                            last_location, path[0])
                        for command in path_commands:
                            chunk.commands.append(command)
                        transitioning = False

                    chunk.commands.append(MoveCommand(path[0], self.get_speed()))
                    last_location = path[0]
                    chunk.commands.append(ToolOnCommand())

                    for location in path.vertices:  # location: Vector
                        chunk.commands.append(MoveCommand(location, self.get_speed()))
                        last_location = location

                    chunk.commands.append(ToolOffCommand())
                chunk.commands.append(NewLayerCommand())
                chunk.commands.append(ToolOffCommand())

            transitioning = True
            chunk.commands.append(ToolOffCommand())
        
        self.generate_frames()

    def generate_frames(self):
        """
        Generates the ChunkFrames for every chunk belonging to this robot.
        \n
        The number of frames needed to simulate a chunk will relate to ``self.get_speed()``
        and the ``am3.settings.FRAMES_PER_SECOND`` variable.
        """

        for chunk in self.chunks:
            chunk.frame_data = [ChunkFrame(self.get_location(), None)]

            tool_on = False
            layer = 1

            accumulated_time = 0
            accumulated_extruder_path = []

            last_location = self.get_location()
            current_command = 0

            printed_objects = []
            time_step = 1 / am3.settings.FRAMES_PER_SECOND

            current_frame = 0

            while current_command < len(chunk.commands):
                command = chunk.commands[current_command]
                command_increment = 1

                # Tool On Command
                if isinstance(command, ToolOnCommand):
                    if not tool_on:
                        tool_on = True
                        accumulated_extruder_path = [last_location]

                # Tool Off Command
                elif isinstance(command, ToolOffCommand):
                    if tool_on:
                        tool_on = False
                        accumulated_extruder_path.append(last_location)
                        printed_objects.append(
                            Material(accumulated_extruder_path, chunk.color)
                        )
                        accumulated_extruder_path = []

                # New Layer Command
                elif isinstance(command, NewLayerCommand):
                    layer += 1

                # Move Command - if command can be completed before the frame finishes, go ahead and
                # add it. Otherwise, only complete part of the command, using linear interpolation
                elif isinstance(command, MoveCommand):
                    time_to_complete = command.calculate_time(last_location)

                    if accumulated_time + time_to_complete < time_step:
                        accumulated_time += time_to_complete
                        if tool_on:
                            accumulated_extruder_path.append(command.location)
                        last_location = command.location
                    else:
                        percent_of_path_to_complete = (
                            time_step - accumulated_time) / time_to_complete

                        next_location = SimulatorMath.linear_interpolation(
                            last_location, command.location, percent_of_path_to_complete)

                        accumulated_time = 0

                        if tool_on:
                            accumulated_extruder_path.append(next_location)
                        last_location = next_location
                        printed_objects.append(
                            Material(accumulated_extruder_path, chunk.color)
                        )
                        chunk.frame_data.append(ChunkFrame(last_location, printed_objects))
                        printed_objects = []
                        accumulated_extruder_path = [last_location]

                        current_frame += 1
                        command_increment = 0

                current_command += command_increment

            if printed_objects:
                chunk.frame_data.append(ChunkFrame(last_location, printed_objects))
