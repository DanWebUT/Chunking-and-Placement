"""
Contains the classes necessary for chunking a model for Cooperative 3D printing.
"""
from typing import Tuple, List

import math
import bpy
import bmesh

from mathutils import Vector
from am3.util import SimulatorMath
from am3.model import Chunk
from am3.robot import Robot, RobotParameters


class ChunkerResult:
    """
    Maintains a list of the chunks in two-robot printing. This object is also a precursor to scaled
    chunking, as SPAR3 printing is based on the two-robot chunking model.
    """

    north_chunks = []
    """
    All the chunks in the ``+Y`` direction (i.e., `North` of the origin chunk). Sorted in printing
    order.

    :type: List[bpy.types.Object]
    """

    origin_chunk = None
    """
    The origin, or \"center\" chunk.

    :type: bpy.types.Object
    """

    south_chunks = []
    """
    All the chunks in the ``-Y`` direction (i.e. `South` of the origin chunk). Sorted in printing
    order.

    :type: List[bpy.types.Object]
    """

    def __init__(self):
        self.north_chunks = []
        self.origin_chunk = None
        self.south_chunks = []

    def set_origin_chunk(self, origin: bpy.types.Object):
        """
        Sets the origin chunk to ``origin``, and hides it from the Blender rendering visibility.

        :param origin: a Blender chunk model
        :type origin: bpy.types.Object
        """

        origin.hide_render = True
        self.origin_chunk = origin

    def add_north_chunk(self, north_chunk: bpy.types.Object):
        """
        Adds a chunk to ``self.north_chunks``, and hides ``north_chunk`` from the Blender rendering
        visibility.

        :param north_chunk: a Blender chunk model
        :type north_chunk: bpy.types.Object
        """

        north_chunk.hide_render = True
        self.north_chunks.append(north_chunk)

    def add_south_chunk(self, south_chunk: bpy.types.Object):
        """
        Adds a chunk to ``self.south_chunks``, and hides ``south_chunk`` from the Blender rendering
        visibility.

        :param south_chunk: a Blender chunk model
        :type south_chunk: bpy.types.Object
        """

        south_chunk.hide_render = True
        self.south_chunks.append(south_chunk)


class Chunker:

    # Class Variables:

    buildplate_dimension = 300
    robot_reach_adjacent = 25
    min_top_dim = 50
    wing_slope = 50
    """
    A class containing all chunking logic in static methods. The main method to be used from
    outside of this class is :func:`~am3.chunker.Chunker.start_scaled`.
    """

    @staticmethod
    def deselect_all_objects():
        """
        Quick tool to deselect all Objects in the Blender scene. Chunking is very dependent on
        selecting specific objects in the scene, so this is frequently used to make sure the
        selection process happens immediately after wiping the slate.
        """

        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except:
            pass

        bpy.ops.object.select_all(action='DESELECT')

    # Starts the AM3 Chunking process
    # Returns: String - error message, or None if no errors occured

    @staticmethod
    def buildplate_chunking(machines: List[Robot], model_object: bpy.types.Object, number_of_machines: int = 2):

        """
        :param machines:
        :param model_object:
        :param number_of_machines:
        :return:

        variables: buildplate_dimension, robot_reach, wing_slope, min_top_dim

        """

        buildplate_dimension = Chunker.buildplate_dimension
        robot_reach_adjacent = Chunker.robot_reach_adjacent
        min_top_dim = Chunker.min_top_dim
        wing_slope = Chunker.wing_slope
        machine_parameters = machines[0].parameters

        if number_of_machines != 2:
            return Chunker.start_scaled(machine_parameters, model_object)

        Chunker.deselect_all_objects()

        scene = bpy.context.scene

        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object

        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active


        model_bounds = SimulatorMath.calculate_bounds(model_object)
        angle_of_elevation = math.pi / 2 - machine_parameters.printhead_slope
        tan_angle = math.tan(angle_of_elevation)

        model_length = (model_bounds.x.max - model_bounds.x.min)

        center_chunk_width = 2 * wing_slope + min_top_dim
        noncenter_chunk_width = 1/2*(buildplate_dimension + 2*robot_reach_adjacent - (center_chunk_width + 2*wing_slope))

        north_plane_co = Vector((0,center_chunk_width,0))
        north_plane_no = Vector((0, 1, tan_angle))

        south_plane_co = Vector((0,-center_chunk_width,0))
        south_plane_no = Vector((0, -1, tan_angle))

        (origin_chunk, north_piece) = Chunker.split_model(new_object, (north_plane_co, north_plane_no))
        (origin_chunk, south_piece) = Chunker.split_model(origin_chunk, (south_plane_co, south_plane_no))

        chunker_result = ChunkerResult()
        chunker_result.set_origin_chunk(origin_chunk)

        north_y = noncenter_chunk_width
        remaining = north_piece
        while north_y < model_bounds.y.max:
            north_plane_co = Vector((0, north_y, 0))
            (north_chunk, remaining) = Chunker.split_model(remaining, (north_plane_co, north_plane_no))
            chunker_result.add_north_chunk(north_chunk)
            north_y += buildplate_dimension
        chunker_result.add_north_chunk(remaining)

        south_y = -1 * noncenter_chunk_width
        remaining = south_piece
        while south_y > model_bounds.y.min:
            south_plane_co = Vector((0, south_y, 0))
            (south_chunk, remaining) = Chunker.split_model(remaining, (south_plane_co, south_plane_no))
            chunker_result.add_south_chunk(south_chunk)
            south_y -= buildplate_dimension
        chunker_result.add_south_chunk(remaining)

        model_object.hide = True
        return chunker_result


    @staticmethod
    def start(robots: List[Robot],
              model_object: bpy.types.Object,
              number_of_robots: int = 2):

        """
        Starts the AM3 Cooperative 3D chunking process. The contents of this method are designed to
        perform only the chunking necessary for TWO robots to cooperate. This is used both for
        two-robot printing as well as used as a first step to scaled chunking. If this method is
        called with ``number_of_robots`` set to anything other than ``2``, this method will
        default to :func:`~am3.chunker.Chunker.start_scaled`.
        \n
        Otherwise, this function calculates the two chunking planes needed for two-robot chunking,
        then iterates those planes outward from the center of the model, slicing the model with
        :func:`~am3.chunker.Chunker.split_model` as it goes.

        :param robots: the Robots that will be printing the chunks
        :type robots: List[Robot]
        :param model_object: the model to Chunk for Cooperative 3D printing
        :type model_object: bpy.types.Object
        :param number_of_robots: the number of robots the caller intends to chunk for. This
            parameter allows this function to work for any number of robots by defaulting to
            another method if the value does not equal 2 (default ``2``).
        :type number_of_robots: int
        :return: if ``number_of_robots == 2``, a Chunker result, else ``None``.
        :rtype: ChunkerResult or None
        """

        if number_of_robots != 2:
            return Chunker.start_scaled(robots, model_object)

        Chunker.deselect_all_objects()

        scene = bpy.context.scene

        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object

        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active

        model_bounds = SimulatorMath.calculate_bounds(model_object)

        robot_parameters = robots[0].parameters

        build_depth = robot_parameters.build_depth
        printhead_depth = robot_parameters.printhead_depth

        model_bounds = SimulatorMath.calculate_bounds(model_object)

        angle_of_elevation = math.pi / 2 - robot_parameters.printhead_slope
        tan_angle = math.tan(angle_of_elevation)

        slope_width = (model_bounds.z.max - model_bounds.z.min) * tan_angle

        origin_chunk_width = min(build_depth, 2 * printhead_depth + 2 * slope_width)

        north_plane_co = Vector((0, origin_chunk_width / 2, 0))
        north_plane_no = Vector((0, 1, tan_angle))

        south_plane_co = Vector((0, -1 * origin_chunk_width / 2, 0))
        south_plane_no = Vector((0, -1, tan_angle))

        (origin_chunk, north_piece) = Chunker.split_model(new_object,
                                                          (north_plane_co, north_plane_no))
        (origin_chunk, south_piece) = Chunker.split_model(origin_chunk,
                                                          (south_plane_co, south_plane_no))

        chunker_result = ChunkerResult()
        chunker_result.set_origin_chunk(origin_chunk)

        north_y = origin_chunk_width / 2 + build_depth - slope_width
        remaining = north_piece
        while north_y < model_bounds.y.max:
            north_plane_co = Vector((0, north_y, 0))
            (north_chunk, remaining) = Chunker.split_model(remaining,
                                                           (north_plane_co, north_plane_no))
            chunker_result.add_north_chunk(north_chunk)
            north_y += build_depth - slope_width
        chunker_result.add_north_chunk(remaining)

        south_y = -1 * origin_chunk_width / 2 - build_depth + slope_width
        remaining = south_piece
        while south_y > model_bounds.y.min:
            south_plane_co = Vector((0, south_y, 0))
            (south_chunk, remaining) = Chunker.split_model(remaining,
                                                           (south_plane_co, south_plane_no))
            chunker_result.add_south_chunk(south_chunk)
            south_y -= build_depth - slope_width
        chunker_result.add_south_chunk(remaining)

        model_object.hide = True
        return chunker_result

    @staticmethod
    def start_single(robot: Robot, model_object: bpy.types.Object):
        """
        Starts the chunking process for a single robot. A single chunking plane is created, and
        iterated from one side of the model to the other.

        :param robot: the Robot that will be printing this model
        :type robot: Robot
        :param model_object: the model that ``robot`` will be printing
        :type model_object: bpy.types.Object
        """

        robot.chunks = []

        robot_parameters = robot.parameters
        build_depth = robot_parameters.build_depth
        model_bounds = SimulatorMath.calculate_bounds(model_object)

        # Start at the northernmost Y coordinate (minus the robot's build depth, that is)
        y_start = model_bounds.y.max - build_depth

        angle_of_elevation = math.pi / 2 - robot_parameters.printhead_slope
        tan_angle = math.tan(angle_of_elevation)

        slope_width = (model_bounds.z.max - model_bounds.z.min) * tan_angle

        plane_co = Vector((0, y_start, 0))
        plane_no = Vector((0, -1, tan_angle))

        counter = 0
        previous_piece = model_object

        while plane_co[1] > model_bounds.y.min - slope_width:
            (chunk, remaining) = Chunker.split_model(previous_piece, (plane_co, plane_no))
            new_chunk = Chunk(chunk, number=counter)
            if counter > 0:
                new_chunk.add_dependency(counter - 1)
            robot.chunks.append(new_chunk)

            counter = counter + 1
            plane_co = plane_co - Vector((0, build_depth - slope_width, 0))
            previous_piece = remaining

        new_chunk = Chunk(remaining, number=counter)
        robot.chunks.append(new_chunk)

    '''
    @staticmethod
    def start_scaled(robots: List[Robot], model_object: bpy.types.Object):
        """
        Performs SPAR3 chunking on ``model_object``, using ``robots``.
        \n
        In the cases where ``len(robots) < 4``, either single-robot chunking or two-robot
        chunking is used. Otherwise, full SPAR3 chunking is used.
        \n
        Assuming a proper two-robot chunk, the SPAR3 chunking process simply subdivides every chunk
        in the two-robot chunk lengthwise, and in the exact same way for every chunk. This method
        also takes care of assigning chunk dependencies and assigning chunks to the proper robot.

        :param robots: the robots that will be used to print this model
        :type robots: List[Robot]
        :param model_object: the model to be printed
        :type model_object: bpy.types.Object
        """

        if len(robots) == 1:
            Chunker.start_single(robots[0], model_object)
            return

        # red, white, and blue, Arkansas flag shades
        colors = [(255, 10, 27), (0, 30, 83), (255, 255, 255)]

        color_index = 0

        robot_parameters = robots[0].parameters
        robot_width = robot_parameters.width
        printhead_slope = robot_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        #chunker_result = Chunker.start(robots, model_object, number_of_robots=2)

        chunker_result = Chunker.buildplate_chunking(robots, model_object, number_of_machines=2)

        chunk_row = Chunker.subdivide_row(chunker_result.origin_chunk,
                                          model_width,
                                          robot_width,
                                          printhead_slope,
                                          num_pieces=len(robots))

        chunk_number = len(chunk_row) - 1
        for (i, chunk) in enumerate(chunk_row):
            new_chunk = Chunk(chunk, number=i)
            new_chunk.color = colors[color_index]

            robot_number = i - (i % 2)
            robots[robot_number].chunks.append(new_chunk)

            new_chunk.set_name("Robot {} Chunk {}".format(robot_number, i))

            if i % 2 == 1:
                new_chunk.add_dependency(i - 1)
                if (i + 1) < len(chunk_row):
                    new_chunk.add_dependency(i + 1)

        center_chunk_dependencies = list(range(0, len(chunk_row)))
        previous_row_dependencies = list(range(0, len(chunk_row)))
        last_chunk_number = center_chunk_dependencies[-1]

        row_number = 0

        color_index = 1

        for south_chunk in chunker_result.south_chunks:
            row_number += 1

            chunk_row = Chunker.subdivide_row(south_chunk,
                                              model_width,
                                              robot_width,
                                              printhead_slope,
                                              num_pieces=len(robots))

            next_row_dependencies = []

            for (i, chunk) in enumerate(chunk_row):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk, number=chunk_number)
                new_chunk.row = -1 * row_number
                new_chunk.color = colors[color_index]
                new_chunk.add_dependency(previous_row_dependencies)

                robot_number = i - (i % 2)
                robots[robot_number].chunks.append(new_chunk)

                new_chunk.set_name("Robot {} Chunk {}".format(robot_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

            color_index += 1
            if color_index > 2:
                color_index = 0

        previous_row_dependencies = center_chunk_dependencies

        row_number = 0

        color_index = 2

        for north_chunk in chunker_result.north_chunks:
            row_number += 1
            chunk_row = Chunker.subdivide_row(north_chunk,
                                              model_width,
                                              robot_width,
                                              printhead_slope,
                                              num_pieces=len(robots))
            next_row_dependencies = []

            for (i, chunk) in enumerate(chunk_row):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk, number=chunk_number)
                new_chunk.row = row_number
                new_chunk.color = colors[color_index]
                new_chunk.add_dependency(previous_row_dependencies)

                robot_number = i - (i % 2) + 1
                robots[robot_number].chunks.append(new_chunk)

                new_chunk.set_name("Robot {} Chunk {}".format(robot_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

            color_index -= 1
            if color_index < 0:
                color_index = 2

    '''
    @staticmethod
    def start_scaled(robots: List[Robot], model_object: bpy.types.Object):
        """
        Performs SPAR3 chunking on ``model_object``, using ``robots``.
        \n
        In the cases where ``len(robots) < 4``, either single-robot chunking or two-robot
        chunking is used. Otherwise, full SPAR3 chunking is used.
        \n
        Assuming a proper two-robot chunk, the SPAR3 chunking process simply subdivides every chunk
        in the two-robot chunk lengthwise, and in the exact same way for every chunk. This method
        also takes care of assigning chunk dependencies and assigning chunks to the proper robot.

        :param robots: the robots that will be used to print this model
        :type robots: List[Robot]
        :param model_object: the model to be printed
        :type model_object: bpy.types.Object
        """

        if len(robots) == 1:
            Chunker.start_single(robots[0], model_object)
            return

        # red, white, and blue, Arkansas flag shades
        colors = [(255, 10, 27), (0, 30, 83), (255, 255, 255)]

        color_index = 0

        robot_parameters = robots[0].parameters
        robot_width = robot_parameters.width
        printhead_slope = robot_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        chunker_result = Chunker.buildplate_chunking(robots, model_object, number_of_machines=2)
        robot_num = list(range(len(robots)))
        chunk_row = Chunker.subdivide_row_newChunking(chunker_result.origin_chunk,
                                          model_width,
                                          robot_width,
                                          printhead_slope,
                                          num_pieces=len(robots))

        chunk_number = len(chunk_row) - 1
        for (i, chunk) in enumerate(chunk_row):
            new_chunk = Chunk(chunk, number=i)
            new_chunk.color = colors[color_index]

            #robot_number = i - (i % 2)
            if i >= len(robot_num):
                robot_number = robot_num[i % len(robot_num)] - (i % 2)
            else:
                robot_number = robot_num[i] - (i % 2)

            robots[robot_number].chunks.append(new_chunk)

            new_chunk.set_name("Robot {} Chunk {}".format(robot_number, i))

            if i % 2 == 1:
                new_chunk.add_dependency(i - 1)
                if (i + 1) < len(chunk_row):
                    new_chunk.add_dependency(i + 1)

        center_chunk_dependencies = list(range(0, len(chunk_row)))
        previous_row_dependencies = list(range(0, len(chunk_row)))
        last_chunk_number = center_chunk_dependencies[-1]

        row_number = 0

        color_index = 1

        for south_chunk in chunker_result.south_chunks:
            row_number += 1

            chunk_row = Chunker.subdivide_row_newChunking(south_chunk,
                                              model_width,
                                              robot_width,
                                              printhead_slope,
                                              num_pieces=len(robots))

            next_row_dependencies = []

            for (i, chunk) in enumerate(chunk_row):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk, number=chunk_number)
                new_chunk.row = -1 * row_number
                new_chunk.color = colors[color_index]
                new_chunk.add_dependency(previous_row_dependencies)

                #robot_number = i - (i % 2)
                if i >= len(robot_num):
                    robot_number = robot_num[i % len(robot_num)] - (i % 2)
                else:
                    robot_number = robot_num[i] - (i % 2)
                robots[robot_number].chunks.append(new_chunk)

                new_chunk.set_name("Robot {} Chunk {}".format(robot_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

            color_index += 1
            if color_index > 2:
                color_index = 0

        previous_row_dependencies = center_chunk_dependencies

        row_number = 0

        color_index = 2

        for north_chunk in chunker_result.north_chunks:
            row_number += 1
            chunk_row = Chunker.subdivide_row_newChunking(north_chunk,
                                              model_width,
                                              robot_width,
                                              printhead_slope,
                                              num_pieces=len(robots))
            next_row_dependencies = []

            for (i, chunk) in enumerate(chunk_row):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk, number=chunk_number)
                new_chunk.row = row_number
                new_chunk.color = colors[color_index]
                new_chunk.add_dependency(previous_row_dependencies)

                #robot_number = i - (i % 2) + 1
                if i >= len(robot_num):
                    robot_number = robot_num[i%len(robot_num)] - (i % 2) + 1
                else:
                    robot_number = robot_num[i] - (i % 2) + 1
                robots[robot_number].chunks.append(new_chunk)

                new_chunk.set_name("Robot {} Chunk {}".format(robot_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

            color_index -= 1
            if color_index < 0:
                color_index = 2


    @staticmethod
    def split_model(model: bpy.types.Object,
                    plane_data: Tuple[Vector, Vector]) -> Tuple[bpy.types.Object, bpy.types.Object]:
        """
        Splits ``model``, a Blender object, around a plane, specified by ``plane_data``.
        ``plane_data`` should be a tuple of ``(plane_coordinate: Vector, plane_normal: Vector)``
        \n
        Returns a tuple of the two chunks produced from the split. The first is on the
        negative side of the plane (relative to the normal), the second is the positive side of
        the plane.

        :param model: the Blender model to split
        :type model: bpy.types.Object
        :param plane_data: the plane around which ``model`` will split
        :type plane_data: Tuple[Vector, Vector]
        :return: two models, the result of splitting ``model`` around ``plane_data``
        :rtype: Tuple[bpy.types.Object, bpy.types.Object]
        """

        scene = bpy.context.scene
        Chunker.deselect_all_objects()
        model.select = True
        scene.objects.active = model

        plane_co = plane_data[0]
        plane_no = plane_data[1]

        bpy.ops.object.duplicate()

        duplicate = bpy.context.object
        if duplicate is None:
            return (None, None)

        duplicate.name = "Temp Model Chunk"

        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.bisect(
            plane_co=plane_co,
            plane_no=plane_no,
            use_fill=True,
            clear_inner=True
        )
        bpy.ops.object.mode_set(mode='OBJECT')

        temp_bmesh = bmesh.new()
        temp_bmesh.from_mesh(duplicate.data)

        bmesh.ops.triangulate(temp_bmesh, faces=temp_bmesh.faces[:], quad_method=0, ngon_method=0)

        temp_bmesh.to_mesh(duplicate.data)
        temp_bmesh.free()

        duplicate.select = False
        model.select = True
        scene.objects.active = model

        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.bisect(
            plane_co=plane_co,
            plane_no=plane_no,
            use_fill=True,
            clear_outer=True
        )
        bpy.ops.object.mode_set(mode='OBJECT')

        temp_bmesh = bmesh.new()
        temp_bmesh.from_mesh(model.data)

        bmesh.ops.triangulate(temp_bmesh, faces=temp_bmesh.faces[:], quad_method=0, ngon_method=0)

        temp_bmesh.to_mesh(model.data)
        temp_bmesh.free()

        return (model, duplicate)


    @staticmethod
    def subdivide_row(chunk: bpy.types.Object,
                      object_width: float,
                      robot_width: float,
                      printhead_slope: float,
                      num_pieces: int = -1) -> List[bpy.types.Object]:
        """
        Subdivides a chunk (from two-robot chunking) into SPAR3 chunks.

        :param chunk: the chunk to subdivide lengthwise
        :type chunk: bpy.types.Object
        :param object_width: the width of the entire model (not just this chunk)
        :type object_width: float
        :param robot_width: the width of an individual robot
        :type robot_width: float
        :param printhead_slope: the slope of the printhead, in radians
        :type printhead_slope: float
        :param num_pieces: the number of pieces this row should be split into
        :type num_pieces: int
        :return: a list of Blender objects resulting from the subdivision of ``chunk``
        :rtype: List[bpy.types.Object]
        """

        if num_pieces == 2:
            return [chunk]

        chunk_bounds = SimulatorMath.calculate_bounds(chunk)

        normal_z = math.tan(math.pi / 2 - printhead_slope)

        height = chunk_bounds.z.max - chunk_bounds.z.min
        slope_width = height * normal_z

        mid_y = (chunk_bounds.y.min + chunk_bounds.y.max) / 2

        plane_normal_east = Vector((1, 0, normal_z))
        plane_normal_west = Vector((-1, 0, normal_z))

        width = object_width
        half_width = width / 2
        if num_pieces != -1:
            chunks = num_pieces // 2
        else:
            chunks = math.ceil(half_width / (robot_width + slope_width / 2))

        chunk_row = []
        facing_east = True
        remaining_chunk = chunk

        piece_width = max(robot_width, width / num_pieces)

        for i in range(int(-chunks) + 1, chunks):
            plane_co = Vector((i * (piece_width + slope_width / 2), mid_y, 0))
            if facing_east:
                plane_no = plane_normal_east
            else:
                plane_no = plane_normal_west

            (west, east) = Chunker.split_model(remaining_chunk, (plane_co, plane_no))

            if facing_east:
                if not west.data.polygons:
                    chunk_row.append(Vector((plane_co[0] + (slope_width / 2), mid_y, 0)))
                else:
                    chunk_row.append(west)
                remaining_chunk = east
            else:
                if not east.data.polygons:
                    chunk_row.append(Vector(
                        (plane_co[0] - (slope_width / 2) - piece_width, mid_y, 0)))
                else:
                    chunk_row.append(east)
                remaining_chunk = west

            facing_east = not facing_east

        if not remaining_chunk.data.polygons:
            chunk_row.append(Vector((chunks * (piece_width + slope_width / 2), mid_y, 0)))
        else:
            chunk_row.append(remaining_chunk)

        return chunk_row


    @staticmethod
    def subdivide_row_newChunking(chunk: bpy.types.Object,
                      object_width: float,
                      robot_width: float,
                      printhead_slope: float,
                      num_pieces: int = -1) -> List[bpy.types.Object]:
        """
        Subdivides a chunk (from two-robot chunking) into SPAR3 chunks.

        :param chunk: the chunk to subdivide lengthwise
        :type chunk: bpy.types.Object
        :param object_width: the width of the entire model (not just this chunk)
        :type object_width: float
        :param robot_width: the width of an individual robot
        :type robot_width: float
        :param printhead_slope: the slope of the printhead, in radians
        :type printhead_slope: float
        :param num_pieces: the number of pieces this row should be split into
        :type num_pieces: int
        :return: a list of Blender objects resulting from the subdivision of ``chunk``
        :rtype: List[bpy.types.Object]
        """

        if num_pieces == 2:
            return [chunk]

        chunk_bounds = SimulatorMath.calculate_bounds(chunk)

        normal_z = math.tan(math.pi / 2 - printhead_slope)

        height = chunk_bounds.z.max - chunk_bounds.z.min
        slope_width = height * normal_z

        mid_y = (chunk_bounds.y.min + chunk_bounds.y.max) / 2

        plane_normal_east = Vector((1, 0, normal_z))
        plane_normal_west = Vector((-1, 0, normal_z))


        buildplate_dimension = Chunker.buildplate_dimension
        robot_reach_adjacent = Chunker.robot_reach_adjacent
        min_top_dim = Chunker.min_top_dim
        wing_slope = Chunker.wing_slope
        center_col = min_top_dim + 2 * wing_slope
        non_center_col = 1/2 * (buildplate_dimension+2*robot_reach_adjacent-(center_col))
        chunke_per_buildpalte = 3

        width = object_width
        half_width = width / 2
        #if num_pieces != -1:
         #   chunks = num_pieces // 2
        #else:
        chunks = math.ceil(half_width/(buildplate_dimension + 2*robot_reach_adjacent)*chunke_per_buildpalte)

        chunk_row = []
        facing_east = True
        remaining_chunk = chunk

        piece_width = max(non_center_col, width/num_pieces)

        for i in range(int(-chunks) + 1, chunks):
            plane_co = Vector((i * (piece_width + slope_width / 2), mid_y, 0))
            if facing_east:
                plane_no = plane_normal_east
            else:
                plane_no = plane_normal_west

            (west, east) = Chunker.split_model(remaining_chunk, (plane_co, plane_no))

            if facing_east:
                if not west.data.polygons:
                    chunk_row.append(Vector((plane_co[0] + (slope_width / 2), mid_y, 0)))
                else:
                    chunk_row.append(west)
                remaining_chunk = east
            else:
                if not east.data.polygons:
                    chunk_row.append(Vector(
                        (plane_co[0] - (slope_width / 2) - piece_width, mid_y, 0)))
                else:
                    chunk_row.append(east)
                remaining_chunk = west

            facing_east = not facing_east

        if not remaining_chunk.data.polygons:
            chunk_row.append(Vector((chunks * (piece_width + slope_width / 2), mid_y, 0)))
        else:
            chunk_row.append(remaining_chunk)

        return chunk_row