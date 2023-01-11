"""
The ``am3.util`` module contains the ``SimulatorMath`` class, which is used to perform
miscellaneous calculations needed throughout the chunking, slicing, and simulating processes.
"""

import math
from typing import List, Tuple, Dict
import bpy
from mathutils import Vector
from am3.slicer.geometry import LineSegment


class SimulatorMath:
    """
    Contains static methods for performing (mostly) geometric calculations needed throughout the
    Cooperative 3D simulation process.
    """

    @staticmethod
    def calculate_bounds(model: bpy.types.Object) -> object:
        """
        Returns a ``bounds`` object, which contains the X, Y, and Z direction boundaries for
        the ``model`` parameter. It will have the following format::

            {
                x: {
                    min: ...,
                    max: ...,
                    distance: ...,
                },
                y: {
                    // same for 'y'
                },
                z: {
                    // same for 'z'
                }
            }

        :param model: a model for which to calculate the bounds.
        :type model: bpy.types.Object
        :return: An object containing boundary information for ``model``
        :rtype: object
        """

        local_coords = model.bound_box[:]
        om = model.matrix_world

        worldify = lambda p: om * Vector(p[:])
        coords = [worldify(p).to_tuple() for p in local_coords]

        rotated = zip(*coords[::-1])

        push_axis = []
        for (_, _list) in zip('xyz', rotated):
            info = lambda: None
            info.max = max(_list)
            info.min = min(_list)
            info.distance = info.max - info.min
            push_axis.append(info)

        import collections

        originals = dict(zip(['x', 'y', 'z'], push_axis))

        o_details = collections.namedtuple('object_details', 'x y z')
        return o_details(**originals)

    @staticmethod
    def distance_to_line(point: Vector, line: List[Vector]) -> float:
        """
        Returns the shortest distance from a point to a line, ignoring z-distance. For example:

        .. image:: ../_static/img/distanceToLine-1.png
            :scale: 50%

        In this example, ``point`` is at (5,3), ``line`` goes from (2,2) to (4,6), and the
        shortest distance between the two is ``sqrt(5)``, or roughly ``2.236``.

        Given the values from the above figure, this function would return about ``2.236``.

        :param point: a point to measure the distance from
        :type point: Vector
        :param line: the line to which to measure the distance
        :type line: List[Vector]
        :return: The shortest Euclidean distance from ``point`` to ``line``
        :rtype: float
        """

        x1 = line[0][0]
        x2 = line[1][0]
        x3 = point[0]

        y1 = line[0][1]
        y2 = line[1][1]
        y3 = point[1]

        px = x2-x1
        py = y2-y1

        something = px*px + py*py

        u = ((x3 - x1) * px + (y3 - y1) * py) / float(something)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = x1 + u * px
        y = y1 + u * py

        dx = x - x3
        dy = y - y3

        dist = math.sqrt(dx*dx + dy*dy)

        return dist

    @staticmethod
    def distance(p1: Vector, p2: Vector) -> float:
        """
        Returns the Euclidean distance between two points.

        :param p1: the first point
        :type p1: Vector
        :param p2: the second point
        :type p2: Vector
        :return: the Euclidean distance between ``p1`` and ``p2``
        :rtype: float
        """

        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    @staticmethod
    def side_of_line(point: Vector, line: List[Vector]) -> int:
        """
        Determines where a point is relative to a line (i.e. to the right or left of it). The idea
        of \"right\" and \"left\" are relative to the line's direction. So, ``point`` relative
        to ``line``, taken as a directional vector, is on the right if, when standing on the start
        of the line segment and turning to face the end of the line segment, it would appear on the
        right to a viewer. Same goes for left.

        If ``point`` is on ``line``, then 0 is returned. Otherwise, ``-1`` indicates left, and ``1``
        indicates right.

        :param point: the point to test
        :type point: Vector
        :param line: the line to test
        :type line: List[Vector]
        :return: The side of the line ``point`` occurs on (-1 if left, 1 if right, 0 if on 
            ``line``)
        :rtype: int
        """
        a = line[0]
        b = line[1]
        c = point

        return SimulatorMath.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))

    @staticmethod
    def sign(number: float) -> int:
        """
        Returns the sign of a signed number as the unit (i.e., ``1``).

        For example::

            sign(10) -> 1
            sign(-10) -> -1
            sign(0) -> 0

        :param number: a number for which to find the sign
        :type number: float
        :return: the sign of the number
        :rtype: int
        """

        if number > 0:
            return 1
        if number < 0:
            return -1
        return 0

    @staticmethod
    def line_intersects_bounds(line: Vector, bounds: object) -> bool:
        """
        Checks if ``line`` fully intersects a rectangular bounds. The line must pass all the way
        through the boundary. The bounds should be computed using
        :func:`~am3.util.SimulatorMath.calculate_bounds`.

        :param line: the line to check for intersection
        :type line: List[Vector]
        :param bounds: a boundary object
        :type bounds: object
        :return: whether ``line`` intersects ``bounds`` in the XY plane
        :rtype: bool
        """

        # Number of intersections. Must be exactly 2 to be a valid intersection
        intersection_count = 0
        bound_a_b = [(bounds.x.min, bounds.y.max), (bounds.x.max, bounds.y.max)]
        bound_b_c = [(bounds.x.max, bounds.y.max), (bounds.x.max, bounds.y.min)]
        bound_c_d = [(bounds.x.max, bounds.y.min), (bounds.x.min, bounds.y.min)]
        bound_d_a = [(bounds.x.min, bounds.y.min), (bounds.x.min, bounds.y.max)]
        bound_lines = [bound_a_b, bound_b_c, bound_c_d, bound_d_a]

        for bound_line in bound_lines:
            if SimulatorMath.segments_intersect(line, bound_line):
                intersection_count += 1

        return intersection_count == 2

    @staticmethod
    def ccw(a: Vector, b: Vector, c: Vector) -> bool:
        """
        Determines if the points ``a``, ``b``, and ``c`` are arranged in a counter-clockwise
        positioning.

        :param a: a point
        :type a: Vector
        :param b: a point
        :type b: Vector
        :param c: a point
        :type c: Vector
        :return: True if the points are counterclockwise, False otherwise
        :rtype: bool
        """

        return (c[1]-a[1]) * (b[0]-a[0]) > (b[1]-a[1]) * (c[0]-a[0])

    @staticmethod
    def segments_intersect(seg1: List[Vector], seg2: List[Vector]) -> bool:
        """
        Determines if the line segments ``seg1`` and ``seg2`` intersect each other in the XY plane.

        :param seg1: a line segment
        :type seg1: List[Vector]
        :param seg2: a line segment
        :type seg2: List[Vector]
        :return: Whether or not the line segments intersect
        :rtype: bool
        """

        a = seg1[0]
        b = seg1[1]
        c = seg2[0]
        d = seg2[1]
        return SimulatorMath.ccw(a, c, d) != SimulatorMath.ccw(b, c, d) and \
            SimulatorMath.ccw(a, b, c) != SimulatorMath.ccw(a, b, d)

    @staticmethod
    def linear_interpolation(p0: Vector, p1: Vector, percentage: float) -> Vector:
        """
        Interpolates a point from ``p0`` to ``p1`` by the value of ``percentage``. ``percentage``
        should vary from ``0`` to ``1``.

        For example::

            p0 = Vector((0, 0, 0))
            p1 = Vector((0, 5, 0))
            interpolated = lineaer_interpolation(p0, p1, 0.5)
            print(interpolated) # prints \"(0, 2.5, 0)\"

        :param p0: the start point
        :type p0: Vector
        :param p1: the end point
        :type p1: Vector
        :param percentage: the percentage along which to interpolate
        :type percentage: float
        :return: the interpolated point
        :rtype: Vector
        """

        x_dist = p1[0] - p0[0]
        y_dist = p1[1] - p0[1]
        z_dist = p1[2] - p0[2]

        x_pos = p0[0] + x_dist * percentage
        y_pos = p0[1] + y_dist * percentage
        z_pos = p0[2] + z_dist * percentage

        return Vector((x_pos, y_pos, z_pos))

    @staticmethod
    def get_intersection_at_x_position(line_segment: LineSegment, x_position: float) -> Vector:
        """
        Returns the intersection of ``line_segment`` with the vertical YZ plane at ``x_position``.

        :param line_segment: the line segment to intersect
        :type line_segment: line_segment
        :param x_position: the position of the YZ plane to intersect
        :type x_position: float
        :return: the intersection of ``line_segment`` with the YZ plane at ``x_position``.
        :rtype: Vector
        """

        min_x = min(line_segment.getFirst()[0], line_segment.getSecond()[0])
        max_x = max(line_segment.getFirst()[0], line_segment.getSecond()[0])

        if min_x == max_x:
            return None

        x_length = line_segment.getSecond()[0] - line_segment.getFirst()[0]

        if x_position < min_x or x_position > max_x:
            return None

        distance_to_first_point_x = x_position -  line_segment.getFirst()[0]
        y_length = line_segment.getSecond()[1] - line_segment.getFirst()[1]

        y_shift = abs(y_length * (distance_to_first_point_x / x_length))

        z_position = line_segment.getFirst()[2]

        if y_length > 0:
            return Vector((x_position, line_segment.getFirst()[1] + y_shift, z_position))

        return Vector((x_position, line_segment.getFirst()[1] - y_shift, z_position))

    @staticmethod
    def get_extreme_x_values(bounds: List[Vector]) -> Tuple[float, float]:
        """
        Returns the lowest and highest X values encountered in ``bounds`` as a Tuple.

        :param bounds: a list of points to find the min and max X values.
        :type bounds: List[Vector]
        :return: the two X values at the min and max of ``bounds``
        :rtype: Tuple[float, float]
        """

        left_extreme = bounds[0]
        right_extreme = bounds[0]

        for point in bounds:
            if point[0] < left_extreme[0]:
                left_extreme = point
            if point[0] > right_extreme[0]:
                right_extreme = point

        return left_extreme[0], right_extreme[0]

    @staticmethod
    def get_intersection_lines_for_bounds(bounds: List[Vector], thickness: float) -> List[float]:
        """
        Returns a list of x-values that span from/to the min/max x-values of ``bounds``,
        respectively. This list of x-values can be used to perform infill slicing. The x-values
        will be ``thickness`` apart.

        :param bounds: the bounds, or line segments, to calculate x-values for
        :type bounds: List[Vector]
        :param thickness: the spacing between x-values.
        :type thickness: float
        :return: the list of x-values spanning ``bounds``, separated by ``thickness``
        :rtype: List[float]
        """

        (left_x_value, right_x_value) = SimulatorMath.get_extreme_x_values(bounds)

        x_values = []
        x_values.append(left_x_value - thickness)

        x_value = left_x_value
        while x_value <= right_x_value + thickness:
            x_values.append(x_value)
            x_value += thickness

        return x_values

    @staticmethod
    def get_intersections_for_line_segments(x_value: float, points: List[Vector]) -> List[Vector]:
        """
        Returns a list of points where the consecutive line segments represented by ``points``
        intersects the YZ plane at the X position ``x_value``. The result will be sorted by Y value.

        :param x_value: the x_value to intersect
        :type x_value: float
        :param points: the points to intersect against the YZ plane at position ``x_value``
        :type points: List[Vector]
        :return: the points where ``points`` intersected the YZ plane at ``x_value``
        :rtype: List[Vector]
        """

        intersections = []

        for i in range(0, len(points) - 1):
            segment = LineSegment(points[i], points[i+1])

            intersection = SimulatorMath.get_intersection_at_x_position(segment, x_value)
            if intersection is not None:
                intersections.append(intersection)

        intersections.sort(key=lambda point: point[1])
        return intersections

    @staticmethod
    def estimate_execution_time(robots: List['Robot']) -> int:
        """
        Returns the expected execution time (in frames) for a simulation that would be executed
        on ``robots``. The Robots should possess Chunks, and the Chunks should already be
        sliced, with frames generated.

        :param robots: the robots that would be simulated
        :type robots: List[Robot]
        :return: the amount of time that will be taken to simulate, in units of frames.
        :rtype: int
        """

        chunks = []
        for robot in robots:
            for chunk in robot.chunks:
                chunks.append(chunk)

        network = {}
        for chunk in chunks:
            network[chunk.number] = chunk

        n = len(chunks)
        current_max = 0

        for i in range(0, n):
            current_max = max(current_max, SimulatorMath.exec_time(network, network[i]))

        return current_max

    @staticmethod
    def exec_time(network: Dict[int, 'Chunk'], chunk: 'Chunk') -> int:
        """
        Returns the amount of time taken to print a chunk in the simulation AND all of its
        dependencies. This time is stored in ``chunk.execution_time``, as well as returned.

        :param network: the dependency network/tree/graph
        :type network: Dict[int, Chunk]
        :param chunk: the chunk for which to evaluate the execution time (including dependencies)
        :type chunk: Chunk
        :return: the execution time for ``chunk``, including its dependencies
        :rtype: int
        """

        if chunk.execution_time != -1:
            return chunk.execution_time

        t_c = len(chunk.frame_data)

        current_max = 0
        for i in chunk.dependencies:
            current_max = max(current_max, SimulatorMath.exec_time(network, network[i]))

        chunk.execution_time = current_max + t_c
        return current_max + t_c
