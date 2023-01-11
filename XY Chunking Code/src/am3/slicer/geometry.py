"""
Contains classes for managing geometric data. Includes:

    * Line segments
    * Planes
    * Triangles
    * Triangle Meshes
"""

from typing import List
from mathutils import Vector

def roundSig(x: float, sig: int=3) -> float:
    """
    Rounds a number to ``sig`` decimal places.

    :param x: the number to round
    :type x: float
    :param sig: the number of decimal places (default ``3``)
    :type sig: int
    :return: the rounded number
    :rtype: float
    """

    format_string = "{0:." + str(sig) + "f}"
    return float(format_string.format(x))

class LineSegment:
    """
    Defines a line segment using two 3D points, ``p0`` and ``p1``.
    """

    vertices = []
    """
    The list of vertices for this line segment. In this case, the list length will be 2

    :type: List[Vector]
    """

    # p0: Vector, p1: Vector
    def __init__(self, p0 = Vector((0,0,0)), p1 = Vector((0,0,0))):
        """
        :param p0: the start point of the line segment
        :type p0: Vector
        :param p1: the end point of the line segment
        :type p1: Vector
        """

        self.vertices = []
        self.vertices.append(p0)
        self.vertices.append(p1)

    def flip(self):
        """
        Flips the start/end order of the points. See the following example::

            p0 = Vector((0, 0, 0))
            p1 = Vector((0, 0, 5))
            ls = LineSegment(p0, p1)
            print(ls.vertices) # [(0, 0, 0), (0, 0, 5)]
            ls.flip()
            print(ls.vertices) # [(0, 0, 5), (0, 0, 0)]
        
        """

        temp = self.vertices[0]
        self.vertices[0] = self.vertices[1]
        self.vertices[1] = temp

    def shrink(self, inset: float):
        """
        Decreases the Y-length of the link segment by ``inset``. For example::

            ls1 = LineSegment(Vector((0, 0, 0)), Vector(0, 5, 0))
            print(ls1.vertices) # [(0, 0, 0), (0, 5, 0)]
            ls1.shrink(1.5)
            print(ls1.vertices) # [(0, 1.5, 0), (0, 3.5, 0)]

            ls2 = LineSegment(Vector((10, 20, 10)), Vector((10, 5, 10)))
            ls2.shrink(5)
            print(ls2.vertices) # [(10, 15, 10), (10, 10, 10)]
        
        :param inset: the amount to shrink the Y length of this LineSegment by
        :type inset: float
        """
        if (self.vertices[0][1] > self.vertices[1][1]):
            self.vertices[0][1] -= inset
            self.vertices[1][1] += inset
        else:
            self.vertices[0][1] += inset
            self.vertices[1][1] -= inset

    def simplify(self, sigfigs: int = 3):
        """
        Rounds each coordinate of this LineSegment to ``sigfigs`` decimal places::

            ls1 = LineSegment(Vector(0.12345, 0.12, 1.2), Vector(10, 10.5432, 123456))
            ls1.simplify(3)
            print(ls1.vertices) # [(0.123, 0.12, 1.2), (10, 10.543, 123456)]

        :param sigfigs: the number of decimal places to round to
        :type sigfigs: int
        """
        for vertex in self.vertices:
            vertex[0] = roundSig(vertex[0], sigfigs)
            vertex[1] = roundSig(vertex[1], sigfigs)
            vertex[2] = roundSig(vertex[2], sigfigs)

    def getFirst(self) -> Vector:
        """
        :return: the first vertex of this line segment
        :rtype: Vector
        """

        return self.vertices[0]

    def getSecond(self):
        """
        :return: the second vertex of this line segment
        :rtype: Vector
        """

        return self.vertices[1]

class Plane:
    """
    Defines a horizontal plane at a specific height (i.e. ``distance``) above the ground plane.
    """

    distance = 0
    """
    The distance from the ground plane

    :type: float
    """

    normal = Vector((0, 0, 0))
    """
    The normal vector out of the face of this plane

    :type: Vector
    """

    def __init__(self):
        """
        Creates a new Plane, with a distance of ``0`` and a meaningless normal vector.
        """

        self.distance = 0
        self.normal = Vector((0,0,0))

    def setNormal(self, normal: Vector):
        """
        :param normal: a normal Vector to set this plane to.
        :type normal: Vector
        """

        self.normal = normal

    def setDistance(self, distance: float):
        """
        :param distance: the distance above the ground plane to set this plane to.
        :type distance: float
        """

        self.distance = distance

    def distanceToPoint(self, point: Vector) -> float:
        """
        Calculates the distance that ``point`` is from the surface of this plane.

        :param point: the point to measure distance to
        :type point: Vector
        :return: the distance the plane is from ``point``
        :rtype: float
        """

        return point.dot(self.normal) - self.distance


class Triangle:
    """
    Represents a Triangle using three Vectors.
    """

    vertices = []
    """
    A List representing the Triangle. Its size is always 3.

    :type: List[Vector]
    """

    def __init__(self, v0: Vector, v1: Vector, v2: Vector):
        """
        Initialize a triangle using three vertices, ``v0``, ``v1``, and ``v2``.

        :type v0: Vector
        :type v1: Vector
        :type v2: Vector
        """

        self.vertices = []
        self.vertices.append(v0)
        self.vertices.append(v1)
        self.vertices.append(v2)

    def subtract(self, point: Vector):
        """
        Subtracts ``point`` from each vertex in this triangle

        :param point: a point in 3D space
        :type point: Vector
        """

        self.vertices[0] -= point
        self.vertices[1] -= point
        self.vertices[2] -= point

    def intersectPlane(self, plane: Plane, line_segment: LineSegment) -> int:
        """
        Determines the intersection and its type this triangle has with ``plane``. The intersection
        as a LineSegment is stored in ``line_segment``, while the function itself returns an int
        corresponding to the type of intersection.

        Intersection types:

            -1:
                the triangle is underneath the plane
            0:
                the plane intersects the triangle
            1:
                the triangle is above the plane
            -2:
                the function errored


        :param plane: the plane to intersect with
        :type plane: Plane
        :param line_segment: the LineSegment in which to store the intersection, if any
        :type line_segment: LineSegment
        :return: an int, indicating what type of intersection occurred.
        :rtype: int
        """

        countFront = 0
        countBack = 0

        for j in range(0, 3):
            distance = plane.distanceToPoint(self.vertices[j])
            if distance < 0:
                countBack += 1
            else:
                countFront += 1

        if countBack == 3:
            return -1
        elif countFront == 3:
            return 1

        lines = [0, 1, 1, 2, 2, 0] #CCW triangle
        intersectPoints = []

        for i in range(0, 3):
            a = self.vertices[lines[i * 2 + 0]]
            b = self.vertices[lines[i * 2 + 1]]
            da = plane.distanceToPoint(a)
            db = plane.distanceToPoint(b)
            if da * db < 0:
                s = da / (da - db) # intersection factor (between 0 and 1)
                b_minus_a = b - a
                intersectPoints.append(a+b_minus_a*s)
            elif da == 0:
                if len(intersectPoints) < 2:
                    intersectPoints.append(a)
            elif db == 0:
                if len(intersectPoints) < 2:
                    intersectPoints.append(b)

        if len(intersectPoints) == 2:
            line_segment.vertices[0] = intersectPoints[0]
            line_segment.vertices[1] = intersectPoints[1]
            return 0

        return -2


class TriangleMesh:
    """
    Represents a triangular mesh using a list of Triangles (see :class:`~am3.slicer.geometry.Triangle`).
    """

    mesh = []
    """
    Represents a triangular mesh using a list of Triangles

    :type: List[Triangle]
    """

    bottomLeftVertex = Vector((999999, 999999, 999999))
    """
    Represents the bottom left corner of the mesh boundary (where \"bottom\" and \"left\" only mean
    anything when related to the position of the ``upperRightVertex``).

    :type: Vector
    """

    upperRightVertex = Vector((-999999, -999999, -999999))
    """
    Represents the upper right corner of the mesh boundary (where \"upper\" and \"right\" only mean
    anything when related to the position of the ``bottomLeftVertex``)

    :type: Vector
    """

    def __init__(self):
        """
        Initializes an emtpy triangular mesh, with an inverted, large bounding box.
        """

        self.bottomLeftVertex = Vector((999999, 999999, 999999))
        self.upperRightVertex = Vector((-999999, -999999, -999999))
        self.mesh = []

    def normalize(self):
        """
        Normalizes a triangular mesh, meaning the mesh is centered around (0, 0, 0)
        """

        box_midpoint = (self.upperRightVertex - self.bottomLeftVertex) / 2
        start = self.bottomLeftVertex + box_midpoint

        for i in range(0, len(self.mesh)):
            self.mesh[i].subtract(start)

        self.bottomLeftVertex = box_midpoint * -1
        self.upperRightVertex = box_midpoint

    def append(self, triangle: Triangle):
        """
        Adds a Triangle to this mesh. After a new triangle is added, it must be tested whether or
        not this triangle requires a boundary expansion.

        :param triangle: the Triangle to add to this mesh
        :type triangle: Triangle
        """

        self.mesh.append(triangle)
        for i in range(0, 3):
            first = triangle.vertices[i].x
            second = self.bottomLeftVertex[0]

            if first < second:
                self.bottomLeftVertex[0] = triangle.vertices[i][0]
            if triangle.vertices[i][1] < self.bottomLeftVertex[1]:
                self.bottomLeftVertex[1] = triangle.vertices[i][1]
            if triangle.vertices[i][2] < self.bottomLeftVertex[2]:
                self.bottomLeftVertex[2] = triangle.vertices[i][2]
            if triangle.vertices[i][0] > self.upperRightVertex[0]:
                self.upperRightVertex[0] = triangle.vertices[i][0]
            if triangle.vertices[i][1] > self.upperRightVertex[1]:
                self.upperRightVertex[1] = triangle.vertices[i][1]
            if triangle.vertices[i][2] > self.upperRightVertex[2]:
                self.upperRightVertex[2] = triangle.vertices[i][2]

    def meshAABBSize(self) -> Vector:
        """
        Returns the AABB size of this triangular mesh, or, more simply, the box in X, Y, Z
        dimensions consumed by this triangular mesh.

        :return: a Vector representing the X, Y, Z dimensions of this mesh.
        :rtype: Vector
        """

        x = self.upperRightVertex[0] - self.bottomLeftVertex[0]
        y = self.upperRightVertex[1] - self.bottomLeftVertex[1]
        z = self.upperRightVertex[2] - self.bottomLeftVertex[2]
        return Vector((x,y,z))
