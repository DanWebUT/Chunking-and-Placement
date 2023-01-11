from mathutils import Vector
from mathutils import Euler
import math

import am3
from am3.slicer.geometry import LineSegment

class SimulatorMath:
    @staticmethod
    def calculateBounds(obj, local=False):
        local_coords = obj.bound_box[:]
        om = obj.matrix_world
        
        if not local:
            worldify = lambda p: om * Vector(p[:])
            coords = [worldify(p).to_tuple() for p in local_coords]
        else:
            coords = [p[:] for p in local_coords]
        
        rotated = zip(*coords[::-1])
        
        push_axis = []
        for (axis, _list) in zip('xyz', rotated):
            info = lambda: None
            info.max = max(_list)
            info.min = min(_list)
            info.distance = info.max - info.min
            push_axis.append(info)
        
        import collections
        
        originals = dict(zip(['x', 'y', 'z'], push_axis))
        
        o_details = collections.namedtuple('object_details', 'x y z')
        return o_details(**originals)

    # Returns the shortest distance from point to the line. Disregards Z coordinate
    @staticmethod
    def distanceToLine(point, line):
        x1 = line[0][0]
        x2 = line[1][0]
        x3 = point[0]

        y1 = line[0][1]
        y2 = line[1][1]
        y3 = point[1]

        px = x2-x1
        py = y2-y1

        something = px*px + py*py

        u =  ((x3 - x1) * px + (y3 - y1) * py) / float(something)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = x1 + u * px
        y = y1 + u * py

        dx = x - x3
        dy = y - y3

        # Note: If the actual distance does not matter,
        # if you only want to compare what this function
        # returns to other results of this function, you
        # can just return the squared distance instead
        # (i.e. remove the sqrt) to gain a little performance

        dist = math.sqrt(dx*dx + dy*dy)

        return dist

    # 2 dimensional distance formula
    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    # Determines if point is to the right or left of the line.
    # Returns -1, 0, or 1, depending on which side the point is on
    @staticmethod
    def sideOfLine(point, line):
        a = line[0]
        b = line[1]
        c = point

        return SimulatorMath.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))

    @staticmethod
    def sign(number):
        if (number > 0):
            return 1
        if (number < 0):
            return -1
        return 0

    # Checks if line (in XY plane) intersects bounds (in XY plane).
    # Bound corner labels:
    #       A---B
    #   ^   |   |
    #   |   D---C
    #   Y
    #   |
    #   +-X-->
    # Returns: True if line intersects bounds, False otherwise
    @staticmethod
    def lineIntersectsBounds(line, bounds):
        # Number of intersections. Must be exactly 2 to be a valid intersection
        intersection_count = 0
        boundAB = [(bounds.x.min, bounds.y.max), (bounds.x.max, bounds.y.max)]
        boundBC = [(bounds.x.max, bounds.y.max), (bounds.x.max, bounds.y.min)]
        boundCD = [(bounds.x.max, bounds.y.min), (bounds.x.min, bounds.y.min)]
        boundDA = [(bounds.x.min, bounds.y.min), (bounds.x.min, bounds.y.max)]
        bound_lines = [boundAB, boundBC, boundCD, boundDA]

        for bound_line in bound_lines:
            if SimulatorMath.segmentsIntersect(line, bound_line):
                intersection_count += 1

        return (intersection_count == 2)

    @staticmethod
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    @staticmethod
    def segmentsIntersect(seg1, seg2):
        A = seg1[0]
        B = seg1[1]
        C = seg2[0]
        D = seg2[1]
        return SimulatorMath.ccw(A,C,D) != SimulatorMath.ccw(B,C,D) and SimulatorMath.ccw(A,B,C) != SimulatorMath.ccw(A,B,D)

    @staticmethod
    def polygonLiesInPlane(mesh, polygon, plane):

        for vert in polygon.vertices:

            if not SimulatorMath.vertexLiesInPlane(mesh.vertices[vert].co, plane):
                return False

        return True

    @staticmethod
    def vertexLiesInPlane(vertex, plane):
        plane_co = plane[0]
        plane_no = plane[1]

        val = abs(plane_no.dot(Vector(vertex) - plane_co))

        return val < 0.1

    @staticmethod
    def linearInterpolation(p0, p1, percentage):
        x_dist = p1[0] - p0[0]
        y_dist = p1[1] - p0[1]
        z_dist = p1[2] - p0[2]

        x_pos = p0[0] + x_dist * percentage
        y_pos = p0[1] + y_dist * percentage
        z_pos = p0[2] + z_dist * percentage

        return Vector((x_pos, y_pos, z_pos))

    @staticmethod
    def getIntersectionAtXPosition(lineSegment, xPosition):
        minX = min(lineSegment.getFirst()[0], lineSegment.getSecond()[0])
        maxX = max(lineSegment.getFirst()[0], lineSegment.getSecond()[0])

        if (minX == maxX):
            return None

        xLength = lineSegment.getSecond()[0] - lineSegment.getFirst()[0]

        if xPosition < minX or xPosition > maxX:
            return None

        distanceToFirstPointX = xPosition -  lineSegment.getFirst()[0]
        yLength = lineSegment.getSecond()[1] - lineSegment.getFirst()[1]

        yShift = abs(yLength * (distanceToFirstPointX / xLength))

        zPosition = lineSegment.getFirst()[2]

        if yLength > 0:
            return Vector((xPosition, lineSegment.getFirst()[1] + yShift, zPosition))
        else:
            return Vector((xPosition, lineSegment.getFirst()[1] - yShift, zPosition))

    @staticmethod
    def getExtremeXValues(bounds):
        leftExtreme = bounds[0]
        rightExtreme = bounds[0]

        for point in bounds:
            if point[0] < leftExtreme[0]:
                leftExtreme = point
            if point[0] > rightExtreme[0]:
                rightExtreme = point

        return leftExtreme[0], rightExtreme[0]

    @staticmethod
    def getIntersectionLinesForBounds(bounds, thickness):
        (leftXValue, rightXValue) = SimulatorMath.getExtremeXValues(bounds)

        xValues = []
        xValues.append(leftXValue - thickness)

        xValue = leftXValue
        while (xValue <= rightXValue + thickness):
            xValues.append(xValue)
            xValue += thickness

        return xValues

    @staticmethod
    def getIntersectionsForLineSegments(xValue, points):
        intersections = []

        for i in range(0, len(points) - 1):
            segment = LineSegment(points[i], points[i+1])

            intersection = SimulatorMath.getIntersectionAtXPosition(segment, xValue)
            if intersection != None:
                intersections.append(intersection)

        intersections.sort(key=lambda point: point[1])
        return intersections

    @staticmethod
    def estimate_execution_time(machines):
        chunks = []
        for machine in machines:
            for chunk in machine.chunks:
                chunks.append(chunk)

        network = {}
        for chunk in chunks:
            network[chunk.number] = chunk

        n = len(chunks)

        current_max = 0

        for i in range(0, n):
            current_max = max(current_max, SimulatorMath.exec_time(network, network[i]))
        
        return current_max

    def exec_time(network, chunk):
        if chunk.execution_time != -1:
            return chunk.execution_time

        t_c = len(chunk.frame_data)

        current_max = 0
        for i in chunk.dependencies:
            current_max = max(current_max, SimulatorMath.exec_time(network, network[i]))
        
        chunk.execution_time = current_max + t_c
        return current_max + t_c