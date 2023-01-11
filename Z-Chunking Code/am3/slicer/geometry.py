from mathutils import *

def roundSig(x, sig=3):
    return float("{0:.3f}".format(x))

class LineSegment:
    # p0: Vector, p1: Vector
    def __init__(self, p0 = Vector((0,0,0)), p1 = Vector((0,0,0))):
        self.vertices = []
        self.vertices.append(p0)
        self.vertices.append(p1)

    def flip(self):
        temp = self.vertices[0]
        self.vertices[0] = self.vertices[1]
        self.vertices[1] = temp

    def shrink(self, inset):
        if (self.vertices[0][1] > self.vertices[1][1]):
            self.vertices[0][1] -= inset
            self.vertices[1][1] += inset
        else:
            self.vertices[0][1] += inset
            self.vertices[1][1] -= inset

    def simplify(self, sigfigs = 3):
        for vertex in self.vertices:
            vertex[0] = roundSig(vertex[0], sigfigs)
            vertex[1] = roundSig(vertex[1], sigfigs)
            vertex[2] = roundSig(vertex[2], sigfigs)

    def getFirst(self):
        return self.vertices[0]

    def getSecond(self):
        return self.vertices[1]

class Plane:
    def __init__(self):
        self.distance = 0
        self.normal = Vector((0,0,0))

    # normal: Vector
    def setNormal(self, normal):
        self.normal = normal

    #distance: float
    def setDistance(self, distance):
        self.distance = distance

    #point: Vector
    def distanceToPoint(self, point):
        return point.dot(self.normal) - self.distance


class Triangle:

    # normal: Vector, v0: Vector, v1: Vector, v2: Vector
    def __init__(self, normal, v0, v1, v2):
        self.v = []
        self.v.append(v0)
        self.v.append(v1)
        self.v.append(v2)
        self.normal = normal

    # here, we're expecting "point" to be a Vector
    def sub(self, point):
        self.v[0] -= point
        self.v[1] -= point
        self.v[2] -= point

    #p: Plane, ls: LineSegment, @return Integer corresponding to:
    #                          -1 = all triangle is on plane back side
    #                           0 = plane intersects the triangle
    #                           1 = all triangle is on plane front side
    #                          -2 = error in function
    def intersectPlane(self, p, ls):
        countFront = 0
        countBack = 0

        for j in range(0, 3):
            distance = p.distanceToPoint(self.v[j])
            if distance < 0:
                countBack += 1
            else:
                countFront += 1

        if countBack == 3:
            return -1
        elif countFront == 3:
            return 1

        lines = [0,1,1,2,2,0] #CCW triangle
        intersectPoints = []

        for i in range(0, 3):
            a = self.v[lines[i*2+0]]
            b = self.v[lines[i*2+1]]
            da = p.distanceToPoint(a)
            db = p.distanceToPoint(b)
            if da*db < 0:
                s = da/(da-db) # intersection factor (between 0 and 1)
                b_minus_a = b - a
                intersectPoints.append(a+b_minus_a*s)
            elif da == 0:
                if len(intersectPoints) < 2:
                    intersectPoints.append(a)
            elif db == 0:
                if len(intersectPoints) < 2:
                    intersectPoints.append(b)
        #end for
        if len(intersectPoints) == 2:
            ls.vertices[0] = intersectPoints[0]
            ls.vertices[1] = intersectPoints[1]
            return 0

        return -2

class TriangleMesh:
    def __init__(self):
        self.bottomLeftVertex = Vector((999999, 999999, 999999))
        self.upperRightVertex = Vector((-999999, -999999, -999999))
        self.mesh = []

    def normalize(self):
        halfBbox = (self.upperRightVertex - self.bottomLeftVertex)/2
        start = self.bottomLeftVertex + halfBbox

        for i in range(0, len(self.mesh)):
            self.mesh[i].sub(start)

        self.bottomLeftVertex = halfBbox * -1
        self.upperRightVertex = halfBbox

    #t: Triangle
    def append(self, t):
        self.mesh.append(t)
        for i in range(0, 3):
            first = t.v[i].x
            second = self.bottomLeftVertex[0]

            if first < second:
                self.bottomLeftVertex[0] = t.v[i][0]
            if t.v[i][1] < self.bottomLeftVertex[1]:
                self.bottomLeftVertex[1] = t.v[i][1]
            if t.v[i][2] < self.bottomLeftVertex[2]:
                self.bottomLeftVertex[2] = t.v[i][2]
            if t.v[i][0] > self.upperRightVertex[0]:
                self.upperRightVertex[0] = t.v[i][0]
            if t.v[i][1] > self.upperRightVertex[1]:
                self.upperRightVertex[1] = t.v[i][1]
            if t.v[i][2] > self.upperRightVertex[2]:
                self.upperRightVertex[2] = t.v[i][2]

    def meshAABBSize(self):
        x = self.upperRightVertex[0] - self.bottomLeftVertex[0]
        y = self.upperRightVertex[1] - self.bottomLeftVertex[1]
        z = self.upperRightVertex[2] - self.bottomLeftVertex[2]
        return Vector((x,y,z))

class Polygon:
    def __init__(self, vertices):
        self.vertices = vertices