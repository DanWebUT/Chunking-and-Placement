import am3
import sys

from am3.settings import *
from am3.util.simulatormath import *
from am3.slicer.geometry import LineSegment, Triangle, TriangleMesh, Polygon, Plane
from am3.model import Chunk, Slice, Path
from mathutils import Vector



sys.setrecursionlimit(5000)

class Slicer:

    @staticmethod
    def slice(chunks):
        min_z = 0
        for chunk in chunks:
            Slicer.slice_chunk(chunk, am3.settings.SLICE_THICKNESS)
            check_min_z = chunk.get_min_position()
            if check_min_z < min_z:
                min_z = check_min_z

        for chunk in chunks:
            chunk.shift_paths(-1 * min_z)

    # input: obj: BlenderObject, sliceSize: float
    # @return [[LineSegment]]
    @staticmethod
    def slice_chunk(chunk, slice_size):
        if chunk.is_empty():
            new_slice = Slice()
            new_path = Path()
            new_path.append(chunk.model)
            new_slice.append(new_path)
            chunk.append(new_slice)
            return

        tri_mesh = Slicer.triangulate_object(chunk.model)

        boundaries = []

        plane = Plane()
        plane.setNormal(Vector((0, 0, 1)))
        aabb = tri_mesh.meshAABBSize()

        n_slices = 1 + math.floor(aabb[2] / slice_size)

        m = tri_mesh.mesh
        z0 = tri_mesh.bottomLeftVertex[2]

        for i in range(0, n_slices):
            linesegs = []
            if i == n_slices - 1:
                plane.setDistance(z0 + (i - 1) * slice_size + slice_size / 2)
            else:
                plane.setDistance(z0 + i*slice_size)
            for t in range(0, len(m)):
                triangle = m[t]
                ls = LineSegment()
                if triangle.intersectPlane(plane, ls) == 0:
                    ls.simplify()
                    linesegs.append(ls)

            boundaries.append(linesegs)

        slices = Slicer.fill_boundaries(boundaries)
        chunk.append(slices)
        chunk.set_hidden(True)

    @staticmethod
    def triangulate_object(obj):
        obj_mesh = obj.data

        if obj_mesh == None:
            bm = bmesh.new()
            bm.from_mesh(obj.data)

            bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method=0, ngon_method=0)

            # Finish up, write the bmesh back to the mesh
            bm.to_mesh(obj.data)
            bm.free()

            obj_mesh = obj.data

        tri_mesh = TriangleMesh()

        for polygon in obj_mesh.polygons:
            vertices = polygon.vertices
            v0t = obj_mesh.vertices[vertices[0]].co
            v1t = obj_mesh.vertices[vertices[1]].co
            v2t = obj_mesh.vertices[vertices[2]].co

            v0 = Vector((v0t[0], v0t[1], v0t[2]))
            v1 = Vector((v1t[0], v1t[1], v1t[2]))
            v2 = Vector((v2t[0], v2t[1], v2t[2]))

            normal = Vector((polygon.normal[0], polygon.normal[1], polygon.normal[2]))

            tri_mesh.append(Triangle(normal, v0, v1, v2))

        return tri_mesh


    # boundaries: [[LineSegment]]
    # @return: [Slice]
    @staticmethod
    def fill_boundaries(boundaries):
        filled_slices = []

        for boundary in boundaries: # slice: [LineSegment]
            rings = Slicer.sort_boundary(boundary) # rings: Slice
            final_slice = Slice()
            for ring in rings: # ring: [Vector]
                final_slice.append(Slicer.fill_ring(ring))
            filled_slices.append(final_slice)

        return filled_slices

    # boundary: [LineSegment]
    # @return: Slice; Unfilled slice rings
    @staticmethod
    def sort_boundary(boundary):
        if len(boundary) == 0:
            return [[]]

        slice = list(boundary)
        map = dict()
        groups = []
        segments = len(slice)

        for segment in slice:
            segment.simplify()
            segment.getFirst().freeze()
            segment.getSecond().freeze()

            if segment.getFirst() in map:
                map[segment.getFirst()].append(segment)
            else:
                map[segment.getFirst()] = [segment]

            if segment.getSecond() in map:
                map[segment.getSecond()].append(segment)
            else:
                map[segment.getSecond()] = [segment]

        i = 0

        while map.items():
            arbitrary_value = next(iter(map.values())) # this is a [LineSegment]

            group = Slicer.build_chain(arbitrary_value[0], map)
            groups.append(group)

            i += 1
            if i > segments:
                print("Something went wrong... Breaking loop")
                break

        slice = Slice()

        for group in groups:
            path = Path()
            for i in range(len(group)):
                path.append(group[i].getFirst())
                if i == len(group) - 1:
                    path.append(group[i].getSecond())

            slice.append(path)

        return slice

    # @return [LineSegment]
    # @recursive
    # Destructive for "map" - segments are removed as they are accessed
    @staticmethod
    def build_chain(segment, map):
        left_list = map[segment.getFirst()]
        right_list = map[segment.getSecond()]

        left_list.remove(segment)
        right_list.remove(segment)

        array = [segment]

        if left_list:
            left_segment = left_list[0]
            if left_segment.getFirst() == segment.getFirst():
                left_segment.flip()
            array = Slicer.build_chain(left_segment, map) + array

        if right_list:
            right_segment = right_list[0]
            if right_segment.getSecond() == segment.getSecond():
                right_segment.flip()
            array = array + Slicer.build_chain(right_list[0], map)

        if not left_list and segment.getFirst() in map:
            del map[segment.getFirst()]

        if not right_list and segment.getSecond() in map:
            del map[segment.getSecond()]

        return array

    # ring: Path, a non-self-intersecting path defining a polygon
    # @return: [Path], all the paths needed to create and fill this polygon
    @staticmethod
    def fill_ring(ring):
        if not ring:
            return []

        paths = [] # paths: [Path]
        paths.append(ring)
        filled_paths = Slicer.generate_infill(ring)
        for filled_path in filled_paths:
            paths.append(filled_path)

        return paths

    @staticmethod
    def generate_infill(in_outline, inset=am3.settings.SLICE_THICKNESS * 0.75):
        intersect_lines = SimulatorMath.getIntersectionLinesForBounds(in_outline.vectors, inset)

        last_size = 0
        intersection_groups = [] # [[Segment]]

        paths = []

        for x_value in intersect_lines:
            intersections = SimulatorMath.getIntersectionsForLineSegments(x_value, in_outline.vectors)

            new_size = len(intersections)

            if new_size != last_size and last_size != 0:
                for intersection_group in intersection_groups:
                    path = Slicer.build_path(intersection_group, inset) # intersectionGroup := [LineSegment]
                    paths.append(path)

                del intersection_groups[:]

            for i in range(0, new_size // 2):
                while i >= len(intersection_groups):
                    intersection_groups.append([])
                intersection_groups[i].append(LineSegment(intersections[i*2], intersections[i*2+1]))

            last_size = new_size

        if intersection_groups:
            for intersection_group in intersection_groups:
                path = Slicer.build_path(intersection_group, inset) # intersectionGroup := [LineSegment]
                paths.append(path)

        return paths

    @staticmethod
    def build_path(segments, inset):
        path = Path()

        for i in range(0, len(segments)):
            if (i % 2 != 0):
                segments[i].flip()

            segments[i].shrink(inset)
            path.append(segments[i].getFirst())
            path.append(segments[i].getSecond())

        return path
