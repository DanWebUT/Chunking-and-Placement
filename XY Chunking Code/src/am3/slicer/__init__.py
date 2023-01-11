"""
Contains classes common to the ``slicer`` subpackage. The only class, so far, is the Slicer class.
"""

import math
import bmesh
import am3
import am3.settings

from typing import List, Dict
from collections import defaultdict
from am3.util import *
from am3.slicer.geometry import LineSegment, Triangle, TriangleMesh, Plane
from am3.model import Chunk, Slice, Path
from mathutils import Vector

from pprint import pprint

class Slicer:
    """
    This class is responsible for taking a Chunk (or list of Chunks) and slicing it into a \"filled
    geometry\". What this means is that the triangular mesh that the Chunk represents must be
    sliced into horizontally-stacked layers, then filled in with line segments such that, when
    material is placed along those paths, the resulting model would be structurally similar to the
    triangular mesh.\n

    It should be noted that this slicer is extremely simplistic. You are going to get almost no
    usability beyond a very simple solid fill pattern in models with no holes. It is also imperative 
    that the model doesn't have any errors in the construction (e.g., misplaced triangles, open
    geometry, etc.). The reason this slicer exists is purely for development purposes - it should
    NEVER be used on a serious print job.
    """

    @staticmethod
    def slice(chunks: List[Chunk]):
        """
        Begins the slicing process for a list of Chunks. This means that each Chunk is fed to the
        slicing algorithm by itself. The minimum ``Z`` position (i.e. the point lowest to the
        ground) is calculated and used as a way to shift every chunk such that the absolute lowest
        point amongst all the sliced chunks ends up at ``Z = 0``.\n

        Once slicing has completed, each Chunk object will contain the slicer result in
        ``chunk.slices``.

        :param chunks: the Chunks to slice
        :type chunks: List[Chunk]
        """

        min_z = 0
        for chunk in chunks:
            Slicer.slice_chunk(chunk, am3.settings.SLICE_THICKNESS)
            check_min_z = chunk.get_min_position()
            if check_min_z < min_z:
                min_z = check_min_z

        for chunk in chunks:
            chunk.shift_paths(-1 * min_z)

    @staticmethod
    def slice_chunk(chunk: Chunk, slice_size: float):
        """
        Slices a single Chunk. The results of the slicer are stored in ``chunk.slices``.

        :param chunk: the chunk to slice
        :type chunk: Chunk
        :param slice_size: the layer height
        :type slice_size: float
        """

        if chunk.is_empty():
            new_slice = Slice()
            new_path = Path()
            new_path.append(chunk.model)
            new_slice.append(new_path)
            chunk.append(new_slice)
            return

        tri_mesh = Slicer.triangulate_object(chunk.model)

        boundaries = [] # keeps track of the boundary LineSegments for each layer

        plane = Plane()
        plane.setNormal(Vector((0, 0, 1))) # upward-facing normal vector
        z_height = tri_mesh.meshAABBSize()[2]

        n_slices = int(1 + math.floor(z_height / slice_size))

        z0 = tri_mesh.bottomLeftVertex[2]

        # Iterate slicing planes from bottom to top
        for i in range(0, n_slices):
            intersections = []

            if i == n_slices - 1: # this is the last slice, so only shift upward by 0.5 * slice_size
                plane.setDistance(z0 + (i - 1) * slice_size + slice_size / 2)
            else:
                plane.setDistance(z0 + i * slice_size)

            for triangle in tri_mesh.mesh:
                intersection = LineSegment()
                if triangle.intersectPlane(plane, intersection) == 0:
                    intersection.simplify()
                    intersections.append(intersection)
                
            boundaries.append(intersections)

        slices = Slicer.fill_boundaries(boundaries)
        chunk.append(slices)
        chunk.set_hidden(True)

    @staticmethod
    def triangulate_object(obj: object) -> TriangleMesh:
        """
        Converts a Blender object to a TriangleMesh object.

        :param obj: a Blender object
        :type obj: bpy.types.Object
        :return: a the TriangleMesh form of ``obj``
        :rtype: TriangleMesh
        """

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

            tri_mesh.append(Triangle(v0, v1, v2))

        return tri_mesh

    @staticmethod
    def fill_boundaries(boundaries: List[List[LineSegment]]) -> List[Slice]:
        """
        Converts a list of boundaries (where a boundary is a list of LineSegments) to a List of
        Slices. The benefits of a Slice over a \"boundary\" is made up purely of distinct paths.
        Each path in a slice either defines its perimeter (which means its made up of points in
        guaranteed-correct order) or defines a fill path inside the perimeter.
        \n
        For each boundary (i.e. a single instance of a ``List[LineSegment]``), it is sorted into
        \"rings\". A ring is simply a closed loop of line segments. When intersecting a model, it
        is possible for the model to have multiple closed polygons intersecting a horizontal plane
        at the same height. When we want to print a fill pattern, we must fill every individual
        ring. For the most part, there will only be one ring per horizontal intersection.
        \n
        Once the rings are sorted (by calling :func:`~am3.slicer.Slicer.sort_boundary`), we visit
        each ring and fill it in. Every single path that is generated during this process is flat-
        mapped into a ``final_slice`` object. Once each ring has been visited, the ``final_slice``
        object is pushed onto the list of ``filled_slices``, which is then returned.

        :param boundaries: the boundaries to convert
        :type boundaries: List[List[LineSegment]]
        :return: a list of filled Slices, one Slice for each boundary
        :rtype: List[Slice]
        """

        filled_slices = []

        for boundary in boundaries:
            rings = Slicer.sort_boundary(boundary)
            final_slice = Slice()

            for ring in rings:
                final_slice.append(Slicer.fill_ring(ring))
                
            filled_slices.append(final_slice)

        return filled_slices

    @staticmethod
    def sort_boundary(boundary: List[LineSegment]) -> Slice:
        """
        Converts a boundary (i.e. a List of LineSegments) to a Slice. The line segments comprising
        the boundary are not considered to be in sorted order, and are thusly sorted. \"Sorted\",
        in this case, means the LineSegments are place in the correct order and orientation such
        that all line segments that share endpoints are placed adjacent to each other.
        \n
        There are possibly more than one closed loops that can be formed from the boundary, so it
        is possible for the resulting Slice to contains multiple ``paths`` after executing this
        method.
        
        :param boundary: the boundary to sort and convert to a Slice.
        :type boundary: List[LineSegment]
        :return: the sorted Slice
        :rtype: Slice
        """

        if len(boundary) == 0: # if the boundary is empty, so is the Slice
            return Slice()

        line_segments = list(boundary) # duplicate the input boundary
        endpoint_to_line_segment_dict = defaultdict(lambda: [])

        rings = []
        num_segments = len(line_segments)

        # This loop builds the ``endpoint_to_line_segment_dict`` so that it is indexed such that
        # a Vector key will return a List of all LineSegments that have endpoints at that Vector.
        for segment in line_segments:
            segment.simplify()

            segment.getFirst().freeze() # this makes the Vector hashable
            segment.getSecond().freeze() # this makes the Vector hashable

            first = segment.getFirst()
            second = segment.getSecond()

            endpoint_to_line_segment_dict[first].append(segment)
            endpoint_to_line_segment_dict[second].append(segment)

        # Creates the rings using the recursive function ``Slicer.build_chain``. Continues until
        # there are no more segments to place into a ring.
        counter = 0
        while endpoint_to_line_segment_dict.items():
            arbitrary_value = next(iter(endpoint_to_line_segment_dict.values()))

            ring = Slicer.build_ring(arbitrary_value[0], endpoint_to_line_segment_dict)
            rings.append(ring)

            counter += 1
            if counter > num_segments:
                print("Something went wrong... Breaking loop")
                break

        boundary_slice = Slice()

        # Converts each ring (which is a List of LineSegments) to a path (which is a list of Vertices).
        for ring in rings:
            path = Path()

            for i in range(len(ring)):
                path.append(ring[i].getFirst())
                if i == len(ring) - 1:
                    path.append(ring[i].getSecond())

            boundary_slice.append(path)

        return boundary_slice

    @staticmethod
    def build_ring(segment: LineSegment, endpoints_to_line_segments_dict: Dict[Vector, LineSegment]) -> List[LineSegment]:
        """
        Recursively builds the ring that contains ``segment``. The logic is that, using
        ``endpoints_to_line_segments_dict``, the current segment should be able to be constructed
        outward from both of its endpoints. That means that we take one of its vertices and check
        the Dict for any LineSegments that share that vertex. If one is found, it is considered
        part of ``segments`` ring, and is placed in an array adjacent to ``segment``. As new
        segments are found, they are removed from the Dict so that they don't get used again.
        \n
        Note that this is destructive to ``endpoints_to_line_segments_dict``. As rings are built,
        the LineSegments are copied into the result and removed from the Dict.

        :param segment: the segment to be used as a starting point for building this ring
        :type segment: LineSegment
        :param endpoints_to_line_segments_dict: the Dict used to index every LineSegment by its
            endpoints
        :type endpoints_to_line_segments_dict: Dict[Vector, LineSegment]
        :return: the ring that contains ``segment``
        :rtype: List[LineSegment]
        """

        left_list = endpoints_to_line_segments_dict[segment.getFirst()]
        right_list = endpoints_to_line_segments_dict[segment.getSecond()]

        left_list.remove(segment)
        right_list.remove(segment)

        array = [segment]

        if left_list:
            left_segment = left_list[0]
            if left_segment.getFirst() == segment.getFirst():
                left_segment.flip()
            array = Slicer.build_ring(left_segment, endpoints_to_line_segments_dict) + array

        if right_list:
            right_segment = right_list[0]
            if right_segment.getSecond() == segment.getSecond():
                right_segment.flip()
            array = array + Slicer.build_ring(right_list[0], endpoints_to_line_segments_dict)

        if not left_list and segment.getFirst() in endpoints_to_line_segments_dict:
            del endpoints_to_line_segments_dict[segment.getFirst()]

        if not right_list and segment.getSecond() in endpoints_to_line_segments_dict:
            del endpoints_to_line_segments_dict[segment.getSecond()]

        return array

    @staticmethod
    def fill_ring(ring: Path) -> List[Path]:
        """
        Takes a ring as a parameter and fills it, returning all the paths (including the original
        ring) needed to print that ring with a solid fill pattern.

        :param ring: the ring to fill
        :type ring: Path
        :return: the filled ring, as a list of Paths
        :rtype: List[Path]
        """

        if not ring or not ring.vertices: # empty ring, empty result
            return []

        paths = []
        paths.append(ring)
        filled_paths = Slicer.generate_infill(ring)
        for filled_path in filled_paths:
            paths.append(filled_path)

        return paths

    @staticmethod
    def generate_infill(in_outline: Path, inset: float = am3.settings.SLICE_THICKNESS * 0.75) -> List[Path]:
        """
        Generates the solid infill Paths for ``in_outline``. 

        :param in_outline: the outline to fill with solid infill
        :type in_outline: Path
        :param inset: the amount to inset the fill from the edge
        :type inset: float
        :return: the Paths that fill ``in_outline``
        :rtype: List[Path]
        """

        intersect_lines = SimulatorMath.get_intersection_lines_for_bounds(in_outline.vertices, inset)

        last_size = 0
        intersection_groups = []

        paths = []

        for x_value in intersect_lines:
            intersections = SimulatorMath.get_intersections_for_line_segments(x_value, in_outline.vertices)

            new_size = len(intersections)

            if new_size != last_size and last_size != 0:
                for intersection_group in intersection_groups:
                    path = Slicer.build_path(intersection_group, inset)
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
    def build_path(segments: List[LineSegment], inset: float):
        """
        Builds the zig-zagged solid infill path from a list of LineSegments

        :param segments: the LineSegments making the paralell infill lines
        :type segments: List[LineSegment]
        :param inset: the amount to inset the LineSegments from the edge
        :type inset: float
        """
        path = Path()

        for i in range(0, len(segments)):
            if (i % 2 != 0):
                segments[i].flip()

            segments[i].shrink(inset)
            path.append(segments[i].getFirst())
            path.append(segments[i].getSecond())

        return path
