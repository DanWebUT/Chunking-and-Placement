"""
This module contains a single class, :class:`~am3.simulator.material.Material`, which represents
a piece of material in the scene.
"""

from typing import List, Tuple

import bpy
import am3

from mathutils import Vector


class Material:
    """
    A class to represent a piece of material material in the scene. Materials
    are represented by a path (defined by ordered points), a color, and a corresponding
    Blender object.
    """

    points = []
    """
    The list of points defining this placed material's path. Depends on the Blender-specific class,
    ``Vector``.
    
    :type: List[Vector]
    """

    material = None
    """
    A reference to the Blender material to use when rendering this path.
    
    :type: bpy.types.Material
    """

    model = None
    """
    A reference to the Blender object representing this Material.
    
    :type: bpy.types.Object
    """

    def __init__(self, points: List[Vector], color: Tuple[int, int, int] = (0, 0, 0)):
        """
        Initialize a Material given a list of points, any unique number, and an RGB color.
        This color should be of the form ``(r: int, g: int, b: int)``, where

        .. math::
            r, g, b \\in \\mathbb{Z} : r, g, b \\in [0, 255]

        :param points: a list of Vectors, defining the path of this Material
        :type points: List[Vector]
        :param number: a (hopefully) unique number for naming this extruded path
        :type number: int
        :param color: an RGB color value for this material (default ``(0, 0, 0)``)
        :type color: Tuple[int, int, int]
        """

        self.points = points
        self.material = self.get_material(color)
        self.model = self.create_model()

    def get_material(self, color: Tuple[int, int, int]) -> object:
        """
        Gets/generates the material object stored in the current ``.blend`` file. If it does not 
        exist, it is generated on the spot and stored for later retrieval.

        :param color: the color for this material
        :type color: Tuple[int, int, int]
        :return: the Blender material that was fetched/generated
        """

        material_name = "am3.simulator.material.Material.material {0} {1} {2}".format(
            color[0], color[1], color[2])
        if material_name in bpy.data.materials:  # fetch existing material
            return bpy.data.materials[material_name]
        
        # Material doesn't exist, go ahead and make a new one
        normalized_color = (color[0] / 256, color[1] / 256, color[2] / 256)
        material = bpy.data.materials.new(name=material_name)
        material.diffuse_color = normalized_color
        return material

    def create_model(self) -> object:
        """
        Generates the Blender object to follow the path stored in ``self.points`` and places
        it in the scene with the name ``Extrusion``.

        :return: the Blender object created to represent this Material.
        """

        # The curve is generated by referencing the `profile` object. If it doesn't exist yet,
        # go ahead and make it here.
        if not 'profile' in bpy.data.objects:
            bpy.ops.curve.primitive_bezier_circle_add()
            curve = bpy.context.selected_objects[0]
            diameter = am3.settings.EXTRUSION_DIAMETER  # extrustion diameter
            curve.scale = [diameter, diameter, diameter]
            curve.name = 'profile'
            curve.data.resolution_u = 2
            curve.data.render_resolution_u = 2

        pv = self.vertices_to_points(self.points)

        # Create a Blender curve
        scene = bpy.context.scene
        new_curve = bpy.data.curves.new('Extrusion Curve', type='CURVE')
        new_spline = new_curve.splines.new(type='POLY')
        new_spline.points.add(int(len(pv)*0.25 - 1))
        new_spline.points.foreach_set('co', pv)
        new_spline.use_endpoint_u = True
        if self.ends_meet():  # If the ends of the path are the same point, set `use_cyclic_u`
            new_spline.use_cyclic_u = True

        # Create a Blender object with new_curve
        new_curve.bevel_object = bpy.data.objects['profile']
        new_curve.dimensions = '3D'
        new_obj = bpy.data.objects.new('Extrusion', new_curve)  # object
        scene.objects.link(new_obj)  # place in active scene
        if new_obj.data.materials:
            new_obj.data.materials[0] = self.material
        else:
            new_obj.data.materials.append(self.material)
        return new_obj

    def ends_meet(self) -> bool:
        """
        Returns whether or not the ends of this path are the same point. This is important
        when drawing the curve, since weird artifacting will occur if the curve isn't made
        `cyclic`.

        :return: `True` if this curve is a closed loop, `False` otherwise
        :rtype: bool
        """

        if not self.points:
            return False
        return self.points[0] == self.points[-1]

    def vertices_to_points(self, vertices: List[Vector]) -> List[float]:
        """
        Returns a pure Python List representation of the ``vertices`` parameter. Blender has an
        interesting way of storing the points in a Bezier path in the scene (the Blender object
        used to visualize printed material). It requires a List of floats, where every 4 elements
        describe a single point in the path. The result should look like the following::

            v1 = Vector((1.0, 2.0, 3.0))
            v2 = Vector((4.0, 5.0, 6.0))
            points = vertices_to_points([v1, v2])
            print(points) # prints \"[1.0, 2.0, 3.0, 0, 4.0, 5.0, 6.0, 0]\"

        :param vertices: a list of Vectors to convert
        :type vertices: List[Vector]
        :return: a list of floats, effectively flat-mapping the input
        :rtype: List[float]
        """

        vertices_array = []
        for vertex in vertices:
            points = [vertex[0], vertex[1], vertex[2]]
            vertices_array += points
            vertices_array.append(0)
        return vertices_array

    def set_hidden(self, hidden: bool):
        """
        Hides or shows the Blender model representing this Material object, depending on the
        value of ``hidden``.

        :param hidden: should be ``True`` to hide the model, and ``False`` to show it.
        :type hidden: bool
        """

        if (self.model != None):
            self.model.hide = hidden
            self.model.hide_render = hidden

    def set_keyframe(self):
        """
        Inserts a keyframe at the current frame (as determined by bpy.context.scene) keeping track
        of this object's "hidden" value at the current frame
        """
        self.model.keyframe_insert("hide")
        self.model.keyframe_insert("hide_render")
