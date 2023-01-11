import bpy
import am3
from am3.settings import *

class Printed:

    def __init__(self, points, number, color=(0, 0, 0)):
        #Blender object 'Mesh'

        self.points = points
        self.number = number
        # A machine object
        self.sourceMachine = None

        # Some blender material
        self.material = self.getMaterial(color)

        self.model = self.createModel()
        pass

    def getMaterial(self, color):
        material_name = "New Weird Material {0} {1} {2}".format(color[0], color[1], color[2])
        if material_name in bpy.data.materials:
            return bpy.data.materials[material_name]
        else:
            # make new material
            normalized_color = (color[0] / 256, color[1] / 256, color[2] / 256)

            material = bpy.data.materials.new(name=material_name)
            material.diffuse_color = normalized_color
            return material

    def createModel(self):
        if not ('profile' in bpy.data.objects):
            bpy.ops.curve.primitive_bezier_circle_add()
            curve = bpy.context.selected_objects[0]
            diameter = am3.settings.EXTRUSION_DIAMETER # extrustion diameter
            curve.scale = [diameter, diameter, diameter]
            curve.name = 'profile'
            curve.data.resolution_u = 2
            curve.data.render_resolution_u = 2

        name = "Extrusion {}".format(self.number)
        pv = self.verticesToPoints(self.points)
        # create curve
        scene = bpy.context.scene
        new_curve = bpy.data.curves.new(name, type='CURVE')
        new_spline = new_curve.splines.new(type='POLY')
        new_spline.points.add(int(len(pv)*0.25 - 1))
        new_spline.points.foreach_set('co', pv)
        new_spline.use_endpoint_u = True
        if self.endsMeet():
            new_spline.use_cyclic_u = True


        # create object with new_curve
        new_curve.bevel_object = bpy.data.objects['profile']
        new_curve.dimensions = '3D'
        new_obj = bpy.data.objects.new(name, new_curve) # object
        scene.objects.link(new_obj) # place in active scene
        if new_obj.data.materials:
            new_obj.data.materials[0] = self.material
        else:
            new_obj.data.materials.append(self.material)
        return new_obj

    def endsMeet(self):
        if not self.points:
            return False
        return self.points[0] == self.points[-1]

    def verticesToPoints(self, vertices):
        vertices_array = []
        for vertex in vertices:
            points = [vertex[0], vertex[1], vertex[2]]
            vertices_array += points
            vertices_array.append(0)
        return vertices_array

    def set_hidden(self, hidden):
        if (self.model != None):
            self.model.hide = hidden
