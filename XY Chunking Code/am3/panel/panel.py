import math
import bpy
import os
from bpy.props import *
from mathutils import Vector

import am3
from am3.slicer.slicer import Slicer
from am3.simulator.simulator_scene import SimulatorScene
from am3.util.simulatormath import SimulatorMath
from am3.machine.machine import Machine, MachineParameters
from am3.chunker.chunker import Chunker

bl_info = {
    'name': 'Simulator',
    'author': 'Jace McPherson',
    'location': 'View3D > UI panel > Add meshes',
    'category': '3D View'
    }


class SimulatorPanel(bpy.types.Panel):
    bl_label = "Control Simulation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "TOOLS"
    bl_category = "Simulation"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.label(text="Required Scene Objects")
        layout.prop_search(scene, "ObjectModel", scene, "objects", text="Model")
        layout.label(text="Machine Parameters")
        layout.prop(scene, "BuildDepth", text="Build Depth (cm)")
        layout.prop(scene, "SlopeAngle", text="Slope Angle (°)")
        layout.prop(scene, "PrintheadDepth", text="Printhead Depth")
        layout.prop(scene, "MachineSpeed", text="Robot Speed (cm/s)")
        layout.prop(scene, "SuggestMachines", text="Suggest Number of Robots")
        layout.prop(scene, "NumberOfMachines", text="Number of Robots")
        layout.label(text="Simulation Parameters")
        layout.prop(scene, "ModelScale", text="Model Scale")
        layout.prop(scene, "FramesPerSecond", text="Simulation FPS")
        layout.prop(scene, "SliceThickness", text="Slice Thickness")
        layout.prop(scene, "ExtrusionDiameter", text="Extrusion Diameter")
        layout.label(text="Actions")
        layout.operator("simulator.simulate", text="Full Simulation").args = "simulate"
        layout.operator("simulator.chunk", text="Chunk Only").args = "chunk"
        layout.operator("simulator.reset", text="Reset Scene").args = "reset"


class SimulationParameters:
    def __init__(self,
                 model_name="",
                 build_depth=2,
                 slope_angle=45,
                 printhead_depth=0.2,
                 machine_speed=0.1,
                 suggest_machines=False,
                 number_of_machines=2,
                 model_scale=1,
                 frames_per_second=2,
                 slice_thickness=0.04,
                 extrusion_diameter=0.02):
        self.model_name = model_name
        self.build_depth = build_depth
        self.slope_angle = slope_angle
        self.printhead_depth = printhead_depth
        self.machine_speed = machine_speed
        self.suggest_machines = suggest_machines
        self.number_of_machines = number_of_machines
        self.model_scale = model_scale
        self.frames_per_second = frames_per_second
        self.slice_thickness = slice_thickness
        self.extrusion_diameter = extrusion_diameter

def validate_parameters(panel):
    model_name = bpy.context.scene.ObjectModel
    build_depth = bpy.context.scene.BuildDepth
    slope_angle = bpy.context.scene.SlopeAngle
    printhead_depth = bpy.context.scene.PrintheadDepth
    machine_speed = bpy.context.scene.MachineSpeed
    suggest_machines = bpy.context.scene.SuggestMachines
    number_of_machines = bpy.context.scene.NumberOfMachines
    frames_per_second = bpy.context.scene.FramesPerSecond
    model_scale = bpy.context.scene.ModelScale
    slice_thickness = bpy.context.scene.SliceThickness
    extrusion_diameter = bpy.context.scene.ExtrusionDiameter

    # Check validity of the model object before we hand it to the chunker
    if model_name == "":
        panel.report(
            {'ERROR_INVALID_INPUT'},
            "You must select a Blender Mesh for your model.")
        return None

    if not model_name in bpy.data.objects:
        panel.report(
            {'ERROR_INVALID_INPUT'},
            "You have selected a Mesh that no longer exists.")
        return None

    model_object = bpy.data.objects[model_name]
    if model_object.type != 'MESH':
        panel.report(
            {'ERROR_INVALID_INPUT'},
            "You must select an Object that is a Mesh as your model.")
        return None

    params = SimulationParameters(
        model_name=model_name,
        build_depth=build_depth,
        slope_angle=slope_angle,
        printhead_depth=printhead_depth,
        machine_speed=machine_speed,
        suggest_machines=suggest_machines,
        number_of_machines=number_of_machines,
        model_scale=model_scale,
        frames_per_second=frames_per_second,
        slice_thickness=slice_thickness,
        extrusion_diameter=extrusion_diameter)
    return params

def import_robot_obj(filename="Robot.obj"):
    filename = os.path.join(bpy.path.abspath("//"), filename)
    bpy.ops.import_scene.obj(filepath=filename)
    return bpy.context.selected_objects

def machine_already_imported():
    return bpy.data.objects['Robot Body 0'] and bpy.data.objects['Robot Printhead 0']

def initialize_machines(params):
    if not machine_already_imported():
        print("Importing Robot...")
        imported_objects = import_robot_obj()
        imported_objects[0].name = 'Robot Printhead 0'
        imported_objects[1].name = 'Robot Body 0'


    printhead_slope = params.slope_angle * math.pi / 180
    machine_bounds = SimulatorMath.calculateBounds(bpy.data.objects['Robot Body 0'])
    machine_width = machine_bounds.x.max - machine_bounds.x.min
    if (machine_width == 0):
        print("0 width")

    model_bounds = SimulatorMath.calculateBounds(bpy.data.objects[params.model_name])
    model_width = model_bounds.x.max - model_bounds.x.min

    if params.suggest_machines:
        machines_per_side = math.ceil(model_width / machine_width / 2)
    else:
        machines_per_side = params.number_of_machines // 2

    machine_parameters = MachineParameters(
        build_depth=params.build_depth,
        printhead_slope=printhead_slope,
        printhead_depth=params.printhead_depth,
        machine_speed=params.machine_speed,
        machine_width=machine_width)

    if not params.suggest_machines and params.number_of_machines == 1:
        machine = Machine(machine_parameters=machine_parameters)
        machine.machine_model = bpy.data.objects['Robot Body 0']
        machine.printhead_model = bpy.data.objects['Robot Printhead 0']
        machine.set_hidden(False)
        machine.clear_animation_data()
        machine.set_location((0, -3, 0))
        return [machine]

    south_machines = []
    south_machines.append(Machine(machine_parameters=machine_parameters))
    south_machines[0].machine_model = bpy.data.objects['Robot Body 0']
    south_machines[0].printhead_model = bpy.data.objects['Robot Printhead 0']
    south_machines[0].set_hidden(False)
    south_machines[0].clear_animation_data()

    north_machines = []
    north_machines.append(south_machines[0].copy(flip_direction=True))

    machine_x = -1 * machines_per_side * machine_width
    south_machines[0].set_location((machine_x, -3, 0))
    north_machines[0].set_location((machine_x, 3, 0))
    machine_x += 2 * machine_width

    for i in range(1, machines_per_side):
        north_machines.append(north_machines[i - 1].copy())
        north_machines[i].set_location((machine_x, 3, 0))

        south_machines.append(south_machines[i - 1].copy())
        south_machines[i].set_location((machine_x, -3, 0))

        machine_x += 2 * machine_width

    all_machines = []

    for i in range(0, machines_per_side):
        south_machine = south_machines[i]
        south_machine.set_number(-1)

        north_machine = north_machines[i]
        north_machine.set_number(-1)

        all_machines.append(south_machine)
        all_machines.append(north_machine)

    for i in range(0, len(all_machines)):
        all_machines[i].set_number(i)
        print("Placed Model: {}".format(all_machines[i].machine_model.name))

    return all_machines

def apply_global_settings(params):
    am3.settings.set_settings(
        model_scale=params.model_scale,
        frames_per_second=params.frames_per_second,
        slice_thickness=params.slice_thickness,
        extrusion_diameter=params.extrusion_diameter)

class ChunkOperator(bpy.types.Operator):
    bl_idname = "simulator.chunk"
    bl_label = "Chunk"
    args = bpy.props.StringProperty()

    def execute(self, context):
        if self.args == "chunk":
            params = validate_parameters(self)
            if params is None:
                return {'FINISHED'}

            model_object = bpy.data.objects[params.model_name]

            machines = initialize_machines(params)
            apply_global_settings(params)

            Chunker.start_scaled(machines, model_object)

            print("Chunking complete")
            return {'FINISHED'}


class SimulateOperator(bpy.types.Operator):
    bl_idname = "simulator.simulate"
    bl_label = "Simulate"
    args = bpy.props.StringProperty()

    def execute(self, context):
        if self.args == "simulate":
            params = validate_parameters(self)
            if params is None: # parameters were invalid
                return {'FINISHED'}

            model_object = bpy.data.objects[params.model_name]

            machines = initialize_machines(params)
            apply_global_settings(params)

            Chunker.start_scaled(machines, model_object)

            print("Chunking complete")
            print("Slicing...")

            total_chunks = 0
            for machine in machines:
                total_chunks += len(machine.chunks)
                Slicer.slice(machine.chunks)
            # if our return value isn't a ChunkerResult, then Chunker threw an error.

            print("Generating commands...")
            for machine in machines:
                print("Generating commands for machine {}".format(machine.machine_number))
                machine.generate_commands()

            print("Animating...")
            SimulatorScene.simulate(machines)

        return{'FINISHED'}

class ResetOperator(bpy.types.Operator):
    bl_idname = "simulator.reset"
    bl_label = "Reset"
    args = bpy.props.StringProperty()

    def execute(self, context):
        params = validate_parameters(self)
        if params is None:
            return {'FINISHED'}

        print("Resetting scene")

        # loop through and delete all items that have 'extrusion' or 'chunk' in the name
        scene = bpy.context.scene

        for ob in scene.objects:
            if 'Robot Body' in ob.name and ob.name != 'Robot Body 0':
                ob.hide = False
                ob.select = True
            elif 'Robot Printhead' in ob.name and ob.name != 'Robot Printhead 0':
                ob.hide = False
                ob.select = True
            elif "Extrusion" in ob.name or "Chunk" in ob.name or ob.name == 'profile':
                ob.hide = False
                ob.select = True
            else:
                ob.select = False

        if scene.objects['Robot Body 0']:
            scene.objects['Robot Body 0'].location = Vector((0, 0, 0))
        if scene.objects['Robot Printhead 0']:
            scene.objects['Robot Printhead 0'].location = Vector((0, 0, 0))

        bpy.ops.object.delete()
        return {'FINISHED'}

def populate_coll(scene):
    bpy.app.handlers.scene_update_pre.remove(populate_coll)
    scene.coll.clear()
    for _, name, _ in enum_items:
        scene.coll.add().name = name


def register():
    bpy.utils.register_class(SimulatorPanel)
    bpy.utils.register_class(SimulateOperator)
    bpy.utils.register_class(ChunkOperator)
    bpy.utils.register_class(ResetOperator)

    bpy.types.Scene.ObjectModel = bpy.props.StringProperty()
    bpy.types.Scene.BuildDepth = bpy.props.FloatProperty(
        name="Build Depth", min=0, max=30, default=2.0)
    bpy.types.Scene.SlopeAngle = bpy.props.FloatProperty(
        name="Slope Angle °", min=0, max=90, default=30)
    bpy.types.Scene.PrintheadDepth = bpy.props.FloatProperty(
        name="Printhead Depth", min=0, max=1, default=0.2)
    bpy.types.Scene.MachineSpeed = bpy.props.FloatProperty(
        name="Machine Speed", min=0.1, max=100, default=0.1)
    bpy.types.Scene.SuggestMachines = bpy.props.BoolProperty(
        name="Suggest Number of Robots", default=True)
    bpy.types.Scene.NumberOfMachines = bpy.props.IntProperty(
        name="Number of Machines", min=1, max=32, default=4)
    bpy.types.Scene.ModelScale = bpy.props.FloatProperty(
        name="Model Scale", min=0.01, max=100, default=1.0)
    bpy.types.Scene.FramesPerSecond = bpy.props.FloatProperty(
        name="Frames Per Second", min=0.01, max=300, default=2)
    bpy.types.Scene.SliceThickness = bpy.props.FloatProperty(
        name="Slice Thickness", min=0.001, max=0.5, default=0.04)
    bpy.types.Scene.ExtrusionDiameter = bpy.props.FloatProperty(
        name="Extrusion Diameter", min=0.001, max=0.5, default=0.02)


def unregister():
    bpy.utils.unregister_class(SimulatorPanel)
    bpy.utils.unregister_class(SimulateOperator)
    bpy.utils.unregister_class(ChunkOperator)
    bpy.utils.unregister_class(ResetOperator)

    del bpy.types.Scene.ObjectModel
    del bpy.types.Scene.BuildDepth
