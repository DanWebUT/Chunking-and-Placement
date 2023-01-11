"""
Contains classes related to the UI panel created in Blender for controlling simulations.
"""
import os
import math
from typing import List

import bpy
from mathutils import Vector

import am3
from am3.slicer import Slicer
from am3.simulator.scene import SimulatorScene, Initializer, JsonLoader
from am3.util import SimulatorMath
from am3.robot import Robot, RobotParameters
from am3.chunker import Chunker

bl_info = {
    'name': 'Simulator',
    'author': 'Jace McPherson',
    'location': 'View3D > UI panel > Add meshes',
    'category': '3D View'
}

class JsonFileOperator(bpy.types.Operator):
    bl_idname = "simulator.json_file"
    bl_label = "JSON File"
    filename_ext = ".json"
    filepath = bpy.props.StringProperty(subtype="FILE_PATH", default="*.json")

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        params = validate_parameters(self)
        if params is None:  # parameters were invalid
            return {'FINISHED'}
            
        apply_global_settings(params)
        model_object = bpy.data.objects[params.model_name]
        model_object.hide = True
        model_object.hide_render = True

        material_color = bpy.context.scene.MaterialColor
        print(material_color)
        material_color = (int(material_color[0] * 255), int(material_color[1] * 255), int(material_color[2] * 255))
        print(material_color)
        
        if not JsonLoader.load_from_simulation_file(self.filepath, material_color=material_color):
            self.report(
                {'ERROR_INVALID_INPUT'},
                "This is either NOT a JSON file, or it is in the invalid format.")

        return {'FINISHED'}


class SimulatorPanel(bpy.types.Panel):
    """
    Main panel that contains all the necessary UI widgets to set up a Cooperative 3D print within
    Blender. Within this class, various methods set up the Blender widgets and define the main
    functions triggered by clicking the buttons. The three main actions are "Simulate", "Chunk",
    and "Reset Scene". The specifics of each of these functions is covered in detail in the
    relevant documented functions.
    """

    bl_label = "Control Simulation"
    """
    Defines the title of this panel
    """

    bl_space_type = "VIEW_3D"
    """
    Defines where the panel should appear (i.e. this panel will only appear in the 3D model
    viewer
    """

    bl_region_type = "TOOLS"
    """
    Defines where in the viewer the model should appear (i.e. the specific pullout bar in the 3D
    model viewer
    """

    bl_category = "Simulation"
    """Defines the category (i.e. the tool tab) of this panel"""

    def draw(self, context):
        """
        The draw function merely tells Blender what widgets to place in the "Simulation" panel.
        The purpose of each widget is defined in the body of this method.

        :param context: The Blender context in which to draw this panel

        """

        layout = self.layout
        scene = context.scene
        layout.label(text="Required Scene Objects")

        # Object Selector: so the user can pick which Blender object should be printed
        layout.prop_search(scene, "ObjectModel", scene,
                           "objects", text="Model")

        layout.label(text="Robot Parameters")

        # Number Entry: specifies the "build depth" robot parameter
        layout.prop(scene, "BuildDepth", text="Build Depth (mm)")
        # Number Entry: specifies the "slope angle" robot parameter
        layout.prop(scene, "SlopeAngle", text="Slope Angle (°)")
        # Number Entry: specifies the "printhead depth" robot parameter
        layout.prop(scene, "PrintheadDepth", text="Printhead Depth (mm)")
        # Number Entry: specifies the "robot speed" robot parameter (how fast the robot moves)
        layout.prop(scene, "RobotSpeed", text="Robot Speed (mm/s)")
        # Boolean Entry: specifies whether or not to use the algorithm-optimized number of robots
        layout.prop(scene, "SuggestRobots", text="Suggest Number of Robots")
        # Number Entry: specifies the number of robots to use. Probably shouldn't use this,
        # but it is useful for debug purposes
        layout.prop(scene, "NumberOfRobots", text="Number of Robots")
        layout.label(text="Simulation Parameters")
        # Number Entry: specifies the scale to be used. Multiple the model size by this number for
        # the expected print size
        layout.prop(scene, "ModelScale", text="Model Scale")
        # Number Entry: the FPS of the output simulation. The smaller this is, the shorter the
        # simulation will be.
        layout.prop(scene, "FramesPerSecond", text="Simulation FPS")
        # Number Entry: tells the slicer how thick to slice
        layout.prop(scene, "SliceThickness", text="Slice Thickness (mm)")
        # Number Entry: determines how thick the output extruded material should be (i.e. the
        # diameter of the "tubes")
        layout.prop(scene, "ExtrusionDiameter", text="Extrusion Diameter (mm)")
        # Boolean Entry: Whether or not to export the simulation in JSON format
        layout.prop(scene, "WriteSimulationFile",
                    text="Write to Simulation File (JSON)")
        layout.label(text="Actions")
        # BUTTON: Run a full simulation (i.e. chunk, slice, simulate)
        layout.operator("simulator.simulate",
                        text="Full Simulation").args = "simulate"
        # BUTTON: Just chunk the model and show the produced chunks
        layout.operator("simulator.chunk", text="Chunk Only").args = "chunk"
        # BUTTON: Attempt to reset the simulation scene to contain a single robot and the target
        # model only
        layout.operator("simulator.reset", text="Reset Scene").args = "reset"

        layout.label(text="Other Actions")
        layout.prop(scene, "MaterialColor", text='Material Color')

        # File Selector: to pick the JSON file to load
        layout.operator("simulator.json_file", text="Load Simulation JSON File", icon="FILE_FOLDER")


class SimulationParameters:
    """
    A plain object class for containing various parameters to the simulation. These parameters
    are pulled directly from the values set in the Blender UI panel defined in
    :class:`~am3.panel.SimulatorPanel`.
    """

    def __init__(self,
                 model_name: str = '',
                 build_depth: float = 50,
                 slope_angle: float = 45,
                 printhead_depth: float = 5,
                 robot_speed: float = 10,
                 suggest_robots: bool = False,
                 number_of_robots: int = 2,
                 model_scale: float = 1,
                 frames_per_second: float = 2,
                 slice_thickness: float = 0.5,
                 extrusion_diameter: float = 0.25,
                 write_simulation_file: bool = False):
        """
        Initializes a new SimulationParameters object. The parameters and their default values are
        documented here.

        :param model_name: the name of the Blender object to simulate printing (default ``''``)
        :type model_name: str
        :param build_depth: the robot's maximum print distance, in mm (default ``50``)
        :type build_depth: float
        :param slope_angle: the robot's nozzle angle, in degrees (default ``45``)
        :type slope_angle: float
        :param printhead_depth: the length of the printhead past the nozzle, in mm (default ``5``)
        :type printhead_depth: float
        :param robot_speed: the speed at which robots will move in mm/s (default ``10``)
        :type robot_speed: float
        :param suggest_robots: whether or not the software should override the user-defined
            number of robots (default ``False``)
        :type suggest_robots: bool
        :param number_of_robots: the number of robots to use for the simulation. This should
            almost always be even (or 1). (default ``2``)
        :type number_of_robots: int
        :param model_scale: the scaling factor for the final output. If you want the print to end
            up smaller than the input object, the scale factor should be less than 1
            (default ``1.0``)
        :type model_scale: float
        :param frames_per_second: the number of frames that should be used for every second of
            real-world movement. For most real-time videos, this value should be 30.
            (default ``2.0``)
        :type frames_per_second: float
        :param slice_thickness: the thickness of each slice in the slicer, in mm. (default ``0.5``)
        :type slice_thickness: float
        :param extrusion_diameter: the diameter of the extruded material, in mm. This is purely a
        cosmetic parameter. (default ``0.25``)
        :type extrusion_diameter: float
        :param write_simulation_file: whether or not to write the full simulation to a JSON file
            for use in other simulators. (default ``False``)
        :type write_simulation_file: bool
        """

        self.model_name = model_name
        self.build_depth = build_depth
        self.slope_angle = slope_angle
        self.printhead_depth = printhead_depth
        self.robot_speed = robot_speed
        self.suggest_robots = suggest_robots
        self.number_of_robots = number_of_robots
        self.model_scale = model_scale
        self.frames_per_second = frames_per_second
        self.slice_thickness = slice_thickness
        self.extrusion_diameter = extrusion_diameter
        self.write_simulation_file = write_simulation_file


def validate_parameters(panel) -> SimulationParameters:
    """
    Checks the parameters input by the user in the Blender UI panel. Any errors that are found
    trigger a call to ``panel.report(...)``, which pulls up a user-friendly error popup with the
    specific problem identified.\n
    Assuming no errors found, this method returns the validated parameters object.

    :param panel: the Blender panel to display errors on, if they occur.
    :return: the validated SimulationParameters
    """

    model_name = bpy.context.scene.ObjectModel
    build_depth = bpy.context.scene.BuildDepth
    slope_angle = bpy.context.scene.SlopeAngle
    printhead_depth = bpy.context.scene.PrintheadDepth
    robot_speed = bpy.context.scene.RobotSpeed
    suggest_robots = bpy.context.scene.SuggestRobots
    number_of_robots = bpy.context.scene.NumberOfRobots
    frames_per_second = bpy.context.scene.FramesPerSecond
    model_scale = bpy.context.scene.ModelScale
    slice_thickness = bpy.context.scene.SliceThickness
    extrusion_diameter = bpy.context.scene.ExtrusionDiameter
    write_simulation_file = bpy.context.scene.WriteSimulationFile

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

    if number_of_robots != 1 and number_of_robots % 2 != 0:
        panel.report(
            {'ERROR_INVALID_INPUT'},
            "You must choose a valid number of robots (just one, or an even number)")
        return None

    params = SimulationParameters(
        model_name=model_name,
        build_depth=build_depth,
        slope_angle=slope_angle,
        printhead_depth=printhead_depth,
        robot_speed=robot_speed,
        suggest_robots=suggest_robots,
        number_of_robots=number_of_robots,
        model_scale=model_scale,
        frames_per_second=frames_per_second,
        slice_thickness=slice_thickness,
        extrusion_diameter=extrusion_diameter,
        write_simulation_file=write_simulation_file)
    return params


def initialize_robots(params: SimulationParameters) -> List[Robot]:
    """
    Initializes the robot objects in the Blender 3D scene.\n
    The setup is highly dependent upon the parameters set by the user in the panel settings.
    For example, if the user has checked ``Suggest Robots`` to be true, then the number of
    robots to place in the scene is dependent upon the width of the model according to the
    following equation:\n

    .. math::
        \\text{robotsPerSide} = \\left\\lceil \\dfrac{\\text{modelWidth}}{2 \\cdot
        \\text{robotWidth}} \\right\\rceil

    In addition to creating the correct number of robots, each robot is given the user-defined
    parameters set in the panel (i.e. build depth, slope angle, etc.). Each robot is arranged
    such that there are two distinct rows of robots, a \"north\" row (in the ``+Y`` direction) and a
    \"south\" row (in the ``-Y`` direction). The arrangement for ``n`` robots (where ``n`` is even)
    is as follows:

    .. math::
        \\begin{array}{cccc}
            (1)  & (3)  & ... & (n-1) \\\\ \\hline
            & \\rlap{\\text{model}} \\\\ \\hline
            (0)  & (2)  & ... & (n-2)
        \\end{array}

    :param params: validated SimulationParameters
    :type params: SimulationParameters
    :return: a List of initialized Robots
    :rtype: List[Robot]
    """

    if not Initializer.is_robot_already_imported():
        print("Importing Robot...")
        Initializer.import_robot_obj()

    printhead_slope = params.slope_angle * math.pi / 180
    robot_bounds = SimulatorMath.calculate_bounds(
        bpy.data.objects['Robot Body 0'])
    robot_width = robot_bounds.x.max - robot_bounds.x.min

    model_bounds = SimulatorMath.calculate_bounds(
        bpy.data.objects[params.model_name])
    model_width = model_bounds.x.max - model_bounds.x.min

    if params.suggest_robots:
        robots_per_side = math.ceil(model_width / robot_width / 2)
    else:
        robots_per_side = params.number_of_robots // 2

    robot_parameters = RobotParameters(
        build_depth=params.build_depth,
        printhead_slope=printhead_slope,
        printhead_depth=params.printhead_depth,
        speed=params.robot_speed,
        width=robot_width)

    # Edge case for when the user wants to do a single robot print
    if not params.suggest_robots and params.number_of_robots == 1:
        robot = Robot(parameters=robot_parameters)
        robot.body_model = bpy.data.objects['Robot Body 0']
        robot.printhead_model = bpy.data.objects['Robot Printhead 0']
        robot.set_hidden(False)
        robot.clear_animation_data()
        robot.set_location((0, -30, 0))
        return [robot]

    robot_count = robots_per_side * 2
    robots = Initializer.import_robots(robot_count, robot_parameters)

    robot_x = -1 * robots_per_side * robot_width
    for i, robot in enumerate(robots):
        if i % 2 == 0:
            # robot is even, therefore on the south side, facing north
            robot.set_location((robot_x, -30, 0))
        else:
            # robot is odd, therefore on the north side, facing south
            robot.set_location((robot_x, 30, 0))
            robot.flip()
            robot_x += 2 * robot_width

    return robots


def apply_global_settings(params):
    """
    Some user-defined parameters needs to be globally available to the module. This function
    sets the appropriate settings -- namely, \"model scale\", \"frames per second\",
    \"slice thickness\", and \"extrusion diamter\".

    :param params: validated SimulationParameters
    :type params: SimulationParameters
    """

    am3.settings.set_settings(
        model_scale=params.model_scale,
        frames_per_second=params.frames_per_second,
        slice_thickness=params.slice_thickness,
        extrusion_diameter=params.extrusion_diameter)


class SimulateOperator(bpy.types.Operator):
    """Defines the Blender Operator for the \"Simulate\" button. Each button needs its own operator.
    This one goes through the entire simulation process, from an unchunked model to a full
    simulation in the Blender 3D viewer.
    """

    bl_idname = "simulator.simulate"
    """A Blender-specific name for this operator"""

    bl_label = "Simulate"
    """The name/label for this operator"""

    args = bpy.props.StringProperty()
    """The data type for the arguments passed to this function. The only argument passed is
    a single StringProperty, ``\"simulate\"``."""

    def execute(self, _):
        """This is where the actual Operator code goes. Here, we do the following:

            * validate parameters,
            * initialize robots
            * apply global settings,
            * chunk the model using :func:`~am3.chunker.Chunker.start_scaled`,
            * slice the module using :func:`~am3.slicer.Slicer.slice`,
            * call :func:`~am3.robot.Robot.generate_visualization` for each Robot, and finally
            * call :func:`~am3.simulator.scene.SimulatorScene.simulate`
        """

        if self.args == "simulate":
            params = validate_parameters(self)
            if params is None:  # parameters were invalid
                return {'FINISHED'}

            model_object = bpy.data.objects[params.model_name]

            robots = initialize_robots(params)
            apply_global_settings(params)

            Chunker.start_scaled(robots, model_object)

            print("Chunking complete")
            print("Slicing...")

            for robot in robots:
                Slicer.slice(robot.chunks)
            # if our return value isn't a ChunkerResult, then Chunker threw an error.

            print("Generating commands...")
            for robot in robots:
                print("Generating commands for robot {}".format(
                    robot.get_number()))
                robot.generate_visualization()

            print("Animating...")
            SimulatorScene.simulate(
                robots, write_simulation_file=params.write_simulation_file)

        return{'FINISHED'}


class ChunkOperator(bpy.types.Operator):
    """Defines the Blender Operator for the \"Chunk\" button. Each button needs its own operator.
    This one merely initializes the scene, calls the :func:`~am3.chunker.Chunker.start_scaled`
    function, then returns.
    """

    bl_idname = "simulator.chunk"
    """A Blender-specific name for this operator"""

    bl_label = "Chunk"
    """The name/label for this operator"""

    args = bpy.props.StringProperty()
    """The data type for the arguments passed to this function. The only argument passed is
    a single StringProperty, ``\"chunk\"``."""

    def execute(self, _):
        """This is where the actual Operator code goes. Here, we do the following:

            * validate parameters,
            * initialize robots
            * apply global settings, and
            * chunk the model using :func:`~am3.chunker.Chunker.start_scaled`
        """

        if self.args == "chunk":
            params = validate_parameters(self)
            if params is None:
                return {'FINISHED'}

            model_object = bpy.data.objects[params.model_name]

            robots = initialize_robots(params)
            apply_global_settings(params)

            Chunker.start_scaled(robots, model_object)

            print("Chunking complete")
            return {'FINISHED'}


class ResetOperator(bpy.types.Operator):
    """Defines the Blender Operator for the \"Reset\" button. Each button needs its own operator.
    This one merely removes all robots except robot ``0``, removes all extrusion objects,
    removes all Chunk objects, sets the first robot's position to ``(0, 0, 0)`` then returns.
    """

    bl_idname = "simulator.reset"
    """A Blender-specific name for this operator"""

    bl_label = "Reset"
    """The name/label for this operator"""

    args = bpy.props.StringProperty()
    """The data type for the arguments passed to this function. The only argument passed is
    a single StringProperty, ``\"reset\"``."""

    def execute(self, _):
        """This is where the Operator code goes. This method does the following:

            * validates the parameters,
            * iterates over every Blender object,
            * for each object, if it should be removed, select it. Otherwise, deselect it
            * objects with \"extrusion\" or \"chunk\" should be removed, as well as any
              robot parts that don't belong to robot ``0``.
        """

        params = validate_parameters(self)
        if params is None:
            return {'FINISHED'}

        print("Resetting scene")

        Initializer.reset_scene()
        model_object = bpy.data.objects[params.model_name]
        model_object.hide = False
        model_object.hide_render = False
        return {'FINISHED'}


def populate_coll(scene):
    """
    This is required by Blender to populate the environment with all the required panel
    and Operator code.
    """

    bpy.app.handlers.scene_update_pre.remove(populate_coll)
    scene.coll.clear()
    for _, name, _ in enum_items:
        scene.coll.add().name = name


def register():
    """
    This is the first method every called. This registers the :class:`~am3.panel.SimulatorPanel`,
    :class:`~am3.panel.SimulateOperator`, :class:`~am3.panel.ChunkOperator`,
    and :class:`~am3.panel.ResetOperator`, as well as creating all the stored properties for the
    user-defined parameters. Under the hood, the widgets in the panel actually modify these
    properties that are stored in the Blender scene.
    """

    bpy.utils.register_class(SimulatorPanel)
    bpy.utils.register_class(SimulateOperator)
    bpy.utils.register_class(ChunkOperator)
    bpy.utils.register_class(ResetOperator)
    bpy.utils.register_class(JsonFileOperator)

    bpy.types.Scene.ObjectModel = bpy.props.StringProperty()
    bpy.types.Scene.BuildDepth = bpy.props.FloatProperty(
        name="Build Depth", min=0, max=100, default=50)
    bpy.types.Scene.SlopeAngle = bpy.props.FloatProperty(
        name="Slope Angle °", min=0, max=90, default=45)
    bpy.types.Scene.PrintheadDepth = bpy.props.FloatProperty(
        name="Printhead Depth", min=0, max=20, default=5)
    bpy.types.Scene.RobotSpeed = bpy.props.FloatProperty(
        name="Robot Speed", min=0.1, max=100, default=10)
    bpy.types.Scene.SuggestRobots = bpy.props.BoolProperty(
        name="Suggest Number of Robots", default=True)
    bpy.types.Scene.NumberOfRobots = bpy.props.IntProperty(
        name="Number of Robots", min=1, max=32, default=4)
    bpy.types.Scene.ModelScale = bpy.props.FloatProperty(
        name="Model Scale", min=0.01, max=100, default=1.0)
    bpy.types.Scene.FramesPerSecond = bpy.props.FloatProperty(
        name="Frames Per Second", min=0.01, max=300, default=2)
    bpy.types.Scene.WriteSimulationFile = bpy.props.BoolProperty(
        name="Write Simulation File", default=False)
    bpy.types.Scene.SliceThickness = bpy.props.FloatProperty(
        name="Slice Thickness", min=0.01, max=5, default=0.5)
    bpy.types.Scene.ExtrusionDiameter = bpy.props.FloatProperty(
        name="Extrusion Diameter", min=0.005, max=2.5, default=0.25)
    bpy.types.Scene.MaterialColor = bpy.props.FloatVectorProperty(
        name="MaterialColor",
        subtype="COLOR",
        size=4,
        min=0.0,
        max=1.0,
        default=(1.0, 1.0, 1.0, 1.0)
    )
    bpy.types.Scene.SimulationJson = bpy.props.StringProperty(subtype="FILE_PATH")


def unregister():
    """
    This unregisters all the items registered in the :func:`~am3.panel.register` function.
    Refer to that method for specifics on what is registered and unregistered. This method won't
    often be used, but it must be used if you do not wish to close your Blender file but need to
    reload code changes. Registering a panel or operator that already exists throws an error in
    Blender.
    """

    bpy.utils.unregister_class(SimulatorPanel)
    bpy.utils.unregister_class(SimulateOperator)
    bpy.utils.unregister_class(ChunkOperator)
    bpy.utils.unregister_class(ResetOperator)
    bpy.utils.unregister_class(JsonFileOperator)

    del bpy.types.Scene.ObjectModel
    del bpy.types.Scene.BuildDepth
    del bpy.types.Scene.SlopeAngle
    del bpy.types.Scene.PrintheadDepth
    del bpy.types.Scene.RobotSpeed
    del bpy.types.Scene.SuggestRobots
    del bpy.types.Scene.NumberOfRobots
    del bpy.types.Scene.ModelScale
    del bpy.types.Scene.FramesPerSecond
    del bpy.types.Scene.WriteSimulationFile
    del bpy.types.Scene.SliceThickness
    del bpy.types.Scene.ExtrusionDiameter
    del bpy.types.Scene.SimulationJson
