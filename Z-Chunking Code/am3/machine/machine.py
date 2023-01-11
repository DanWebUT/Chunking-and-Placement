import bpy
from am3.commands.command import *
from am3.settings import *
from mathutils import *
from am3.simulator.printed import Printed
from am3.util.simulatormath import SimulatorMath

class MachineParameters:
    def __init__(self,
                 build_depth=0,
                 printhead_slope=0,
                 machine_width=0,
                 printhead_depth=0,
                 machine_number=0,
                 temperature=0.0,
                 machine_speed=0.1,
                 model_shift=Vector((0, 0, 0)),
                 printhead_height=0.0):
        self.build_depth = build_depth
        self.printhead_slope = abs(printhead_slope)
        self.machine_width = machine_width
        self.printhead_depth = printhead_depth
        self.machine_number = machine_number
        self.temperature = temperature
        self.machine_speed = machine_speed
        self.model_shift = model_shift
        self.printhead_height = printhead_height
        self.chunks = []

    def copy(self):
        new_machine_parameters = MachineParameters()
        new_machine_parameters.build_depth = self.build_depth
        new_machine_parameters.printhead_slope = self.printhead_slope
        new_machine_parameters.machine_width = self.machine_width
        new_machine_parameters.temperature = self.temperature
        new_machine_parameters.machine_number = self.machine_number + 1
        new_machine_parameters.machine_speed = self.machine_speed
        new_machine_parameters.model_shift = self.model_shift.copy()
        new_machine_parameters.printhead_height = self.printhead_height

        return new_machine_parameters

class Machine:
    # initialize a machine.
    # bDepth: Build depth of machine, in meters
    # pSlope: Printhead slope, as a float (ignoring sign) [think y = mx + b, slope should be "m"]
    # mWidth: Width of the Machine. Not needed for chunker
    # pDepth : Printhead depth, in meters
    def __init__(self, machine_parameters):
        self.machine_parameters = machine_parameters

        self.build_depth = machine_parameters.build_depth
        self.printhead_slope = machine_parameters.printhead_slope
        self.machine_width = machine_parameters.machine_width
        self.printhead_depth = machine_parameters.printhead_depth

        #Machine blender model as a mesh object
        self.machine_model = None
        self.printhead_model = None
        self.temperature = machine_parameters.temperature
        self.machine_speed = machine_parameters.machine_speed
        self.printhead_height = machine_parameters.printhead_height
        self.commands = [] # G, M, T commands as objects (classes below)
        self.points = []
        self.axes = ['X', 'Y', 'Z', 'F', 'S', 'C']
        self.layers = 0
        self.printing = False
        self.printed = []
        self.drawn = []
        self.chunks = []
        self.machine_number = machine_parameters.machine_number
        self.model_shift = machine_parameters.model_shift
        self.notify_map = {}
        self.all_dependencies = {}

    def copy(self, flip_direction=False):
        new_machine_parameters = self.machine_parameters.copy()
        new_machine = Machine(new_machine_parameters)

        # duplicate object
        scene = bpy.context.scene

        # scene.objects.active = self.machine_model
        new_machine_model = self.machine_model.copy()
        new_machine_model.data = self.machine_model.data.copy()
        new_machine_model.animation_data_clear()
        scene.objects.link(new_machine_model)
        new_machine_model.name = 'Robot Body {}'.format(new_machine.machine_number)
        new_machine.machine_model = new_machine_model

        # scene.objects.active = self.printhead_model
        new_printhead_model = self.printhead_model.copy()
        new_printhead_model.data = self.printhead_model.data.copy()
        new_printhead_model.animation_data_clear()
        scene.objects.link(new_printhead_model)
        new_printhead_model.name = 'Robot Printhead {}'.format(new_machine.machine_number)
        new_machine.printhead_model = new_printhead_model

        if flip_direction:
            new_machine.flip()

        return new_machine


    def print_machine_parameters(self):
        print("Build Depth: {}, Printhead Slope: {}, Machine Width: {}, Printhead Depth: {}".format(
            self.build_depth,
            self.printhead_slope,
            self.machine_width,
            self.printhead_depth))

    def set_location(self, location):
        self.last_location = location
        self.machine_model.location[0] = location[0] + self.model_shift[0]
        self.machine_model.location[1] = location[1] + self.model_shift[1]

        self.printhead_model.location[0] = location[0] + self.model_shift[0]
        self.printhead_model.location[1] = location[1] + self.model_shift[1]
        self.printhead_model.location[2] = location[2] + self.model_shift[2]

    def set_last_location(self):
        self.set_location(self.last_location)

    def set_number(self, number):
        self.machine_number = number
        self.machine_parameters.number = number
        self.machine_model.name = 'Robot Body {}'.format(number)
        self.printhead_model.name = 'Robot Printhead {}'.format(number)

    def set_hidden(self, hidden):
        self.machine_model.hide = hidden
        self.printhead_model.hide = hidden

    def rotate(self, amount):
        self.machine_model.rotation_euler.z += amount
        self.printhead_model.rotation_euler.z += amount

    def flip(self):
        self.model_shift[0] *= -1
        self.model_shift[1] *= -1
        self.rotate(math.pi)

    def set_keyframe(self):
        self.machine_model.keyframe_insert(data_path="location")
        self.printhead_model.keyframe_insert(data_path="location")

    def get_location(self):
        return self.printhead_model.location - self.model_shift

    def clear_animation_data(self):
        self.machine_model.animation_data_clear()
        self.printhead_model.animation_data_clear()

    # slicedChunks: [[[[Vector]]]]
    def generate_commands(self, ):
        last_location = self.get_location()
        transitioning = False

        for chunk in self.chunks:
            chunk.commands.append(ToolOffCommand())

            for slice in chunk.slices:
                for path in slice.paths:
                    if not path.vectors:
                        continue
                    if transitioning:
                        path_commands = self.get_path_between_points(last_location, path[0])
                        for command in path_commands:
                            chunk.commands.append(command)
                        transitioning = False

                    chunk.commands.append(MoveCommand(path[0], self.machine_speed))
                    last_location = path[0]
                    chunk.commands.append(ToolOnCommand())

                    for location in path.vectors: # location: Vector
                        chunk.commands.append(MoveCommand(location, self.machine_speed))
                        last_location = location

                    chunk.commands.append(ToolOffCommand())
                chunk.commands.append(NewLayerCommand())
                chunk.commands.append(ToolOffCommand())
            
            transitioning = True
            chunk.commands.append(ToolOffCommand())
            self.set_path(chunk)

    def get_path_between_points(self, point1, point2):
        commands = []
        next_location = Vector((0, 0, 0.05)) + point1
        commands.append(MoveCommand(next_location, self.machine_speed))
        next_location = Vector((point2[0], point2[1], point1[2] + 0.05))
        commands.append(MoveCommand(next_location, self.machine_speed))
        next_location = point2
        commands.append(MoveCommand(next_location, self.machine_speed))

        return commands

    def set_path(self, chunk):
        chunk.frame_data = []
        chunk.frame_data.append((self.get_location(), None))

        tool_on = False
        layer = 1
        
        accumulated_time = 0
        accumulated_extruder_path = []

        last_location = self.get_location()
        current_command = 0

        printed_objects = []
        time_step = 1 / am3.settings.FRAMES_PER_SECOND

        current_frame = 0

        while current_command < len(chunk.commands):
            command = chunk.commands[current_command]
            command_increment = 1

            # Tool On Command
            if isinstance(command, ToolOnCommand):
                if not tool_on:
                    tool_on = True
                    accumulated_extruder_path = [last_location]

            # Tool Off Command
            elif isinstance(command, ToolOffCommand):
                if tool_on:
                    tool_on = False
                    accumulated_extruder_path.append(last_location)
                    printed_objects.append(Printed(accumulated_extruder_path, current_frame, chunk.color))
                    accumulated_extruder_path = []

            # New Layer Command
            elif isinstance(command, NewLayerCommand):
                layer += 1

            # Move Command - if command can be completed before the frame finishes, go ahead and
            # add it. Otherwise, only complete part of the command, using linear interpolation
            elif isinstance(command, MoveCommand):
                last_color = command.color
                time_to_complete = command.calculateTime(last_location)

                if accumulated_time + time_to_complete < time_step:
                    accumulated_time += time_to_complete
                    if tool_on:
                        accumulated_extruder_path.append(command.location)
                    last_location = command.location
                else:
                    percent_of_path_to_complete = (time_step - accumulated_time) / time_to_complete

                    next_location = SimulatorMath.linearInterpolation(
                        last_location, command.location, percent_of_path_to_complete)

                    accumulated_time = 0

                    if tool_on:
                        accumulated_extruder_path.append(next_location)
                    last_location = next_location
                    printed_objects.append(Printed(accumulated_extruder_path, current_frame, chunk.color))
                    chunk.frame_data.append((last_location, printed_objects))
                    printed_objects = []

                    current_frame += 1
                    command_increment = 0

            current_command += command_increment

        if printed_objects:
            chunk.frame_data.append((last_location, printed_objects))
