import bpy
import os
import sys
import json
import argparse
from math import radians
from mathutils import Matrix

parser = argparse.ArgumentParser()
parser.add_argument(
    "-f", "--file-name", type=str, dest="file_name", help="the STL file to chunk"
)
parser.add_argument(
    "-n",
    "--number-of-robots",
    type=int,
    dest="number_of_bots",
    help="the number of robots to use",
)
parser.add_argument(
    "-c",
    "--center",
    dest="should_center",
    action="store_const",
    const=True,
    default=False,
    help="should center the object model",
)

argv = sys.argv
argv = argv[argv.index("--") + 1 :]  # get all args after "--"

args = parser.parse_args(argv)

stl_file_name = args.file_name
number_of_bots = args.number_of_bots
should_center = args.should_center

print("Chunking file: {}".format(stl_file_name))

directory = sys.path.append(os.path.dirname(os.path.abspath(__file__)))
if not directory in sys.path:
    sys.path.append(directory)

from mathutils import Vector

import am3
from am3.machine.machine import Machine, MachineParameters
from am3.chunker.chunker import Chunker


def get_output_file_name(output_file_name):
    file_path_array = stl_file_name.rsplit(os.path.sep, 1)
    file_path = file_path_array[0]
    print(file_path)

    output_folder = os.path.join(bpy.path.abspath("//"), file_path)

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    return os.path.join(output_folder, output_file_name)


def write_string_to_file(string_value, file_name):
    text_file = open(file_name, "w")
    text_file.write(string_value)
    text_file.close()


def export_stl(obj, machine_number):
    align_models_on_build_plate(obj.model)
    bpy.ops.object.select_all(action="DESELECT")
    obj.model.select = True
    chunk_name = "m" + str(machine_number) + "_c" + str(chunk.number)
    outfile = get_output_file_name(chunk_name + ".stl")

    bpy.ops.export_mesh.stl(filepath=outfile, use_selection=True)


def write_machine_chunks_file(machine_chunks):
    outfile = get_output_file_name("machine_chunks.json")
    write_string_to_file(json.dumps(machine_chunks), outfile)


def write_chunk_dependencies_file(chunk_dependencies):
    outfile = get_output_file_name("chunk_dependencies.json")
    write_string_to_file(json.dumps(chunk_dependencies), outfile)


def write_empty_chunks_file(empty_chunks):
    outfile = get_output_file_name("empty_chunks.json")
    write_string_to_file(json.dumps(empty_chunks), outfile)


# This function writes the error after chunk validation
def write_error_file(error_msg):
    outfile = get_output_file_name("error_logs")
    write_string_to_file(json.dumps(error_msg), outfile)


# prints out the vertical chunk information to a json file
def write_layer_information(layers):
    outfile = get_output_file_name("vertical_layer_information.json")
    write_string_to_file(json.dumps(layers), outfile)


def calculate_bounds(obj, local=False):
    local_coordinates = obj.bound_box[:]
    object_matrix = obj.matrix_world

    if not local:
        worldify = lambda p: object_matrix * Vector(p[:])
        coordinates = [worldify(p).to_tuple() for p in local_coordinates]
    else:
        coordinates = [p[:] for p in local_coordinates]

    rotated = zip(*coordinates[::-1])

    push_axis = []
    for (axis, _list) in zip("xyz", rotated):
        info = lambda: None
        info.max = max(_list)
        info.min = min(_list)
        info.distance = info.max - info.min
        push_axis.append(info)

    import collections

    originals = dict(zip(["x", "y", "z"], push_axis))

    o_details = collections.namedtuple("object_details", "x y z")
    return o_details(**originals)


def align_models_on_build_plate(model):
    bounds = calculate_bounds(model)
    # align model to the left edge of the build plate
    x_build_plate_edge = 0
    x_offset = bounds.x.min - x_build_plate_edge
    model.location[0] -= x_offset
    # center the model in the middle of the build plate
    y_build_plate_center = -150
    y_model_center = (bounds.y.min + bounds.y.max) / 2
    y_offset = y_build_plate_center - y_model_center
    model.location[1] -= y_offset


def center_model(model):
    bounds = calculate_bounds(model)
    min_x = bounds.x.min
    max_x = bounds.x.max
    min_y = bounds.y.min
    max_y = bounds.y.max

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    model.location[0] -= center_x
    model.location[1] -= center_y


machines = []

lower_bound_slope_angle = 0.34906555
upper_bound_slope_angle = 1.570795
maximum_depth = 350
maximum_width = 350
wing_slope = 50.00

for x in range(0, int(number_of_bots)):
    # construct a basic machine
    machine_parameters = MachineParameters(
        build_depth=350,
        printhead_slope=radians(70),
        machine_width=350,
        printhead_depth=350,
        machine_number=x,
        temperature=0.0,
        machine_speed=0.1,
        model_shift=Vector((0, 0, 0)),
        printhead_height=0.0,
    )
    machines.append(Machine(machine_parameters=machine_parameters))

    # error reporting if the chunking constraints voilated
    # The block below checks for the chunk angle. If chunk angle is larger than 90 or smaller than 20, it reports error
    if (
        machine_parameters.printhead_slope < lower_bound_slope_angle
        or machine_parameters.printhead_slope > upper_bound_slope_angle
    ):
        # throw error and write error to json file in json format
        error_msg = "The value of slope is not between 20 degrees and 90 degrees"
        write_error_file(error_msg)
        sys.exit()

    # the block below checks for the depth (print_head depth) of chunk to ensure that the chunk width is not larger than the reach of the robot.
    # if machine_parameters.printhead_depth > maximum_depth:
    #     error_msg = "The print_head is larger than the reach of the robot"
    #     write_error_file(error_msg)
    #     sys.exit()

    # This block checks for the width of the chunk to ensure that the chunk width is not so large that the scara arm cannot print it.
    if machine_parameters.machine_width > maximum_width:
        error_msg = "The maximum width of the chunk is larger than the reach of the printing robot"
        write_error_file(error_msg)
        sys.exit()


# import an STL
filename = os.path.join(bpy.path.abspath("//"), stl_file_name)
bpy.ops.import_mesh.stl(filepath=filename)

imported_model = bpy.context.object
if should_center:
    center_model(imported_model)

layers = {}

# global variable to change the chunking option
chunk_option = 2
"""
if  chunk_option == 1 --> same side chunking
    chunk_option == 2 --> vertical chunking
    chunk_option == 3 --> same side vertical chunking
otherwise,
    normal chunking

"""

if chunk_option == 1:
    Chunker.start_scaled_oneSided(machines, imported_model)
elif chunk_option == 2:
    Chunker.start_scaled_layers(machines, imported_model)
elif chunk_option == 3:
    Chunker.start_scaled_layers_sameside(machines, imported_model)
else:
    imported_model.rotation_euler = (
        imported_model.rotation_euler.to_matrix() * Matrix.Rotation(radians(90), 3, "Z")
    ).to_euler()

    Chunker.start_scaled(machines, imported_model)


if chunk_option == 2 or chunk_option == 3:
    (layer, h, z_height) = Chunker.vertical_layers(imported_model)
    for i in range(0, layer):
        if i < layer-1:
            z_max = h[i]
        else:
            z_max = z_height
        layers[i] = z_max
        write_layer_information(layers)


bpy.ops.object.mode_set(mode="OBJECT")

dependencies = {}
machine_chunks = []
empty_chunks = {}

for machine in machines:
    chunk_numbers = []

    for chunk in machine.chunks:
        dependencies[chunk.number] = chunk.dependencies
        chunk_numbers.append(chunk.number)

        if not chunk.is_empty():
            export_stl(chunk, machine.machine_number)
        else:
            point = chunk.model
            empty_chunks[chunk.number] = [point[0], point[1], point[2]]

    machine_chunks.append(chunk_numbers)

write_machine_chunks_file(machine_chunks)
write_chunk_dependencies_file(dependencies)
write_empty_chunks_file(empty_chunks)
