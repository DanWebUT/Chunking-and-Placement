import bpy
import os
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

stl_file_name = argv[0]
print("Chunking file: {}".format(stl_file_name))

directory = os.path.dirname(bpy.data.filepath)
if not directory in sys.path:
    sys.path.append(directory)

from mathutils import Vector

from am3.machine.machine import Machine, MachineParameters
from am3.chunker.chunker import Chunker


def export_stl(export_chunk):
    # Select this chunk for exporting
    bpy.ops.object.select_all(action='DESELECT')
    export_chunk.model.select = True

    # Create the output directory
    output_folder_path = os.path.join(bpy.path.abspath('//'), 'export')
    os.makedirs(output_folder_path, exist_ok=True)

    # Write to {chunk.number}.stl
    output_file_path = os.path.join(output_folder_path, '{}.stl'.format(export_chunk.number))
    bpy.ops.export_mesh.stl(filepath=output_file_path, use_selection=True)


# construct a basic machine
machine_parameters = MachineParameters(
    build_depth=300,
    printhead_slope=60 * 3.14159 / 180,
    machine_width=300,
    printhead_depth=300,
    machine_number=0,
    temperature=0.0,
    machine_speed=0.1,
    model_shift=Vector((0, 0, 0)),
    printhead_height=0.0
)

machine1 = Machine(machine_parameters=machine_parameters)
machine2 = Machine(machine_parameters=machine_parameters)
machine3 = Machine(machine_parameters=machine_parameters)
machine4 = Machine(machine_parameters=machine_parameters)

#machines = [machine1, machine2, machine3, machine4]
machines = [machine1, machine2]
# import an STL
filename = os.path.join(bpy.path.abspath("//"), stl_file_name)
bpy.ops.import_mesh.stl(filepath=filename)

imported_model = bpy.context.object

#recender the object
Chunker.recenter_object(imported_model)

Chunker.start_2_robots_sameside(machines, imported_model, 2)

bpy.ops.object.mode_set(mode='OBJECT')

for machine in machines:
    for chunk in machine.chunks:
        if not chunk.is_empty():
            export_stl(chunk)
