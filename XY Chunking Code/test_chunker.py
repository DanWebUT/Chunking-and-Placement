"""
This is a program to work with the chunker.py file without having to import
an stl file. Instead, I create a simple opject and chunk that.
-Daniel Weber
"""
import bpy
import os
import sys

directory = os.path.dirname(bpy.data.filepath)
if not directory in sys.path:
    sys.path.append(directory)
    
from mathutils import Vector

from am3.machine.machine import Machine, MachineParameters
from am3.chunker.chunker import Chunker

#This is export code copied from blender_chunk.py
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


# Copied from blender_chunk.py. Construct a basic machine
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
machines = [machine1,machine2,machine3,machine4]

"""
First Test: Create a new python file and export the stl
"""
bpy.ops.object.select_all(action='TOGGLE')
bpy.ops.object.select_all(action='TOGGLE')
bpy.ops.object.delete(use_global=False)
bpy.ops.mesh.primitive_cube_add(radius=1, location=(0, 0, 0))
bpy.ops.transform.resize(value=(3, 3, 6), constraint_axis=(True, True, True), constraint_orientation='GLOBAL')
#this is the resize command to put it into the right unit structure (mm)
bpy.ops.transform.resize(value=(100, 100, 100))


imported_model = bpy.context.object

#recender the object
Chunker.recenter_object(imported_model)

Chunker.start_scaled_layers(machines, imported_model)

bpy.ops.object.mode_set(mode='OBJECT')

for machine in machines:
    for chunk in machine.chunks:
        if not chunk.is_empty():
            export_stl(chunk)

"""
This was a test to see if the cube could be exported at all, is successfull
# Create the output directory
output_folder_path = os.path.join(bpy.path.abspath('//'), 'export')
os.makedirs(output_folder_path, exist_ok=True)

# Write to {chunk.number}.stl
output_file_path = os.path.join(output_folder_path, '{}.stl'.format(1)) #the 1 was for test
bpy.ops.export_mesh.stl(filepath=output_file_path, use_selection=True)
"""









