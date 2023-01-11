"""
This module contains classes which are responsible for performing the logic of manipulating a Blender
scene for various reasons. These include simulating a fresh slicing job, loading a JSON Simulation,
and doing rudimentary operations such as importing the Robot model and resetting the scene.
"""

import bpy
import json
import os
import time

from typing import List, Union, Tuple, Dict
from am3.util import SimulatorMath
from am3.robot import ChunkFrame, Robot, RobotParameters
from am3.simulator.material import Material
from am3.simulator.simulation import Simulation
from mathutils import Vector


class SimulatorScene:
    """
    This class is responsible for performing the logic to convert the simulation frames stored
    in each Robot's chunks into a single list of simulation frames, as well as rendering these
    frames in the form of Blender animations. The main simulation method is 
    :func:`~am3.simulator.scene.SimulatorScene.simulate`.
    """

    @staticmethod
    def still_running(finished_robots: List[bool]) -> bool:
        """
        Checks if the simulation is finished (i.e., are any robots `not` finished?).

        :param finished_robots: a list of booleans indexed by robot number. The value at a
            given index should indicate if the robot is finished or not.
        :type finished_robots: List[bool]
        :return: whether or not all robots are finished
        :rtype: bool
        """

        for value in finished_robots:
            if value is False:
                return True

    @staticmethod
    def write_simulation_to_file(simulation: Simulation, indent: bool=False):
        """
        Writes ``simulation`` to JSON. Assuming the entire simulation has been generated
        and stored in ``simulation``, this JSON file is the final \"simulation data file\",
        which can be imported in any other AM3 Simulator (e.g., the Three.JS simulator).\n
        The name format for this file is: ``simulation_data_{current_timestamp_ms}.json``.
        \n
        The indentation can be specified with the ``indent`` parameter. By default, there is no
        indentation, since whitespace for a file this large can take up to 80% of the file size.
        Thus, it is not recommended to use indentation except for debug purposes.

        :param simulation: a \"Simulation\" object
        :type simulation: Simulation
        :param indent: whether or not to indent the output JSON file with whitespace. You probably
            shouldn't.
        :type indent: bool
        """

        timestamp = int(time.time())
        filename = os.path.join(bpy.path.abspath('//'), 'simulation_data_{}.json'.format(timestamp))
        with open(filename, 'w') as outfile:
            if indent:
                outfile.write(json.dumps(simulation.serialize(), indent=2))
            else:
                outfile.write(json.dumps(simulation.serialize()))

        print('File written to {}'.format(filename))

    @staticmethod
    def simulate(robots: List[Robot], write_simulation_file=False):
        """
        Executes the logic for generating a simulation given a list of Robots containing
        \"ready\" chunks. Chunks that are \"ready\" have the following characteristics:

            * The dependencies between chunks exist
            * The chunks have been sliced
            * The chunks have subsequently had chunk frames generated after slicing

        This method does the following things:

            * Sets up the Blender scene for a fresh animation
            * Creates \"trackers\", to track finished robots, finished chunks, the current chunks
              a robot is working on, and the current frame of a chunk that is being worked on.
            * Makes a pass through all the robots, evaluating for each robot what "data" they 
              will contribute to the current frame. Once each robot has contributed one chunk
              frame to the current simulation frame, the simulation frame is animated and a new
              frame is created.
            * Once all robots have been marked as finished (in ``finished_robots``), the
              simulation ends.

        :param robots: a List of initialized robots. These should be in their start positions,
            and should each possess chunks for printing, meeting the \"ready\" characteristics
            mentioned above.
        :type robots: List[Robot]
        :param write_simulation_file: whether or not to write the simulation data to a JSON file,
            in addition to rendering it in Blender. (default ``False``)
        :type write_simulation_file: bool
        """

        finished_robots = []
        finished_chunks = []
        current_chunks = []
        current_data_indices = []

        for robot in robots:
            robot.set_keyframe()
            finished_robots.append(False)
            current_chunks.append(0)
            current_data_indices.append(0)

        scene = bpy.context.scene
        bpy.context.scene.frame_set(0)

        print("SIMULATOR: Animating build")
        print("Frame estimation: {}", SimulatorMath.estimate_execution_time(robots))

        if write_simulation_file:
            simulation = Simulation(robots=robots)

        # go back to frame 0
        scene.frame_set(0)
        current_frame = 0

        for robot in robots:
            for chunk in robot.chunks:
                for chunk_frame in chunk.frame_data:
                    if chunk_frame.has_material():
                        for material in chunk_frame.materials:
                            material.set_hidden(True)
                            material.set_keyframe()

        while SimulatorScene.still_running(finished_robots):
            if current_frame % 100 == 0:
                print("Frame {}".format(current_frame))

            scene.frame_set(current_frame - 1)
            for robot in robots:
                robot.set_last_location()
                robot.set_keyframe()
            scene.frame_set(current_frame)

            material_added = []

            for i in range(0, len(robots)):
                if finished_robots[i] is True:
                    continue
                robot = robots[i]
                current_chunk_index = current_chunks[i]
                
                if current_chunk_index >= len(robot.chunks):
                    finished_robots[i] = True
                    continue

                current_chunk = robot.chunks[current_chunk_index]
                current_data_index = current_data_indices[i]
                chunk_frame_data = current_chunk.frame_data

                # If this chunk is ready to be printed (i.e. no filaments is needed for this 
                # region or the dependencies are satisfied)
                if current_chunk.is_empty() or current_chunk.dependencies_satisfied(finished_chunks):
                    current_data_index += 1
                    current_data_indices[i] = current_data_index
                    if current_data_index >= len(chunk_frame_data):
                        robot.set_last_location()
                        robot.set_keyframe()

                        current_chunks[i] += 1

                        if current_chunks[i] >= len(robot.chunks):
                            finished_robots[i] = True
                        else:
                            next_chunk = robot.chunks[current_chunks[i]]
                            if next_chunk.is_empty():
                                next_location = next_chunk.frame_data[1].location
                                robot.set_location(next_location)
                                robot.set_keyframe()
                        current_data_indices[i] = 0

                        finished_chunks.append(current_chunk.number)
                    else:
                        frame_data = chunk_frame_data[current_data_index]
                        location = frame_data.location
                        materials = frame_data.materials

                        robot.set_location(location)
                        robot.set_keyframe()

                        if materials:
                            for material in materials:
                                material_added.append(material)
                                material.set_hidden(False)
                                material.set_keyframe()

            if write_simulation_file:
                simulation.add_frame(robots, material_added)
                material_added.clear()

            current_frame += 1
            scene.frame_end = current_frame
            scene.frame_set(current_frame)

        if write_simulation_file:
            print("Writing simulation data to file...")
            SimulatorScene.write_simulation_to_file(simulation)
        
        scene.frame_end = current_frame
        print("Finished rendering")


class Initializer:
    """
    This class contains static methods that are all rudimentary operations performed when initializing
    a Blender scene. For example, :func:`~am3.simulator.scene.Initializer.reset_scene()` resets the
    scene to a state with no material or chunks, and either 0 or 1 robots in the scene.
    """

    @staticmethod
    def reset_scene():
        """
        Resets the scene, removing all current robots (except robot 0), all chunks, and all extruded
        material. Robot 0 will have its location set to (0, 0, 0)
        """

        scene = bpy.context.scene

        for obj in scene.objects:
            name = obj.name
            if 'Robot Body' in name and name != 'Robot Body 0':
                obj.hide = False
                obj.select = True
            elif 'Robot Printhead' in name and name != 'Robot Printhead 0':
                obj.hide = False
                obj.select = True
            elif "Extrusion" in name or "Chunk" in name or name == 'profile':
                obj.hide = False
                obj.select = True
            else:
                obj.select = False

        if scene.objects['Robot Body 0']:
            scene.objects['Robot Body 0'].location = Vector((0, 0, 0))
        if scene.objects['Robot Printhead 0']:
            scene.objects['Robot Printhead 0'].location = Vector((0, 0, 0))

        bpy.ops.object.delete()

    @staticmethod
    def is_robot_already_imported() -> bool:
        """
        Checks if the robot body and printhead have already been imported from their OBJ files

        :return: ``True`` if the robot OBJ file has already been imported, ``False`` otherwise.
        :rtype: bool
        """
        return 'Robot Body 0' in bpy.data.objects and 'Robot Printhead 0' in bpy.data.objects

    @staticmethod
    def import_robot_obj():
        """
        Imports the robot body and printhead from their OBJ files.

        :param filename: the filename of the OBJ file to import (default ``Robot.obj``)
        :type filename: str
        :return: the Blender objects imported from the OBJ file
        :rtype: List[object]
        """
        filename = os.path.join(bpy.path.abspath("//"), 'Robot.obj')
        bpy.ops.import_scene.obj(filepath=filename)

        objects = bpy.context.selected_objects
        if 'BODY' in objects[0].name:
            objects[0].name = 'Robot Body 0'
            objects[1].name = 'Robot Printhead 0'
        else:
            objects[0].name = 'Robot Printhead 0'
            objects[1].name = 'Robot Body 0' 

        return objects

    @staticmethod
    def import_robots(count: int, parameters: RobotParameters = None) -> List[Robot]:
        """
        Creates ``count`` robots. Existing robots, chunks, and material are removed in the process.
        The robots are returned with (0, 0, 0) as their initial positions, in order of their robot
        number, which will range from ``[0.. count-1]``.
        """

        Initializer.reset_scene()
        if not Initializer.is_robot_already_imported():
            Initializer.import_robot_obj()

        if not parameters:
            parameters = RobotParameters()

        robot0 = Robot(parameters=parameters)
        robot0.body_model = bpy.data.objects['Robot Body 0']
        robot0.printhead_model = bpy.data.objects['Robot Printhead 0']
        robot0.set_hidden(False)
        robot0.clear_animation_data()

        robots = [robot0]
        for i in range(1, count):
            new_robot = robots[-1].copy()
            new_robot.set_number(i)
            robots.append(new_robot)

        assert len(robots) == count, 'Programmer error, the list of robots is the incorrect size'
        return robots


class JsonLoader:
    """
    This class is responsible for loading a simulation from a JSON file.
    """

    @staticmethod
    def load_from_simulation_file(
            simulation_file_path: str,
            material_color: Tuple[int, int, int] = (255, 255, 255)) -> bool:
        """
        Does the heavy lifting of loading a previously-generated simulation from a JSON file and
        setting the Blender scene to animate the data in that file.

        :param simulation_file_path: the file name for the JSON file
        :type simulation_file_path: str
        :return: whether or not the loading was successful
        :rtype: bool
        """

        if not simulation_file_path.endswith('.json'):
            return False

        with open(simulation_file_path, 'r') as json_file:
            simulation_data = json.load(json_file)

        simulation = Simulation(data=simulation_data)
        robots = Initializer.import_robots(simulation.robot_count())

        scene = bpy.context.scene
        scene.frame_set(0)
        scene.frame_end = simulation.frame_count()

        print("WILL RENDER {} FRAMES".format(simulation.frame_count()))

        for robot, init_robot in zip(robots, simulation.init.robots):
            robot.set_location(init_robot.location)
            robot.set_rotation(init_robot.rotation)
            robot.set_keyframe()

        all_material = []
        for i, frame in enumerate(simulation.frames):
            if i % 20 == 0:
                print("CREATING MATERIAL IN FRAME {}".format(i))

            materials = [Material(material.points, material_color) for material in frame.materials]
            all_material.append(materials)
            for material in materials:
                material.set_hidden(True)
                material.set_keyframe()

        for i, frame in enumerate(simulation.frames):
            if i % 50 == 0:
                print("WORKING ON FRAME {}".format(i))

            scene.frame_set(i + 1)

            frame_robots = frame.robots
            frame_materials = all_material[i]

            for material in frame_materials:
                material.set_hidden(False)
                material.set_keyframe()

            for robot, frame_robot in zip(robots, frame_robots):
                robot.set_location(frame_robot.location)
                robot.set_rotation(frame_robot.rotation)
                robot.set_keyframe()

        return True
