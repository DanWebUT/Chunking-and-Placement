import math
import bpy
import bmesh
import pdb
import numpy as np

from am3.util.simulatormath import *
from am3.model import Chunk
from math import radians

counter_value_f = 0


class ChunkingParameters:

    build_plate_x = 300
    build_plate_y = 300
    starting_point = 50.0
    slope_y_dir = radians(160)
    max_reach_x = 425.00
    rest = 300.00
    max_reach_z = 20 #mm
    vertical_check_density = 5 #number of splits created to check disjoint XY chunks or islands without AGs
    

class AGParameters:
    AG = True #set True to add Alignment Geometry
    spacing = 20 #mm
    offset = 2 #mm distance AG should be from edge
    min_AG_number = 5
    fit_multiplier = 1.15 #how much larger the 'hole' is than the 'peg'
    grid_density = 1 #mm grid of interface check, lower number for finer detail
    min_AG_rad = .5 #mm, determined by printer performance
    min_AG_height = 1.5*min_AG_rad #mm determined by printer performance
    rad_inc = .5 #mm, how much the radius increases during each trial iteration, lower = faster runtime
    max_AG_rad = 4

class ChunkerResult:
    north_chunks = []
    origin_chunk = None
    south_chunks = []
    bottom_layer = []

    def __init__(self):
        pass

    def set_origin_chunk(self, origin):
        origin.hide_render = True
        self.origin_chunk = origin

    def add_north_chunk(self, north_chunk):
        north_chunk.hide_render = True
        self.north_chunks.append(north_chunk)

    def add_south_chunk(self, south_chunk):
        south_chunk.hide_render = True
        self.south_chunks.append(south_chunk)

    def add_bottom_layer(self, bottom_layer):
        bottom_layer.hide_render = True
        self.bottom_layer.append(bottom_layer)


class Chunker:
    @staticmethod
    def deselect_all_objects():
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except:
            print()

        bpy.ops.object.select_all(action='DESELECT')

    @staticmethod
    def assign_face_origin_data(mesh, plane1, plane2):
        # Calculate faces that lie on the planes
        origins = {}

        for polygon in mesh.polygons:
            if SimulatorMath.polygonLiesInPlane(mesh, polygon, plane1) or \
                    SimulatorMath.polygonLiesInPlane(mesh, polygon, plane2):
                origins[Chunker.vertices_to_string(polygon.vertices)] = 1
            else:
                origins[Chunker.vertices_to_string(polygon.vertices)] = 0

        # Now assign values in the dictionary to the Mesh object
        mesh["faceOrigins"] = origins

    @staticmethod
    def vertices_to_string(vertices):
        output = ""

        for i in range(0, 2):
            output += "{},".format(vertices[i])

        output += "{}".format(vertices[2])

        return output

    # Starts the AM3 Chunking process
    # Returns: String - error message, or None if no errors occured
    @staticmethod
    def start(machine_parameters, model_object, number_of_machines=2):
        if number_of_machines != 2:
            return Chunker.start_scaled(machine_parameters, model_object)

        Chunker.deselect_all_objects()

        chunker_result = ChunkerResult()
        scene = bpy.context.scene

        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object

        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active

        build_depth = machine_parameters.build_depth
        printhead_depth = machine_parameters.printhead_depth

        model_bounds = SimulatorMath.calculate_bounds(model_object)

        model_depth = model_bounds.y.max - model_bounds.y.min

        if model_depth < build_depth:
            origin_chunk = new_object
            chunker_result.set_origin_chunk(origin_chunk)
            return chunker_result

        angle_of_elevation = math.pi / 2 - machine_parameters.printhead_slope
        tan_angle = math.tan(angle_of_elevation)

        slope_width = (model_bounds.z.max - model_bounds.z.min) * tan_angle

        origin_chunk_width = min(build_depth, 2 * printhead_depth + 2 * slope_width)

        north_plane_co = Vector((0, origin_chunk_width / 2, 0))
        north_plane_no = Vector((0, 1, tan_angle))

        south_plane_co = Vector((0, -1 * origin_chunk_width / 2, 0))
        south_plane_no = Vector((0, -1, tan_angle))

        (origin_chunk, north_piece) = Chunker.split_model(new_object, (north_plane_co, north_plane_no))
        (origin_chunk, south_piece) = Chunker.split_model(origin_chunk, (south_plane_co, south_plane_no))

        chunker_result.set_origin_chunk(origin_chunk)

        north_y = origin_chunk_width / 2 + build_depth - slope_width
        remaining = north_piece
        while north_y < model_bounds.y.max:
            north_plane_co = Vector((0, north_y, 0))
            (north_chunk, remaining) = Chunker.split_model(remaining, (north_plane_co, north_plane_no))
            chunker_result.add_north_chunk(north_chunk)
            north_y += build_depth - slope_width
        chunker_result.add_north_chunk(remaining)

        south_y = -1 * origin_chunk_width / 2 - build_depth + slope_width
        remaining = south_piece
        while south_y > model_bounds.y.min:
            south_plane_co = Vector((0, south_y, 0))
            (south_chunk, remaining) = Chunker.split_model(remaining, (south_plane_co, south_plane_no))
            chunker_result.add_south_chunk(south_chunk)
            south_y -= build_depth - slope_width
        chunker_result.add_south_chunk(remaining)

        model_object.hide = True
        return chunker_result

    @staticmethod
    def start_single(machine, model_object):
        machine_parameters = machine.machine_parameters
        build_depth = machine_parameters.build_depth
        model_bounds = SimulatorMath.calculate_bounds(model_object)

        y_start = model_bounds.y.max - build_depth

        angle_of_elevation = math.pi / 2 - machine_parameters.printhead_slope
        tan_angle = math.tan(angle_of_elevation)

        slope_width = (model_bounds.z.max - model_bounds.z.min) * tan_angle

        plane_co = Vector((0, y_start, 0))
        plane_no = Vector((0, -1, tan_angle))

        counter = 0
        previous_piece = model_object

        print("Build depth: {}".format(build_depth))
        print("Slope width: {}".format(slope_width))

        while plane_co[1] > model_bounds.y.min - slope_width:
            print("Chunking... plane_co[1] = {}".format(plane_co[1]))
            (chunk, remaining) = Chunker.split_model(previous_piece, (plane_co, plane_no))
            new_chunk = Chunk(chunk, number=counter)
            if counter > 0:
                new_chunk.add_dependency(counter - 1)
            machine.chunks.append(new_chunk)

            counter = counter + 1
            plane_co = plane_co - Vector((0, build_depth - slope_width, 0))
            previous_piece = remaining

        new_chunk = Chunk(remaining, number=counter)
        machine.chunks.append(new_chunk)

    @staticmethod
    def start_oneSided(machine_parameters, model_object, number_of_machines=2):
        if number_of_machines != 2:
            return Chunker.start_scaled(machine_parameters, model_object)

        Chunker.deselect_all_objects()

        scene = bpy.context.scene

        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object

        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active

        chunker_result = ChunkerResult()
        build_depth = machine_parameters.build_depth

        chunk_depth = ChunkingParameters.build_plate_y

        model_bounds = SimulatorMath.calculate_bounds(model_object)

        model_depth = model_bounds.y.max - model_bounds.y.min

        if model_depth < build_depth:
            origin_chunk = new_object
            chunker_result.set_origin_chunk(origin_chunk)
            return chunker_result

        angle_of_elevation = math.pi / 2 - machine_parameters.printhead_slope
        tan_angle = math.tan(angle_of_elevation)

        slope_width = (model_bounds.z.max - model_bounds.z.min) * tan_angle

        z_min = model_bounds.z.min

        #origin_chunk_width = chunk_depth
        origin_chunk_width = 300.0


        south_plane_co = Vector((0, model_bounds.y.max - origin_chunk_width, 0))
        south_plane_no = Vector((0, -1, tan_angle))

        #(origin_chunk, north_piece) = Chunker.split_model(new_object, (north_plane_co, north_plane_no))
        (origin_chunk, south_piece) = Chunker.split_model(new_object, (south_plane_co, south_plane_no))

        chunker_result.set_origin_chunk(origin_chunk)

        south_y = south_plane_co[1] - chunk_depth - slope_width
        # south_y = -1 * origin_chunk_width / 2 - build_depth + slope_width
        remaining = south_piece
        while south_y > model_bounds.y.min:
            south_plane_co = Vector((0, south_y, 0))
            (south_chunk, remaining) = Chunker.split_model(remaining, (south_plane_co, south_plane_no))
            chunker_result.add_south_chunk(south_chunk)
            south_y -= build_depth - slope_width
        chunker_result.add_south_chunk(remaining)

        model_object.hide = True
        return chunker_result

    @staticmethod
    def start_2_robots_sameside(machines, model_object, number_of_machines):

        if len(machines) == 1:
            Chunker.start_single(machines[0], model_object)

        color = (0, 0, 255)
        swap_color = (255, 0, 0)

        machine_parameters = machines[0].machine_parameters
        machine_width = machine_parameters.machine_width
        printhead_slope = machine_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        chunker_result = Chunker.start_oneSided(machine_parameters, model_object, number_of_machines=2)

        chunk_row = Chunker.split(chunker_result.origin_chunk,
                                  model_width,
                                  machine_width,
                                  printhead_slope,
                                  num_pieces=-1)

        chunk_number = len(chunk_row)
        mid_point = math.ceil(chunk_number/2)

        for (i, chunk) in enumerate(chunk_row):
            new_chunk = Chunk(chunk, number=i)
            new_chunk.color = color
            color, swap_color = swap_color, color

            if len(machines) == 2:
                if i < mid_point:
                    machine_number = 0
                else:
                    machine_number = 1
            else:
                machine_number = i
                if machine_number >= len(machines):
                    machine_number = machine_number % len(machines)

            machines[machine_number].chunks.append(new_chunk)

            new_chunk.set_name("Machine {} Chunk {}".format(machine_number, i))

            if i % 2 == 1:
                new_chunk.add_dependency(i - 1)
                if (i + 1) < len(chunk_row):
                    new_chunk.add_dependency(i + 1)

        center_chunk_dependencies = list(range(0, len(chunk_row)))
        previous_row_dependencies = list(range(0, len(chunk_row)))
        last_chunk_number = center_chunk_dependencies[-1]

        row_number = 0

        for south_chunk in chunker_result.south_chunks:
            row_number += 1
            color, swap_color = swap_color, color
            chunk_row = Chunker.split(south_chunk,
                                      model_width,
                                      machine_width,
                                      printhead_slope,
                                      num_pieces=-1)
            next_row_dependencies = []

            for (i, chunk) in enumerate(chunk_row):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk, number=chunk_number)
                new_chunk.row = row_number
                new_chunk.color = color
                color, swap_color = swap_color, color
                new_chunk.add_dependency(previous_row_dependencies)

                if len(machines) == 2:
                    if i < mid_point:
                        machine_number = 0
                    else:
                        machine_number = 1
                else:
                    machine_number = i
                    if machine_number >= len(machines):
                        machine_number = machine_number % len(machines)

                machines[machine_number].chunks.append(new_chunk)

                new_chunk.set_name("Machine {} Chunk {}".format(machine_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            last_chunk_number = previous_row_dependencies[-1]

    @staticmethod
    def vertical_layers(model_object):
        #calculate model bounds and height
        model_bounds = SimulatorMath.calculate_bounds(model_object)
        z_height = model_bounds.z.max - model_bounds.z.min
        
        #calculate the minimum number of layers
        min_vertical_layers = math.ceil(z_height / (ChunkingParameters.max_reach_z-(AGParameters.min_AG_height*AGParameters.fit_multiplier)))
        layer_split_number = max(ChunkingParameters.vertical_check_density, min_vertical_layers)
        layer_height = z_height/(layer_split_number+1)
        
        #FOR TESTING
        print('Min Vertical Layers: '+ str(min_vertical_layers))
        
        # Make the object visible, selected, active, and ready for editing
        scene = bpy.context.scene
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object
        
        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active
        plane_no = Vector((0, 0, 1))
        remaining = new_object
        
        layer_islands_data = np.zeros((2,layer_split_number))
        
        # pdb.set_trace()
        
        #Split and loop through layers while recording the number of islands and islands without AG
        for i in range(1, layer_split_number+1):
            layer_location = layer_height*i
            plane_co = Vector((0, 0, layer_location))

            (layer_i, remaining) = Chunker.split_model(remaining, (plane_co, plane_no))
            
            (lower_ray_bound, location_grid_bottom, success_grid) = Chunker.ray_grid(remaining, model_bounds, Vector((0,0,1)), layer_location)
            (numeric_grid, x_index, y_index) = Chunker.interface_check(model_bounds, success_grid, AGParameters.min_AG_rad)
            (islands_grid, island_number, no_AG_island_number) = Chunker.generate_islands_grid(numeric_grid, x_index, y_index)
            
            layer_islands_data[0, i-1] = island_number
            layer_islands_data[1, i-1] = no_AG_island_number
            
        #Decide where to put layer boundaries by minimizing number of layers
        (num_vertical_layers, split_locations_array) = Chunker.layer_locations(layer_islands_data, layer_split_number,layer_height)
        
        #FOR TESTING
        # pdb.set_trace()
        
        return (num_vertical_layers, split_locations_array, z_height)
    
    @staticmethod
    def layer_locations(layer_islands_data, layer_split_number, layer_height):
        '''
        This method is to do an exhaustive search of all layer splits that could make up
        the vertical chunking for a job. It treats the set of layers as an integer (either
        0, corresponding to an inactive cut, or 1, corresponding to an active cut). It 
        treats the set of all cuts as a binary string of active and inactive cuts and 
        generates a set of the best solution (least amount of distinct parts, islands or
        vertical layers). Based on this set of lowest chunk number solutions, you can
        optimize in different ways, the first of which will be implemented is to have
        the chunk height be as equal as possible. 
        '''
        #FOR TESTING
        # pdb.set_trace()
        
        split_status = np.zeros([2**layer_split_number,layer_split_number])
        feasibility_array = np.ones(2**layer_split_number, dtype=bool)
        num_chunks_array = np.zeros([2**layer_split_number,2])
        for i in range(0,(2**layer_split_number)):
            #convert index to binary
            binary_string = bin(i)
            
            #assign 0 or 1 to each layer
            j = layer_split_number-1
            while j >= 0:
                num_digits_bin = (len(binary_string)-2)
                if layer_split_number - j <= num_digits_bin:
                    split_status[i,j] = binary_string[j - (layer_split_number-len(binary_string))]
                else:
                    break
                j -= 1
            
            #Check if combination feasible based on height
            current_height = layer_height
            for j in range(0, layer_split_number):
                if split_status[i,j] == 1:
                    current_height = 0
                current_height += layer_height
                
                if current_height  > ChunkingParameters.max_reach_z:
                    feasibility_array[i] = False
                    break
            
            #figure out number of islands at each configuration
            chunk_number = 0
            no_AG_chunk_number = 0
            if feasibility_array[i] == True:
                for j in range(0, layer_split_number):
                    chunk_number += split_status[i,j]*(1+layer_islands_data[0,j])
                    no_AG_chunk_number += split_status[i,j]*(layer_islands_data[1,j])
                num_chunks_array[i,0] = chunk_number
                num_chunks_array[i,1] = no_AG_chunk_number
            else:
                num_chunks_array[i,0] = 1000
                num_chunks_array[i,1] = 1000
        
        #FOR TESTING
        # pdb.set_trace()
        
        #Find all the locations with minimum number of islands
        min_row = np.amin(num_chunks_array, axis=0)
        min_val = min_row[0]
        min_locations = np.where(min_val == num_chunks_array) 
        min_locations_array = np.copy(min_locations)
        #these are the indexes of the combinations in num_chunks_array with fewest chunks
        number_min_locations = np.size(min_locations[0])
        
        #compare number of no_AG_islands among min set
        for i in range(0, number_min_locations):
            index_array = min_locations[0]
            min_locations_array[1,i] = num_chunks_array[index_array[0],1]
        min_row_no_AG = np.amin(min_locations_array, axis=0)
        min_val_no_AG = min_row_no_AG[0]
        min_locations_no_AG = np.where(min_val_no_AG == min_locations_array) 
        #these are the indexes in min_locations_array with fewest no AG islands
        min_no_AG_array = min_locations_no_AG[1]
        
        #Next optimize for even height of layers
        layer_diff_array = np.zeros(np.size(min_no_AG_array))
        for i in range(0, np.size(min_no_AG_array)):
            #get configuration from indexes
            config_index = min_locations_array[0,min_no_AG_array[i]]
            config = split_status[config_index]
            num_splits = np.sum(config)
            avg_layer_height = (layer_height*(layer_split_number+1))/(num_splits+1)
            
            #calculate cumulative deviation from average layer height in all layers
            current_height = layer_height
            layer_height_diff = 0
            for j in range(0, layer_split_number):
                if config[j] == 1:
                    layer_height_diff += abs(current_height-avg_layer_height)
                    current_height = 0
                current_height += layer_height
            layer_diff_array[i] = layer_height_diff
            
        #find minimum height and where that is
        min_layer_diff = np.min(layer_diff_array)
        min_layer_location = np.where(min_layer_diff == layer_diff_array)
        min_layer_index = min_layer_location[0]
        
        #find config associated with the ideal traits found above
        split_config_index = min_locations_array[0,min_no_AG_array[min_layer_index]]
        split_config = np.array(split_status[split_config_index])
        num_splits = int(np.sum(split_config))
        num_vertical_layers = num_splits+1
        split_locations_array = np.zeros(num_splits)
        split_count = 0
        for i in range(0,layer_split_number):
            if split_config[0,i] == 1:
                split_locations_array[split_count] = i*layer_height
                split_count += 1
        return(num_vertical_layers, split_locations_array)
    
    @staticmethod
    def ray_grid(model_object, model_bounds, direction, layer_location):
        '''
        This method uses raycasts to find the topology of an object in a give
        direction (up or down). It outputs a success grid of all the locations
        where the object is on the chunking boundary and a locations grid
        of the z position of successful raycasts (not the same as the success
        grid because success must be on chunk boundary while location does not)
        '''
        #FOR TESTING
        # pdb.set_trace()
        
        #Find bounds
        z_max = model_bounds.z.max
        z_min = model_bounds.z.min
        x_min = model_bounds.x.min
        x_max = model_bounds.x.max
        y_min = model_bounds.y.min
        y_max = model_bounds.y.max
        x_extreme = max(abs(x_min), abs(x_max))
        y_extreme = max(abs(y_min), abs(y_max))
        
        x_val = x_min
        y_val = y_min
        z_val = direction[2]*-10000
        
        #create grids
        x_size = (math.ceil(x_extreme*2/(AGParameters.grid_density)))
        y_size = (math.ceil(y_extreme*2/(AGParameters.grid_density)))
        location_grid = np.zeros((y_size,x_size))
        success_grid = np.zeros((y_size,x_size), dtype = bool)
        x_step_size = (x_extreme*2)/(x_size-1)
        y_step_size = (y_extreme*2)/(y_size-1)
        
        x_index = 0
        y_index = 0
        while x_val <= x_max:
            while y_val <= y_max:
                #FOR TESTING
                # pdb.set_trace()
                # print('x: ' +str(x_val))
                # print('y: ' +str(y_val))
                
                ray_origin = Vector((x_val,y_val,z_val))
                success, location, normal, val = model_object.ray_cast(ray_origin,direction)
                if success == True:
                    location_grid[y_index,x_index] = location[2]
                else:
                    if direction[2] > 0:
                        location_grid[y_index,x_index] = z_min
                    else:
                        location_grid[y_index,x_index] = z_max
                y_val += y_step_size
                y_index += 1
            y_index = 0
            y_val = y_min
            x_val += x_step_size
            x_index += 1
        
        #FOR TESTING
        # pdb.set_trace()
            
        if direction[2] < 0:
            extreme_location = np.max(location_grid)
        else:
            extreme_location = np.min(location_grid)
                
        for j in range(0,y_size):
            for i in range(0,x_size):
                if abs(location_grid[j,i] - layer_location) <= 1:
                    success_grid[j,i] = True
        
        #FOR TESTING
        # pdb.set_trace()
        
        return(extreme_location, location_grid, success_grid)
    
    @staticmethod
    def interface_check(model_bounds, top_success, rad):
        '''
        Checks a chunk interface (as given by a grid of True or False values)
        and returns potential AG locations as well as the number of unique XY 
        chunks or 'islands' and the number of 'islands' that aren't connected
        by AG's
        '''
        clearance = (rad*AGParameters.fit_multiplier)+AGParameters.offset
        
        #Import model Bounds
        y_max = model_bounds.y.max
        y_min = model_bounds.y.min
        x_max = model_bounds.x.max
        x_min = model_bounds.x.min
        x_extreme = max(abs(x_min), abs(x_max))
        y_extreme = max(abs(y_min), abs(y_max))
        
        #Find size of success grid and map to bounds to find coordinate of points
        x_index = np.size(top_success,1)
        y_index = np.size(top_success,0)
        x_step_size = abs(x_max-x_min)/(x_index-1)
        x_gap = math.ceil(clearance/x_step_size)
        y_step_size = abs(y_max-y_min)/(y_index-1)
        y_gap = math.ceil(clearance/y_step_size)
        
        #Make sure top and bottom success are true
        success = np.copy(top_success)
        # for i in range(0,x_index):
        #     for j in range(0,y_index):
        #         if top_success[j,i] != bottom_success[j,i]:
        #             success[j,i] = False
                
        #run through algorithm
        '''
        First, create a numeric grid to represent the interface, 0 means that 
        success at that location is false, -1 means it hasn't been checked yet, 1
        means suitable location for a minimum size AG, 2 means connected to a
        suitable AG location
        '''
        #create numeric grid
        numeric_grid = -np.ones((y_index,x_index))
        for i in range(0,x_index):
            for j in range(0,y_index):
                if success[j,i] == False:
                    numeric_grid[j,i] = 0
                    
        #Set start point and 
        y_count = 0
        x_count = 0
        
        #start loop to check for all viable AG positions
        while x_count < x_index:
            while y_count < y_index:
                
                y_pos = y_min + (y_count)*y_step_size
                x_pos = x_min + (x_count)*x_step_size
                #skip points too close to boundary
                if abs(abs(y_pos)-y_extreme) < clearance or abs(abs(x_pos)-x_extreme)< clearance:
                    Pass = False
                #only consider not yet checked points:
                elif numeric_grid[y_count,x_count] == -1:
                    # pdb.set_trace()
                    # check all neighboring points within bounding box of min AG
                    Pass = True
                    for i in range(-x_gap,x_gap+1):
                        for j in range(-y_gap,y_gap+1):
                            if numeric_grid[y_count+j,x_count+i] == 0:
                                Pass = False
                    # if all neighboring points are also good, AG location good
                    if Pass == True:
                        numeric_grid[y_count,x_count] = 1
                        
                    else:
                        numeric_grid[y_count,x_count] = -1
                #skip everything else
                else:
                    Pass = False
                y_count += 1
            y_count = 0
            x_count += 1

        # pdb.set_trace()    
        
        numeric_grid = Chunker.connections(numeric_grid, x_index, y_index,-1,-1, 1, 2, 2)
        return(numeric_grid, x_index, y_index) 
    
    @staticmethod
    def generate_islands_grid(numeric_grid,x_index,y_index):
        '''
        This method creates an island map, counts the number of islands
        as well as the number of islands without an AG
        '''
        islands_grid = -(numeric_grid)*10
        island_number = 1
        no_AG_island_number = 0
        
        #Go once over entire grid
        for x in range(0,x_index):
            for y in range(0,y_index):
                if islands_grid[y,x] == -10 or islands_grid[y,x] == -20:
                    islands_grid[y,x] = island_number
                    islands_grid = Chunker.connections(islands_grid, x_index, y_index, -10, -20, island_number, island_number, island_number)
                    island_number += 1
                elif islands_grid[y,x] == 10:
                    islands_grid[y,x] =  island_number
                    islands_grid = Chunker.connections(islands_grid, x_index, y_index, 10, 10, island_number, island_number, island_number)
                    island_number += 1
                    no_AG_island_number += 1
        #revert the last island number because none were actually created            
        island_number -= 1
        
        return(islands_grid, island_number, no_AG_island_number)
            
    @staticmethod
    def connections(numeric_grid,x_index,y_index,check_number_1, check_number_2, target_value_1, target_value_2, change_to):
        '''
        This method changes numeric grid to show if points are connected to a potential AG location. It takes 
        in a grid with -1 for unchecked, 0 for unsuccessful, and 1 for potential AG location, the x and y 
        size of the grid, as well as two numbers to check for neighbors (check_numbers), the numbersof the
        neighbor to check for (target_values) and the number to change to if the neighbors meet that
        criteria (change_to)
        '''    
        iterations = 0
        while np.min(numeric_grid) != 0 and iterations <= max(x_index,y_index):
            #create a duplicate to compare to. If it doesn't change between iterations, stop
            dup_numeric_grid = np.copy(numeric_grid)
            
            #first loop over the entire area
            for i in range (0,x_index):
                for j in range(0,y_index):
                    #check if the spot is either unchecked, or unsuitable for AG
                    if numeric_grid[j,i] == check_number_1 or numeric_grid[j,i] == check_number_2:
                        Connected = False
                        #loop over neighbors
                        for x in range(-1,2):
                            for y in range(-1,2):
                                #check if the neighbor is a suitable AG location or connected to one
                                #skip points out of bounds
                                if j+y >= y_index or j+y < 0 or i+x >= x_index or i+x < 0 :
                                    continue
                                else:    
                                    if numeric_grid[j+y,i+x] == target_value_1 or numeric_grid[j+y,i+x] == target_value_2:
                                        Connected = True
                                    else:
                                        continue
                                
                                if Connected == True:
                                    break
                            if Connected == True:
                                break
                            
                        if Connected == True:
                            numeric_grid[j,i] = change_to
                    
            #check if any change was made
            if np.array_equal(numeric_grid, dup_numeric_grid):
                iterations = max(x_index,y_index)+1
            else:
                iterations += 1
            
        return(numeric_grid)
    
    @staticmethod    
    def AG_locations(model_bounds, TS, min_rad, max_rad):
        '''
        The purpose of this function is to find what size and where to place AG on the
        interface. It will slowly loop up different radius sizes for the AG and 
        analyze the different islands or interfaces to find the best one for each island.
        Returns a 3xn array where n is the number of islands on the interface. the first 
        row corresponds to the radius, and the next two rows correspond to x and y 
        position respectively
        '''
        #Import model Bounds
        y_max = model_bounds.y.max
        y_min = model_bounds.y.min
        x_max = model_bounds.x.max
        x_min = model_bounds.x.min
        
        #Find size of success grid and map to bounds to find coordinate of points
        x_index = np.size(TS,1)
        y_index = np.size(TS,0)
        x_step_size = abs(x_max-x_min)/(x_index-1)
        y_step_size = abs(y_max-y_min)/(y_index-1)
        spacing = max(AGParameters.spacing, 2*max_rad + AGParameters.offset*2)
        x_spacing_index = int(math.ceil(spacing/x_step_size))
        y_spacing_index = int(math.ceil(spacing/y_step_size))
        
        #generate interface data at minimum radius
        (numeric_grid, x_index_1, y_index_1) = Chunker.interface_check(model_bounds,TS, min_rad)
        (islands_grid_min_rad, island_number_min_rad, no_AG_island_number) = Chunker.generate_islands_grid(numeric_grid, x_index, y_index)
        rad_array = [min_rad]
        dup_numeric_grid = np.copy(numeric_grid)
        AG_locations_array = np.zeros((3,island_number_min_rad))
        rad = min_rad
        island_number = island_number_min_rad
        #create check island grid at min rad
        for x in range(0,x_index):
            for y in range(0,y_index):
                #check if suitable location
                if numeric_grid[y,x] == 1:
                    #store radius size and position at successful location
                    island_index = int(islands_grid_min_rad[y,x]-1)
                    AG_locations_array[0, island_index] = rad
                    x_pos = x_min + x*x_step_size
                    AG_locations_array[1, island_index] = x_pos
                    y_pos = y_min + y*y_step_size
                    AG_locations_array[2, island_index] = y_pos
        rad += AGParameters.rad_inc
        
        #now loop over while increasing rad
        while island_number > 0 and rad <= max_rad:
            (numeric_grid, x_index_1, y_index_1) = Chunker.interface_check(model_bounds,TS, rad)
            (islands_grid, island_number, no_AG_island_number) = Chunker.generate_islands_grid(numeric_grid, x_index, y_index)
            rad_array.append(rad)
            #first check if the islands grid is any different
            if not np.array_equal(dup_numeric_grid, numeric_grid):
                #loop over interface
                for x in range(0,x_index):
                    for y in range(0,y_index):
                        #check if suitable location
                        if numeric_grid[y,x] == 1:
                            #store radius size and position at successful location
                            island_index = int(islands_grid_min_rad[y,x]-1)
                            AG_locations_array[0, island_index] = rad
                            x_pos = x_min + x*x_step_size
                            AG_locations_array[1, island_index] = x_pos
                            y_pos = y_min + y*y_step_size
                            AG_locations_array[2, island_index] = y_pos
            rad += AGParameters.rad_inc
            dup_numeric_grid = np.copy(numeric_grid)
        rad = np.max(rad_array)
        
        #if we get to the largest radius and still have more room, can add multiple AG/island
        island_AG_count = np.zeros(island_number_min_rad)
        if rad+AGParameters.rad_inc > max_rad:
            for x in range(0,x_index):
                for y in range(0,y_index):
                    #check if suitable location
                    if numeric_grid[y,x] == 1:
                        #set all other suitable locations within spacing to be unsuitable
                        for i in range(-x_spacing_index, x_spacing_index+1):
                            for j in range(-y_spacing_index, y_spacing_index+1):
                                #check if in bounds
                                if y+j <= y_index-1 and y+j >= 0 and x+i <= x_index-1 and x+i >= 0:
                                    #check suitability, if true, set to connecting.
                                    if numeric_grid[y+j,x+i] == 1:
                                        numeric_grid[y+j,x+i] = 2
                        #FOR TESTING
                        # pdb.set_trace()
                        
                        #store radius size and position at successful location
                        numeric_grid[y,x] = 1
                        island_index = int(islands_grid_min_rad[y,x]-1)
                        if island_AG_count[island_index] == 0:
                            AG_locations_array[0, island_index] = rad
                            x_pos = x_min + x*x_step_size
                            AG_locations_array[1, island_index] = x_pos
                            y_pos = y_min + y*y_step_size
                            AG_locations_array[2, island_index] = y_pos  
                            island_AG_count[island_index] += 1
                        else:
                            temp_array_1 = np.append(AG_locations_array[0],rad)
                            x_pos = x_min + x*x_step_size
                            temp_array_2 = np.append(AG_locations_array[1],x_pos)
                            y_pos = y_min + y*y_step_size
                            temp_array_3 = np.append(AG_locations_array[2],y_pos)
                            AG_locations_array = np.array([temp_array_1,temp_array_2,temp_array_3])
        return(AG_locations_array)

    @staticmethod
    def vertical_chunking(machines, model_object):

        Chunker.deselect_all_objects()

        scene = bpy.context.scene
        
        #Find bounds
        model_bounds = SimulatorMath.calculate_bounds(model_object)
        z_max = model_bounds.z.max
        z_min = model_bounds.z.min
        x_max = model_bounds.x.max
        x_min = model_bounds.x.min
        y_max = model_bounds.y.max
        y_min = model_bounds.y.min

        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object

        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active

        #Find the lower ray bound
        layer_location = 0
        (lower_ray_bound, lower_grid_bottom, lower_success_grid) = Chunker.ray_grid(model_object, model_bounds, Vector((0,0,1)), layer_location)
        
        (num_vertical_layers, split_locations_array, z_height) = Chunker.vertical_layers(model_object)
        
        #FOR TESTING
        # num_vertical_layers = 2
        # z_height = z_max-z_min
        # split_locations_array = np.array([185])
        
        
        
        print('Number of layers: ' + str(num_vertical_layers))
        chunker_result = ChunkerResult()
        plane_no = Vector((0, 0, 1))
        remaining = new_object
        
        
        #Check normalization
        normalize = False

        
        #FOR TESTING
        # pdb.set_trace()
        
        layer_number = 0
        total_height = z_max-z_min
        # AG_parameters = Vector((AGParameters.radius_base, AGParameters.height, AGParameters.spacing, AGParameters.offset,num_vertical_layers,layer_number,total_height,lower_ray_bound,AGParameters.fit_multiplier, layer_location))
        
        for i in range(0, num_vertical_layers-1):
            layer_number = i
            layer_location = split_locations_array[i]
            plane_co = Vector((0, 0, layer_location))

            (layer_i, remaining) = Chunker.split_model(remaining, (plane_co, plane_no))
            
            #Set layer location
            # if normalize == True:
            #     #this is the normalized height of this layer compared to chunk in range [-1,1]
            #     layer_location = (2/num_vertical_layers)*layer_number-1
            # else:
            #     layer_location = ((total_height/num_vertical_layers)*(layer_number+1))+(lower_ray_bound)
            # # add alignment geometry to top of split off chunk if requested
            # AG_parameters[5] = layer_number
            # AG_parameters[9] = layer_location
            
            #FOR TESTING
            print('Layer Number: '+ str(layer_number))
            # pdb.set_trace()
                
            if AGParameters.AG == True :
                #topology of chunk
                (lower_ray_bound, location_grid_bottom, success_grid) = Chunker.ray_grid(remaining, model_bounds, Vector((0,0,1)), layer_location)
                # (upper_ray_bound, location_grid_top, upper_success_grid) = Chunker.ray_grid(layer_i, model_bounds, Vector((0,0,-1)))
                
                #Find the AG locations and size
                (numeric_grid, x_index, y_index) = Chunker.interface_check(model_bounds, success_grid, AGParameters.min_AG_rad)
                (islands_grid, island_number, no_AG_island_number) = Chunker.generate_islands_grid(numeric_grid, x_index, y_index)
                AG_locations_array = Chunker.AG_locations(model_bounds, success_grid, AGParameters.min_AG_rad, AGParameters.max_AG_rad)
                 
                #place AG
                layer_i = Chunker.alignment_geometry_top(layer_i, AG_locations_array, layer_location)
                remaining = Chunker.alignment_geometry_bottom(remaining, AG_locations_array, layer_location)
            layer_i.name = "Layer " + str(i)
            chunker_result.add_bottom_layer(layer_i)
        remaining.name = "Layer " + str(i+1)
        chunker_result.add_bottom_layer(remaining)

        model_object.hide = True

        return chunker_result
    
    
    """
    This method is to add alignment geometry to the top of an object
    """
    @staticmethod
    def alignment_geometry_top(model, AG_locations_array, layer_location):
        #Prepare Scene
        scene = bpy.context.scene
        bpy.ops.object.select_all(action='DESELECT')
        model.select = True
        model.name = "model_name"
        scene.objects.active = model
        
        #FOR TESTING
        # pdb.set_trace()
        
        #Creat Aligment Geometries at successful locations
        for i in range(0,np.size(AG_locations_array[0])):
            if AG_locations_array[0,i] > 0:
                #read and generate geometry and location
                radius = AG_locations_array[0,i]
                height = 2*radius
                x_location = AG_locations_array[1,i]
                y_location = AG_locations_array[2,i]
                z_location = layer_location + height/2
                
                #create geometry
                bpy.ops.mesh.primitive_cone_add(radius1=radius, radius2=0, depth=height, location=(x_location, y_location, z_location))
                
                #join the AG to the model, select the part to attatch to and then join
                model.select = True
                bpy.ops.object.join()
                
                #tell blender that model is the combined object
                model = scene.objects.active
                model.name = "model_name"
        
        return(model)
        
    @staticmethod
    def alignment_geometry_bottom(model, AG_locations_array, layer_location):
        #Prepare Scene
        scene = bpy.context.scene
        bpy.ops.object.select_all(action='DESELECT')
        model.select = True
        model.name = "model_name"
        scene.objects.active = model
        
        #Creat Aligment Geometries at successful locations
        for i in range(0,np.size(AG_locations_array[0])):
            if AG_locations_array[0,i] > 0:
                #read and generate geometry and location
                radius = AG_locations_array[0,i]*AGParameters.fit_multiplier
                height = 2*radius*AGParameters.fit_multiplier
                x_location = AG_locations_array[1,i]
                y_location = AG_locations_array[2,i]
                z_location = layer_location + height/2
                
                #create geometry
                bpy.ops.mesh.primitive_cone_add(radius1=radius, radius2=0, depth=height, location=(x_location, y_location, z_location))
                AG = scene.objects.active
                AG.name = "AG_name"
                AG.select = False
                
                #use boolean subtract to subtract the AG from the model
                scene.objects.active = model
                model.select = True
                bpy.ops.object.modifier_add(type='BOOLEAN')
                bpy.context.object.modifiers["Boolean"].operation = 'DIFFERENCE'
                bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["AG_name"]
                bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Boolean")
                
                #Delete the AG model after it has been subtracted
                model.select = False
                AG.select = True
                bpy.ops.object.delete(use_global=False)
        
        return(model)
        

    """
    start_scaled_layers() implements vertical chunking if vertical_chunking() return vertical chunks
    if height of the desired part is smaller than the designated height, the function reverts back to the normal 
    start_scaled() method
    """

    @staticmethod
    def start_scaled_layers(machines, model_object):
        if len(machines) == 1:
            Chunker.start_single(machines[0], model_object)

        colors = [(255, 10, 27), (0, 30, 83), (255, 255, 255)]

        color_index = 0
        machine_parameters = machines[0].machine_parameters
        machine_width = machine_parameters.machine_width
        printhead_slope = machine_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        wing_slope = 50.00
        z_height = model_bounds.z.max - model_bounds.z.min
        # h = wing_slope * math.tan(printhead_slope)

        if z_height <= ChunkingParameters.max_reach_z:
            return Chunker.start_scaled(machines, model_object)

        max_chunk_num = 0

        layers_division = Chunker.vertical_chunking(machines, model_object)

        for (j, layers) in enumerate(layers_division.bottom_layer):
            layer_mult = (j + 1) * 10
            # chunker_result = Chunker.start(machine_parameters, layers, number_of_machines=2)
            # chunk_row = Chunker.split(chunker_result.origin_chunk,
            #                           model_width,
            #                           machine_width,
            #                           printhead_slope,
            #                           num_pieces=-1)
            # chunk_number = len(chunk_row) - 1 + layer_mult
            # for (i, chunk) in enumerate(chunk_row):
            new_chunk = Chunk(layers, number=layer_mult)
            new_chunk.color = colors[color_index]

            machine_number = j- (j % 2)
            machines[machine_number].chunks.append(new_chunk)

            new_chunk.set_name("Machine {} Chunk {}".format(machine_number, layer_mult))

            #     if i % 2 == 1:
            #         new_chunk.add_dependency(i - 1)
            #         if (i + 1) < len(chunk_row):
            #             new_chunk.add_dependency(i + 1)
            #     if j > 0:
            #         new_chunk.add_dependency(max_chunk_num)

            # center_chunk_dependencies = list(range(0 + layer_mult, len(chunk_row) + layer_mult))
            # previous_row_dependencies = list(range(0 + layer_mult, len(chunk_row) + layer_mult))
            # last_chunk_number = center_chunk_dependencies[-1]

            # row_number = 0

            # color_index = 1

            # for south_chunk in chunker_result.south_chunks:
            #     row_number += 1

            #     chunk_row = Chunker.split(south_chunk,
            #                               model_width,
            #                               machine_width,
            #                               printhead_slope,
            #                               num_pieces=-1)

            #     next_row_dependencies = []

            #     for (i, chunk) in enumerate(chunk_row):
            #         chunk_number = i + last_chunk_number + 1
            #         next_row_dependencies.append(chunk_number)
            #         new_chunk = Chunk(chunk, number=chunk_number)
            #         new_chunk.row = -1 * row_number
            #         new_chunk.color = colors[color_index]
            #         new_chunk.add_dependency(previous_row_depend
            
            

            #         new_chunk.set_name("Machine {} Chunk {}".format(machine_number, chunk_number))

            #         if i % 2 == 1:
            #             new_chunk.add_dependency(chunk_number - 1)
            #             if (i + 1) < len(chunk_row):
            #                 new_chunk.add_dependency(chunk_number + 1)

            #     previous_row_dependencies = next_row_dependencies
            #     next_row_dependencies = []
            #     last_chunk_number = previous_row_dependencies[-1]

            #     color_index += 1
            #     if color_index > 2:
            #         color_index = 0

            # previous_row_dependencies = center_chunk_dependencies

            # row_number = 0

            # color_index = 2

            # for north_chunk in chunker_result.north_chunks:
            #     row_number += 1
            #     chunk_row = Chunker.split(north_chunk,
            #                               model_width,
            #                               machine_width,
            #                               printhead_slope,
            #                               num_pieces=-1)
            #     next_row_dependencies = []

            #     for (i, chunk) in enumerate(chunk_row):
            #         chunk_number = i + last_chunk_number + 1
            #         next_row_dependencies.append(chunk_number)
            #         new_chunk = Chunk(chunk, number=chunk_number)
            #         new_chunk.row = row_number
            #         new_chunk.color = colors[color_index]
            #         new_chunk.add_dependency(previous_row_dependencies)

            #         machine_number = i - (i % 2)
            #         if machine_number >= len(machines):
            #             machine_number = machine_number % len(machines)
            #         machines[machine_number].chunks.append(new_chunk)

            #         new_chunk.set_name("Machine {} Chunk {}".format(machine_number, chunk_number))

            #         if i % 2 == 1:
            #             new_chunk.add_dependency(chunk_number - 1)
            #             if (i + 1) < len(chunk_row):
            #                 new_chunk.add_dependency(chunk_number + 1)

            #     previous_row_dependencies = next_row_dependencies
            #     next_row_dependencies = []
            #     last_chunk_number = previous_row_dependencies[-1]

            #     max_chunk_num = last_chunk_number

            #     color_index -= 1
            #     if color_index < 0:
            #         color_index = 2

    @staticmethod
    def start_scaled(machines, model_object):

        if len(machines) == 1:
            Chunker.start_single(machines[0], model_object)
            return

        color = (0, 0, 255)
        swap_color = (255, 0, 0)

        machine_parameters = machines[0].machine_parameters
        machine_width = machine_parameters.machine_width
        printhead_slope = machine_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        chunker_result = Chunker.start(machine_parameters, model_object, number_of_machines=2)
        chunk_row = Chunker.split(chunker_result.origin_chunk, model_width, machine_width, printhead_slope,
                                  num_pieces=-1)

        chunk_number = len(chunk_row)
        mid_point = math.ceil(chunk_number/2)

        for i in range(0, len(chunk_row)):
            new_chunk = Chunk(chunk_row[i], number=i)
            new_chunk.color = color
            color, swap_color = swap_color, color

            if len(machines) == 2:
                if i < mid_point:
                    machine_number = 0
                else:
                    machine_number = 1
            else:

                machine_number = i
                if machine_number >= len(machines):
                    machine_number = machine_number % len(machines)

            machines[machine_number].chunks.append(new_chunk)

            new_chunk.set_name("Machine {} Chunk {}".format(machine_number, i))

            if i % 2 == 1:
                new_chunk.add_dependency(i - 1)
                if (i + 1) < len(chunk_row):
                    new_chunk.add_dependency(i + 1)

        center_chunk_dependencies = list(range(0, len(chunk_row)))
        previous_row_dependencies = list(range(0, len(chunk_row)))
        last_chunk_number = center_chunk_dependencies[-1]

        row_number = 0

        for south_chunk in chunker_result.south_chunks:
            color, swap_color = swap_color, color
            row_number += 1

            chunk_row = Chunker.split(south_chunk, model_width, machine_width, printhead_slope,
                                      num_pieces=-1)

            next_row_dependencies = []

            for i in range(0, len(chunk_row)):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk_row[i], number=chunk_number)
                new_chunk.row = -1 * row_number
                new_chunk.color = color
                color, swap_color = swap_color, color
                new_chunk.add_dependency(previous_row_dependencies)

                machine_number = i - (i % 2) + 1
                if machine_number >= len(machines):
                    machine_number = machine_number % len(machines)
                machines[machine_number].chunks.append(new_chunk)

                new_chunk.set_name("Machine {} Chunk {}".format(machine_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

        previous_row_dependencies = center_chunk_dependencies

        row_number = 0

        for north_chunk in chunker_result.north_chunks:
            color, swap_color = swap_color, color
            row_number += 1
            chunk_row = Chunker.split(north_chunk, model_width, machine_width, printhead_slope,
                                      num_pieces=-1)
            next_row_dependencies = []

            for i in range(0, len(chunk_row)):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk_row[i], number=chunk_number)
                new_chunk.row = row_number
                new_chunk.color = color
                color, swap_color = swap_color, color
                new_chunk.add_dependency(previous_row_dependencies)

                machine_number = i - (i % 2)
                if machine_number >= len(machines):
                    machine_number = machine_number % len(machines)
                machines[machine_number].chunks.append(new_chunk)

                new_chunk.set_name("Machine {} Chunk {}".format(machine_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

    """
    vertical_chunking() does the vertical chunking based on given wing slope. Height is calculated using the given wingslope
    It outputs the vertical chunks which is then inputed in the scaled_layer_chunking()

    """

    @staticmethod
    def start_scaled_oneSided(machines, model_object):
        """
        Performs SPAR3 chunking on ``model_object``, using ``robots``.
        \n
        In the cases where ``len(robots) < 4``, either single-robot chunking or two-robot
        chunking is used. Otherwise, full SPAR3 chunking is used.
        \n
        Assuming a proper two-robot chunk, the SPAR3 chunking process simply subdivides every chunk
        in the two-robot chunk lengthwise, and in the exact same way for every chunk. This method
        also takes care of assigning chunk dependencies and assigning chunks to the proper robot.

        :param robots: the robots that will be used to print this model
        :type robots: List[Robot]
        :param model_object: the model to be printed
        :type model_object: bpy.types.Object
        """
        if len(machines) == 1:
            Chunker.start_single(machines[0], model_object)

        if len(machines) == 2:
            Chunker.start_2_robots_sameside(machines, model_object, 2)
            return

        color = (0, 0, 255)
        swap_color = (255, 0, 0)

        robot_parameters = machines[0].machine_parameters
        robot_width = robot_parameters.machine_width
        printhead_slope = robot_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        chunker_result = Chunker.start_oneSided(machines, model_object, number_of_machines=2)

        chunk_row = Chunker.split(chunker_result.origin_chunk,
                                  model_width,
                                  robot_width,
                                  printhead_slope,
                                  num_pieces=len(machines))

        chunk_number = len(chunk_row) - 1
        for (i, chunk) in enumerate(chunk_row):
            new_chunk = Chunk(chunk, number=i)
            new_chunk.color = color
            color, swap_color = swap_color, color

            # robot_number = i - (i % 2)
            robot_number = i
            machines[robot_number].chunks.append(new_chunk)

            new_chunk.set_name("Machine {} Chunk {}".format(robot_number, i))

            if i % 2 == 1:
                new_chunk.add_dependency(i - 1)
                if (i + 1) < len(chunk_row):
                    new_chunk.add_dependency(i + 1)

        center_chunk_dependencies = list(range(0, len(chunk_row)))
        previous_row_dependencies = list(range(0, len(chunk_row)))
        last_chunk_number = center_chunk_dependencies[-1]

        row_number = 0

        for south_chunk in chunker_result.south_chunks:
            color, swap_color = swap_color, color
            row_number += 1
            chunk_row = Chunker.split(south_chunk,
                                      model_width,
                                      robot_width,
                                      printhead_slope,
                                      num_pieces=-1)
            next_row_dependencies = []

            for (i, chunk) in enumerate(chunk_row):
                chunk_number = i + last_chunk_number + 1
                next_row_dependencies.append(chunk_number)
                new_chunk = Chunk(chunk, number=chunk_number)
                new_chunk.row = row_number
                new_chunk.color = color
                color, swap_color = swap_color, color
                new_chunk.add_dependency(previous_row_dependencies)

                robot_number = i
                machines[robot_number].chunks.append(new_chunk)

                new_chunk.set_name("Machine {} Chunk {}".format(robot_number, chunk_number))

                if i % 2 == 1:
                    new_chunk.add_dependency(chunk_number - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(chunk_number + 1)

            previous_row_dependencies = next_row_dependencies
            next_row_dependencies = []
            last_chunk_number = previous_row_dependencies[-1]

    @staticmethod
    def start_scaled_layers_sameside(machines, model_object):
        if len(machines) == 1:
            Chunker.start_single(machines[0], model_object)

        colors = [(255, 10, 27), (0, 30, 83), (255, 255, 255)]

        color_index = 0
        robot_parameters = machines[0].machine_parameters
        robot_width = robot_parameters.width
        printhead_slope = robot_parameters.printhead_slope

        model_bounds = SimulatorMath.calculate_bounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        wing_slope = 50.0
        z_height = model_bounds.z.max - model_bounds.z.min
        h = wing_slope * math.tan(printhead_slope)

        if z_height <= h:
            return Chunker.start_scaled_oneSided(machines, model_object)

        max_chunk_num = 0

        layers_division = Chunker.vertical_chunking(machines, model_object)

        for (j, layers) in enumerate(layers_division.bottom_layer):
            layer_mult = (j + 1) * 10
            chunker_result = Chunker.start_oneSided(machines, layers, number_of_machines=2)
            chunk_row = Chunker.split(chunker_result.origin_chunk,
                                      model_width,
                                      robot_width,
                                      printhead_slope,
                                      num_pieces=len(machines))
            chunk_number = len(chunk_row) - 1 + layer_mult
            for (i, chunk) in enumerate(chunk_row):
                new_chunk = Chunk(chunk, number=i + layer_mult)
                new_chunk.color = colors[color_index]

                robot_number = i - (i % 2) + 1
                machines[robot_number].chunks.append(new_chunk)

                new_chunk.set_name("Machine {} Chunk {}".format(robot_number, i + layer_mult))

                if i % 2 == 1:
                    new_chunk.add_dependency(i - 1 + layer_mult)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(i + 1 + layer_mult)
                if j > 0:
                    new_chunk.add_dependency(max_chunk_num)

            center_chunk_dependencies = list(range(0 + layer_mult, len(chunk_row) + layer_mult))
            previous_row_dependencies = list(range(0 + layer_mult, len(chunk_row) + layer_mult))
            last_chunk_number = center_chunk_dependencies[-1]

            row_number = 0

            color_index = 1

            for north_chunk in chunker_result.north_chunks:
                row_number += 1
                chunk_row = Chunker.split(north_chunk,
                                          model_width,
                                          robot_width,
                                          printhead_slope,
                                          num_pieces=len(robots))
                next_row_dependencies = []

                for (i, chunk) in enumerate(chunk_row):
                    chunk_number = i + last_chunk_number + 1
                    color_index -= 1
                    next_row_dependencies.append(chunk_number)
                    new_chunk = Chunk(chunk, number=chunk_number)
                    new_chunk.row = row_number
                    new_chunk.color = colors[color_index]
                    new_chunk.add_dependency(previous_row_dependencies)

                    robot_number = i - (i % 2) + 1
                    machines[robot_number].chunks.append(new_chunk)

                    new_chunk.set_name("Machine {} Chunk {}".format(robot_number, chunk_number))

                    if i % 2 == 1:
                        new_chunk.add_dependency(chunk_number - 1)
                        if (i + 1) < len(chunk_row):
                            new_chunk.add_dependency(chunk_number + 1)

                    if color_index < 0:
                        color_index = 2

                previous_row_dependencies = next_row_dependencies
                next_row_dependencies = []
                last_chunk_number = previous_row_dependencies[-1]

                max_chunk_num = last_chunk_number

                color_index -= 1
                if color_index < 0:
                    color_index = 2

    """
    split_model() splits a blender object around a plane, specified by plane_data.
    plane_data should be a tuple of (plane_coordinate, plane_normal)

    Returns a tuple of the two chunks produced from the split. The first is on the
    Negative side of the plane, the second is the positive side of the plane.
    """

    @staticmethod
    def split_model(model, plane_data):
        global counter_value_f
        scene = bpy.context.scene
        Chunker.deselect_all_objects()
        model.select = True
        scene.objects.active = model

        plane_co = plane_data[0]
        plane_no = plane_data[1]

        bpy.ops.object.duplicate()

        duplicate = bpy.context.object
        if duplicate == None:
            return (None, None)

        duplicate.name = "Temp Model Chunk"

        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.bisect(
            plane_co=plane_co,
            plane_no=plane_no,
            use_fill=True,
            clear_inner=True
        )
        bpy.ops.object.mode_set(mode='OBJECT')

        temp_bmesh = bmesh.new()
        temp_bmesh.from_mesh(duplicate.data)

        bmesh.ops.triangulate(temp_bmesh, faces=temp_bmesh.faces[:], quad_method=0, ngon_method=0)

        temp_bmesh.to_mesh(duplicate.data)
        temp_bmesh.free()

        duplicate.select = False
        model.select = True
        scene.objects.active = model

        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.bisect(
            plane_co=plane_co,
            plane_no=plane_no,
            use_fill=True,
            clear_outer=True
        )
        bpy.ops.object.mode_set(mode='OBJECT')

        temp_bmesh = bmesh.new()
        temp_bmesh.from_mesh(model.data)

        bmesh.ops.triangulate(temp_bmesh, faces=temp_bmesh.faces[:], quad_method=0, ngon_method=0)

        temp_bmesh.to_mesh(model.data)
        temp_bmesh.free()

        return (model, duplicate)

    # @staticmethod
    # def split(chunk, object_width, machine_width, printhead_slope, num_pieces=-1):
    #     if num_pieces == 2:
    #         return [chunk]
    #
    #     chunk_bounds = SimulatorMath.calculate_bounds(chunk)
    #
    #     normal_z = math.tan(math.pi / 2 - printhead_slope)
    #
    #     height = chunk_bounds.z.max - chunk_bounds.z.min
    #     slope_width = 50
    #
    #     mid_y = (chunk_bounds.y.min + chunk_bounds.y.max) / 2
    #
    #     plane_normal_east = Vector((1, 0, normal_z))
    #     plane_normal_west = Vector((-1, 0, normal_z))
    #
    #     width = object_width
    #     half_width = width / 2
    #     if num_pieces != -1:
    #         chunks = num_pieces // 2
    #     else:
    #         chunks = math.ceil(half_width / machine_width)
    #
    #     chunk_row = []
    #     facing_east = True
    #     #remaining_chunk = chunk
    #
    #     #piece_width = max(machine_width, width / num_pieces)
    #
    #     piece_width = 150.00
    #     mid_x = (chunk_bounds.x.min + chunk_bounds.x.max) / 2
    #     plane_co = Vector((mid_x, mid_y, 0))
    #     plane_co_west = Vector((mid_x + 150, mid_y, 0))
    #
    #     (origin_chunk, east_piece) = Chunker.split_model(chunk, (plane_co, plane_normal_east))
    #     (origin_chunk, west_piece) = Chunker.split_model(origin_chunk, (plane_co_west, plane_normal_west))
    #
    #     chunk_row.append(origin_chunk)
    #
    #     east_x = mid_x - piece_width
    #
    #     remaining_chunk = east_piece
    #     while east_x < chunk_bounds.x.max:
    #         plane_co = Vector((east_x, mid_y, 0))
    #
    #         if facing_east:
    #             plane_no = plane_normal_east
    #         else:
    #             plane_no = plane_normal_west
    #
    #         (west, east) = Chunker.split_model(remaining_chunk, (plane_co, plane_no))
    #
    #         if facing_east:
    #             if not west.data.polygons:
    #                 chunk_row.append(Vector((plane_co[0], mid_y, 0)))
    #             else:
    #                 chunk_row.append(west)
    #             remaining_chunk = east
    #         else:
    #             if not east.data.polygons:
    #                 chunk_row.append(Vector((plane_co[0], mid_y, 0)))
    #             else:
    #                 chunk_row.append(east)
    #             remaining_chunk = west
    #
    #         facing_east = not facing_east
    #         east_x -= piece_width
    #     chunk_row.append(remaining_chunk)
    #
    #     west_x = plane_co_west[0] + piece_width
    #
    #     remaining_chunk = west_piece
    #
    #     while west_x > chunk_bounds.x.max:
    #         plane_co = Vector((west_x, mid_y, 0))
    #
    #         if facing_east:
    #             plane_no = plane_normal_east
    #         else:
    #             plane_no = plane_normal_west
    #
    #         (west, east) = Chunker.split_model(remaining_chunk, (plane_co, plane_no))
    #
    #         if facing_east:
    #             if not west.data.polygons:
    #                 chunk_row.append(Vector((plane_co[0], mid_y, 0)))
    #             else:
    #                 chunk_row.append(west)
    #             remaining_chunk = east
    #         else:
    #             if not east.data.polygons:
    #                 chunk_row.append(Vector((plane_co[0], mid_y, 0)))
    #             else:
    #                 chunk_row.append(east)
    #             remaining_chunk = west
    #
    #         facing_east = not facing_east
    #         west_x += piece_width
    #
    #     chunk_row.append(remaining_chunk)
    #
    #     return chunk_row



    '''
    The code below is to make the temporary test bed work. 
    The values are hardcoded to they can be easily changed as desired.
    For normal code implemention, comment out the code below and uncomment the code above
    '''
    @staticmethod
    def split(chunk, object_width, machine_width, printhead_slope, num_pieces=-1):
        if num_pieces == 2:
            return [chunk]

        chunk_bounds = SimulatorMath.calculate_bounds(chunk)

        normal_z = math.tan(math.pi / 2 - printhead_slope)
        alpha = ChunkingParameters.slope_y_dir
        angled_plane = math.tan(alpha)
        max_reach = ChunkingParameters.max_reach_x
        starting_point = ChunkingParameters.starting_point
        build_plate_x = ChunkingParameters.build_plate_x
        mount_location_x = build_plate_x / 2

        mid_y = (chunk_bounds.y.min + chunk_bounds.y.max) / 2
        z_min = chunk_bounds.z.min
        x_min = chunk_bounds.x.min
        height = chunk_bounds.z.max - chunk_bounds.z.min
        plane_normal_east = Vector((1, 0, normal_z))
        plane_normal_west = Vector((-1, angled_plane, normal_z))

        width = object_width
        slope_width = height * normal_z
        chunk_width = ChunkingParameters.rest

        if num_pieces != -1:
            chunks = num_pieces // 2
        else:
            chunks = math.ceil(width/chunk_width)

        chunk_row = []
        facing_east = True
        remaining_chunk = chunk

        piece_width = min(max_reach, chunk_width)

        for i in range(1, chunks):

            plane_co = Vector((x_min + (i * piece_width), mid_y, z_min))

            if facing_east:
                plane_no = plane_normal_east
            else:
                plane_no = plane_normal_west

            (west, east) = Chunker.split_model(remaining_chunk, (plane_co, plane_no))

            if facing_east:
                if not west.data.polygons:
                    chunk_row.append(Vector((plane_co[0], mid_y, z_min)))
                else:
                    chunk_row.append(west)
                remaining_chunk = east

            else:
                if not east.data.polygons:
                    chunk_row.append(Vector((plane_co[0] - slope_width, mid_y, z_min)))

                else:
                    chunk_row.append(east)
                remaining_chunk = west

            facing_east = not facing_east

        if not remaining_chunk.data.polygons:
            chunk_row.append(Vector((chunks * piece_width, mid_y, z_min)))
        else:
            chunk_row.append(remaining_chunk)

        return chunk_row






