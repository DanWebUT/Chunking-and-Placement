import math
import bpy
import bmesh
import pdb

from am3.util.simulatormath import *
from am3.model import Chunk

counter_value_f = 0


class ChunkingParameters:

    build_plate_x = 300
    build_plate_y = 300
    starting_point = 50.0
    slope_y_dir = 2.53073  # 145 degrees
    max_reach_x = 425.00
    rest = 300.00


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

    #Simple recentering method. Uses z_min and shifts the object up that amount
    #only recenters in z direction (for now), should work for above and below z = 0
    @staticmethod
    def recenter_object(model_object):
        model_bounds = SimulatorMath.calculateBounds(model_object)
        
        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene = bpy.context.scene
        scene.objects.active = model_object
            
        min_x = model_bounds.x.min
        max_x = model_bounds.x.max
        min_y = model_bounds.y.min
        max_y = model_bounds.y.max
    
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
    
        model_object.location[0] -= center_x
        model_object.location[1] -= center_y
        model_object.location[2] -= model_bounds.z.min
            
            


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

        model_bounds = SimulatorMath.calculateBounds(model_object)

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
        model_bounds = SimulatorMath.calculateBounds(model_object)

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

        #adding in an escape sequence if the while statement below is not true to begin with
        if plane_co[1] <= model_bounds.y.min - slope_width:
            machine.chunks.append(model_object)
            print("No chunking required")
            return
            

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
        # build_depth = machine_parameters.build_depth
        build_depth = 350

        chunk_depth = ChunkingParameters.build_plate_y

        model_bounds = SimulatorMath.calculateBounds(model_object)

        model_depth = model_bounds.y.max - model_bounds.y.min

        if model_depth < build_depth:
            origin_chunk = new_object
            chunker_result.set_origin_chunk(origin_chunk)
            return chunker_result

        # angle_of_elevation = math.pi / 2 - machine_parameters.printhead_slope
        angle_of_elevation = math.pi / 2 - (60 * 3.14159 / 180)
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

        model_bounds = SimulatorMath.calculateBounds(model_object)
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
    def vertical_layers(machines, model_object):

        # wing_slope = data["constants"]["border_chunk_slopewidth"]
        wing_slope = 50.0  # results in height of 28.866 mm
        model_bounds = SimulatorMath.calculateBounds(model_object)
        machine_parameters = machines[0].machine_parameters
        printhead_slope = machine_parameters.printhead_slope
        z_height = model_bounds.z.max - model_bounds.z.min
        #h = wing_slope * math.tan(printhead_slope)
        h = 250 #changed h to 250mm, maybe add this as machine parameter
        num_vertical_layers = math.ceil(z_height / h)
        height_vertical_layers = z_height/num_vertical_layers

        return (num_vertical_layers, height_vertical_layers, z_height)

    @staticmethod
    def vertical_chunking(machines, model_object):
        Chunker.deselect_all_objects()
        scene = bpy.context.scene
        
        #Find bounds
        model_bounds = SimulatorMath.calculateBounds(model_object)
        z_max = model_bounds.z.max
        z_min = model_bounds.z.min
        x_min = model_bounds.x.min
        x_max = model_bounds.x.max
        y_min = model_bounds.y.min
        y_max = model_bounds.y.max

        #Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object

        # Duplicate the object so we don't overwrite the original model
        bpy.ops.object.duplicate()
        new_object = scene.objects.active

        (num_vertical_layers, height_vertical_layers, z_height) = Chunker.vertical_layers(machines, model_object)
        print('Number of layers: ' + str(num_vertical_layers))
        chunker_result = ChunkerResult()
        plane_no = Vector((0, 0, 1))
        layers = []
        remaining = new_object
        
        #find lower ray bound. Used later to find location of layer in raycast coordinates
        lower_ray_bound = 0
        ray_direction = Vector((0,0,1))
        x_val = x_min
        y_val = y_min
        while x_val <= x_max:
            while y_val <= y_max:
                ray_origin = Vector((x_val,y_val,-10000))
                success, location, normal, val = model_object.ray_cast(ray_origin,ray_direction)
                if success == True:
                    temp_lower_ray_bound = location[2]
                    if temp_lower_ray_bound < lower_ray_bound:
                        lower_ray_bound = temp_lower_ray_bound
                y_val += 1
            y_val = y_min
            x_val += 1
            
        # #For Testing
        # print('Lower_Ray_Bound: '+str(lower_ray_bound))
        
        # Set alignment Geometry Parameters
        AG = False #set True to add Alignment Geometry
        diameter = 10 #mm
        radius_base = diameter/2
        height = 10 #mm
        spacing = 300 #mm
        offset = 1 #mm distance AG should be from edge
        layer_number = 0
        total_height = z_max-z_min
        AG_parameters = Vector((radius_base, height, spacing, offset,num_vertical_layers,layer_number,total_height,lower_ray_bound))
        min_AG_number = 5
        
        for i in range(1, num_vertical_layers):
            AG_parameters[5] = i
            
            z_max = min(height_vertical_layers * i, z_height)
            plane_co = Vector((0, 0, z_max))

            (layer_i, remaining) = Chunker.split_model(remaining, (plane_co, plane_no))
            
            # add alignment geometry to top of split off chunk if requested
            AG_parameters_layer = AG_parameters
                
            if AG == True :
                #duplicate chunk
                """
                The purpose of duplicating the chunk is to allow for the alignment geometries to be
                placed on the duplicate chunk as a test. This allows for the spacing and size of AG's 
                to be tuned by the algorithm before the original chunk is modified.
                Update: with the double loop setup in the check phase, one could likely update this to
                not need to test placing alignment geometry on a test chunk and instead just count the
                number of Trues in the grid_success list.
                """
                bpy.ops.object.select_all(action='DESELECT')
                layer_i.select = True
                bpy.ops.object.duplicate()
                test_duplicate = bpy.context.object
                test_duplicate.name = "Test Chunk"   
                
                AG_Count, test_duplicate = Chunker.alignment_geometry_top(AG_parameters_layer, test_duplicate)
                
                #delete duplicate
                bpy.ops.object.select_all(action='DESELECT')
                test_duplicate.select = True
                bpy.ops.object.delete(use_global=False)
                    
                
                #Make sure there are at least 3 AG's present
                """
                This section loops until the number of AG is greater than or equal to the user set
                minimum AG number. It has a break after 200 iterations to prevent infinite loops where AG 
                are not possible. It works by incrementally lowering the spacing between AG's until the d
                desired number is possible. If spacing becomes so small that AG's would collide, it resets
                the spacing to the original value and makes the AG's 10% smaller and tries again.
                """
                iteration_count = 0
                if AG_Count < min_AG_number:
                    while AG_Count < min_AG_number:
                        if iteration_count >1000:
                            break
                        
                        bpy.ops.object.select_all(action='DESELECT')
                        layer_i.select = True
                        scene.objects.active = layer_i
                        bpy.ops.object.duplicate()
                        duplicate = bpy.context.object
                        duplicate.name = "Temp Chunk"  
                        
                        #AG_gap is the distance between AG's and should be at least 10mm
                        #it should be smaller than the spacing
                        AG_gap = AG_parameters_layer[1]*2 +10 
                        if AG_gap <= AG_parameters_layer[2]-10:
                            AG_parameters_layer[2] -= 1
                            # print('Spacing Too large')
                            # print('Changing Spacing to ' + str(AG_parameters_layer[2]))
                        else:
                            #if AG_gap is not larger than spacing, reset spacing
                            #and lower size of AG by 10%
                            AG_parameters_layer[2] = AG_parameters[2]
                            AG_parameters_layer[0] = AG_parameters_layer[0]*.9
                            AG_parameters_layer[1] = AG_parameters_layer[1]*.9
                            # print('AG Size Too Large')
                            # print('Changing AG Radius to ' + str(AG_parameters_layer[0]))
                   
                    
                        AG_Count, duplicate = Chunker.alignment_geometry_top(AG_parameters_layer, duplicate) 
                        
                        #delete duplicate
                        bpy.ops.object.select_all(action='DESELECT')
                        duplicate.select = True
                        bpy.ops.object.delete(use_global=False)
                        
                        
                        iteration_count += 1
                        
                AG_Count, layer_i = Chunker.alignment_geometry_top(AG_parameters_layer, layer_i)
                    
                remaining = Chunker.alignment_geometry_bottom(AG_parameters_layer, remaining)
                

                
        
            chunker_result.add_bottom_layer(layer_i)
                
        chunker_result.add_bottom_layer(remaining)

        model_object.hide = True

        return chunker_result

    """
    start_scaled_layers() implements vertical chunking if vertical_chunking() return vertical chunks
    if height of the desired part is smaller than the designated height, the function reverts back to the normal 
    start_scaled() method
    """

    """
    This method is to add alignment geometry to the top of an object
    """
    @staticmethod
    def alignment_geometry_top(AG_parameters, model):
        #Prepare Scene
        scene = bpy.context.scene
        bpy.ops.object.select_all(action='DESELECT')
        model.select = True
        model.name = "model_name"
        scene.objects.active = model
        
        #set size and spacing variables
        radius_base = AG_parameters[0]
        height = AG_parameters[1]
        spacing = AG_parameters[2]
        offset = AG_parameters[3]
        num_vertical_layers = AG_parameters[4]
        layer_number = AG_parameters[5]
        total_height = AG_parameters[6]
        lower_ray_bound = AG_parameters[7]
        
        #Find bounds and calculate height and width based on them
        model_bounds = SimulatorMath.calculateBounds(model)
        z_max = model_bounds.z.max
        z_min = model_bounds.z.min
        z_location = z_max+height/2
        x_min = model_bounds.x.min
        x_max = model_bounds.x.max
        x_extreme = max(abs(x_min), abs(x_max))
        x_location = 0
        y_min = model_bounds.y.min
        y_max = model_bounds.y.max
        y_extreme = max(abs(y_min), abs(y_max))
        y_location = 0    
        
        #Set x and y start coordinates for grid
        """Do the math to start in the -x,-y quadrant but still align with 0,0. This
        will make the loop easier. AG's will be placed in a rectular alignment. 
        This work with any shape because the location will be checked first to
        make sure it is solid
        """
        num_x = math.floor((x_extreme)/spacing)
        x_start = -1*num_x*spacing
        num_y = math.floor((y_extreme)/spacing)
        y_start = -1*num_y*spacing
        x_location = x_start
        y_location = y_start
        
        #Set counter for number of alignment geometry
        AG_count  = 0
        
        #Set values for raycast
        ray_direction = Vector((0,0,-1))
        z_ray_location = 1000
        
        """
        For objects created (such as molds), raycast will use normalized coordinates from -1 to 1
        For imported objects, raycast will use global coordinates, so we need to check which is in use
        Raycast from either side of the chunk in several locations, and if they are outside the -1 to 1 range, 
        don't normalize, otherwise do. Even without normalized coordinates, raycast does not respond to the
        recentering algoirithm and can therefor have negative values for z_location
        """
        normalize = False
        top_success, top_location, normal, val = model.ray_cast(Vector((0,0,1000)),Vector((0,0,-1)))
        bottom_success, bottom_location, normal, val = model.ray_cast(Vector((0,0,-1000)),Vector((0,0,1)))
        if top_location[2]-bottom_location[2] <= 2:
            normalize = True
        if abs(top_location[2]) > 2 or abs(bottom_location[2]) > 2:
            normalize = False
        
        #Set layer location
        if normalize == True:
            #this is the normalized height of this layer compared to chunk in range [-1,1]
            layer_location = (2/num_vertical_layers)*layer_number-1
        else:
            layer_location = ((total_height/num_vertical_layers)*layer_number)+(lower_ray_bound)
        
        # #FOR TESTING
        # print('Layer Location: '+str(layer_location))
        
        #Check locations on grid
        """
        The initial plan was to check each location as the AG was placed there, but
        due to this changing the chunk geometry, it works best with raycast to check 
        each grid location first, then go through and create alignment geometries at
        successful locations
        """
        grid_success = []
        
        while x_location < x_max:
            while y_location < y_max:
                Pass = True  
                
                """
                The check step will check 9 points for each alignment geometry: The center,
                the desired radius of the AG in the 4 unit directions (+/-x/y), and the radius
                plus the offset in the 4 unit directions
                """
                #Check Center
                if normalize == True:
                    x_ray_location = (x_location/x_extreme)
                    y_ray_location = (y_location/y_extreme)
                else:
                    x_ray_location = x_location
                    y_ray_location = y_location
                ray_origin = Vector((x_ray_location,y_ray_location,z_ray_location))
                success, location, normal, val = model.ray_cast(ray_origin,ray_direction)
                
                if normalize == True:
                    if success == False:
                        Pass = False
                    else:
                        if abs(location[2]-layer_location) > .1:
                            Pass = False
                else: 
                    if success == False:
                        Pass = False
                    else:
                        if abs(location[2]-layer_location) > 1:
                                Pass = False
                                
                # #FOR TESTING
                # if success == True:
                #     print("Success at Center, Location: "+str(location))
                
                #If center is good, check the other points around center
                if Pass == True:
                    # x_locations_list = []
                    x_locations_list = Vector((x_location,x_location,x_location,x_location,x_location+radius_base,x_location+radius_base+offset,x_location-radius_base,x_location-radius_base-offset))
                    y_locations_list = Vector((y_location+radius_base,y_location+radius_base+offset,y_location-radius_base,y_location-radius_base-offset,y_location,y_location,y_location,y_location))
                    for i in range(0,len(x_locations_list)):
                        
                        #Normalize x,y location on scale [-1,1]
                        if normalize == True:
                            x_ray_location = (x_locations_list[i]/x_extreme)
                            y_ray_location = (y_locations_list[i]/y_extreme)
                        else:
                            x_ray_location = x_locations_list[i]
                            y_ray_location = y_locations_list[i]
                    
                        #Perform Raycast
                        ray_origin = Vector((x_ray_location,y_ray_location,z_ray_location))
                        success, location, normal, val = model.ray_cast(ray_origin,ray_direction)
                        
                        #Perform Checks
                        if normalize == True:
                            if success == False:
                                Pass = False
                            else:
                                if abs(location[2]-layer_location) > .1:
                                    Pass = False
                        else: 
                            if success == False:
                                        Pass = False
                            else:
                                if abs(location[2]-layer_location) > 1:
                                    Pass = False
                grid_success.append(Pass)
                
                y_location += spacing
                
            y_location = y_start
            x_location += spacing
            
        x_location = x_start
        y_location = y_start
        
        grid_counter = 0
        #Creat Aligment Geometries at successful locations
        while x_location < x_max:
            while y_location < y_max:
                #create alignment geometry, in this case cone, it will be selected after creation
                if grid_success[grid_counter] == True:
                    AG_count += 1
                    bpy.ops.mesh.primitive_cone_add(radius1=radius_base, radius2=0, depth=height, location=(x_location, y_location, z_location))
                
                    #join the AG to the model, select the part to attatch to and then join
                    model.select = True
                    bpy.ops.object.join()
                    
                
                    #tell blender that model is the combined object
                    model = scene.objects.active
                    model.name = "model_name"
                    
                #iterate y_location
                y_location += spacing
                grid_counter += 1
                
            y_location = y_start
            x_location += spacing
                
        
        return(AG_count, model)
        
    @staticmethod
    def alignment_geometry_bottom(AG_parameters, model):
        scene = bpy.context.scene
        Chunker.deselect_all_objects()
        model.select = True
        scene.objects.active = model
        
        #set size and spacing variables
        radius_base = AG_parameters[0]
        height = AG_parameters[1]
        spacing = AG_parameters[2]
        offset = AG_parameters[3]
        num_vertical_layers = AG_parameters[4]
        layer_number = AG_parameters[5]
        total_height = AG_parameters[6]
        
        model_bounds = SimulatorMath.calculateBounds(model)
        z_min = model_bounds.z.min
        z_location = z_min+height/2
        x_min = model_bounds.x.min
        x_max = model_bounds.x.max
        x_location = 0
        x_extreme = max(abs(x_min), abs(x_max))
        y_min = model_bounds.y.min
        y_max = model_bounds.y.max
        y_location = 0
        y_extreme = max(abs(y_min), abs(y_max))
        
        """Do math to start in the -x,-y quadrant but still align with 0,0. This
        will make the loop easier. AG's will be placed in a rectular alignment. 
        This work with any shape because the location will be checked first to
        make sure it is solid
        """
        num_x = math.floor((0-x_min)/spacing)
        x_start = -1*num_x*spacing
        num_y = math.floor((0-y_min)/spacing)
        y_start = -1*num_y*spacing
        
        x_location = x_start
        y_location = y_start
        
        #Set values for raycast
        ray_direction = Vector((0,0,1))
        z_ray_location = -1000
        
        #Normalize for raycast if needed
        normalize = False
        top_success, top_location, normal, val = model.ray_cast(Vector((0,0,1000)),Vector((0,0,-1)))
        bottom_success, bottom_location, normal, val = model.ray_cast(Vector((0,0,-1000)),Vector((0,0,1)))
        if top_location[2]-bottom_location[2] <= 2:
            normalize = True
        if abs(top_location[2]) > 2 or abs(bottom_location[2]) > 2:
            normalize = False
        
        #Set layer location
        if normalize == True:
            #this is the normalized height of this layer compared to chunk in range [-1,1]
            layer_location = (2/num_vertical_layers)*layer_number-1
        else:
            layer_location = ((total_height/num_vertical_layers)*layer_number)-(total_height/2)
        
        
        #Check locations on grid
        """
        The initial plan was to check each location as the AG was placed there, but
        due to this changing the chunk geometry, it works best with raycast to check 
        each grid location first, then go through and create alignment geometries at
        successful locations
        """
        grid_success = []

        while x_location < x_max:
            while y_location < y_max:
                Pass = True  
                
                #Check Center
                x_ray_location = (x_location/x_extreme)
                y_ray_location = (y_location/y_extreme)
                ray_origin = Vector((x_ray_location,y_ray_location,z_ray_location))
                success, location, normal, val = model.ray_cast(ray_origin,ray_direction)
                
                if normalize == True:
                    if success == False:
                        Pass = False
                    else:
                        if abs(location[2]-layer_location) > .1:
                            Pass = False
                else: 
                    if success == False:
                                Pass = False
                    else:
                        if abs(location[2]-layer_location) > 1:
                            Pass = False
                
                """
                The check step will check 9 points for each alignment geometry: The center,
                the desired radius of the AG in the 4 unit directions (+/-x/y), and the radius
                plus the offset in the 4 unit directions
                """
                if Pass == True:
                    x_locations_list = Vector((x_location,x_location,x_location,x_location,x_location+radius_base,x_location+radius_base+offset,x_location-radius_base,x_location-radius_base-offset))
                    y_locations_list = Vector((y_location+radius_base,y_location+radius_base+offset,y_location-radius_base,y_location-radius_base-offset,y_location,y_location,y_location,y_location))
                    for i in range(0,len(x_locations_list)):
                        #Normalize x,y location on scale [-1,1]
                        if normalize == True:
                            x_ray_location = (x_locations_list[i]/x_extreme)
                            y_ray_location = (y_locations_list[i]/y_extreme)
                        else:
                            x_ray_location = x_locations_list[i]
                            y_ray_location = y_locations_list[i]
                    
                        #Perform Raycast
                        ray_origin = Vector((x_ray_location,y_ray_location,z_ray_location))
                        model
                        success, location, normal, val = model.ray_cast(ray_origin,ray_direction)
                        
                        #Perform Checks
                        if normalize == True:
                            if success == False:
                                Pass = False
                            else:
                                if abs(location[2]-layer_location) > .1:
                                    Pass = False
                        else: 
                            if success == False:
                                        Pass = False
                            else:
                                if abs(location[2]-layer_location) > 1:
                                    Pass = False
                    
                grid_success.append(Pass)
                
                y_location += spacing
                
            y_location = y_start
            x_location += spacing
            
        x_location = x_start
        y_location = y_start
        
        grid_counter = 0
        
        #Creat Aligment Geometries at successful locations
        while x_location < x_max:
            while y_location < y_max:
                #create alignment geometry, in this case cone, it will be selected after creation
                if grid_success[grid_counter] == True:
                    bpy.ops.mesh.primitive_cone_add(radius1=radius_base, radius2=0, depth=height, location=(x_location, y_location, z_location))
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

                    
                #iterate y_location
                y_location += spacing
                grid_counter += 1
                
            y_location = y_start
            x_location += spacing
                
        
        return(model)
        



    @staticmethod
    def start_scaled_layers(machines, model_object):
        """
        The following loops infinitely when machines is 1 because there is no z chunking in the 
        start_single method
        """
        if len(machines) == 1:
            Chunker.start_single(machines[0], model_object)

        colors = [(255, 10, 27), (0, 30, 83), (255, 255, 255)]

        color_index = 0
        machine_parameters = machines[0].machine_parameters
        machine_width = machine_parameters.machine_width
        printhead_slope = machine_parameters.printhead_slope

        model_bounds = SimulatorMath.calculateBounds(model_object)
        model_width = model_bounds.x.max - model_bounds.x.min

        wing_slope = 50.00
        z_height = model_bounds.z.max - model_bounds.z.min
        h = wing_slope * math.tan(printhead_slope)

        if z_height <= h:
            return Chunker.start_scaled(machines, model_object)

        max_chunk_num = 0

        layers_division = Chunker.vertical_chunking(machines, model_object)

        for (j, layers) in enumerate(layers_division.bottom_layer):
            layer_mult = (j ) * 100 #changed from (j+1) * 10
            chunker_result = Chunker.start(machine_parameters, layers, number_of_machines=2)
            chunk_row = Chunker.split(chunker_result.origin_chunk,
                                      model_width,
                                      machine_width,
                                      printhead_slope,
                                      num_pieces=-1)
            chunk_number = len(chunk_row) - 1 + layer_mult
            for (i, chunk) in enumerate(chunk_row):
                new_chunk = Chunk(chunk, number=i + layer_mult)
                new_chunk.color = colors[color_index]

                machine_number = i - (i % 2)
                machines[machine_number].chunks.append(new_chunk)

                new_chunk.set_name("Machine {} Chunk {}".format(machine_number, i + layer_mult))

                if i % 2 == 1:
                    new_chunk.add_dependency(i - 1)
                    if (i + 1) < len(chunk_row):
                        new_chunk.add_dependency(i + 1)
                if j > 0:
                    new_chunk.add_dependency(max_chunk_num)

            center_chunk_dependencies = list(range(0 + layer_mult, len(chunk_row) + layer_mult))
            previous_row_dependencies = list(range(0 + layer_mult, len(chunk_row) + layer_mult))
            last_chunk_number = center_chunk_dependencies[-1]

            row_number = 0

            color_index = 1

            for south_chunk in chunker_result.south_chunks:
                row_number += 1

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
                    new_chunk.row = -1 * row_number
                    new_chunk.color = colors[color_index]
                    new_chunk.add_dependency(previous_row_dependencies)

                    machine_number = i - (i % 2)+1
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

                color_index += 1
                if color_index > 2:
                    color_index = 0

            previous_row_dependencies = center_chunk_dependencies

            row_number = 0

            color_index = 2

            for north_chunk in chunker_result.north_chunks:
                row_number += 1
                chunk_row = Chunker.split(north_chunk,
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
                    new_chunk.color = colors[color_index]
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

                max_chunk_num = last_chunk_number

                color_index -= 1
                if color_index < 0:
                    color_index = 2

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

        model_bounds = SimulatorMath.calculateBounds(model_object)
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
 
    """
    The following method start_mold creates a negative of the input file in a box and then sends to start_scaled_layers for chunking
    """
    
    @staticmethod
    def start_mold(machines, model_object):
        Chunker.deselect_all_objects()
        scene = bpy.context.scene
        
        # Make the object visible, selected, active, and ready for editing
        model_object.hide = False
        model_object.select = True
        scene.objects.active = model_object
        model_object.name = "model_object_name"
        
        #Set mold parameters
        wall_thickness = 20 #mm this is the thickness of the sides
        base_thickness = wall_thickness #this is the thickness of the top and bottom
        
        #calculate model bounds. I only need max values because object comes in centered
        model_bounds = SimulatorMath.calculateBounds(model_object)
        z_max = model_bounds.z.max
        x_max = model_bounds.x.max
        y_max = model_bounds.y.max
        box_z_location = z_max/2
        
        z_size = z_max/2 + base_thickness
        x_size = x_max + wall_thickness
        y_size = y_max + wall_thickness
        
        #create box around object
        bpy.ops.mesh.primitive_cube_add(radius=1, location=(0, 0, 0))
        bpy.ops.transform.resize(value=(x_size, y_size, z_size), constraint_orientation='GLOBAL')
        bpy.ops.transform.translate(value=(0,0,box_z_location), constraint_axis=(False, False, True), constraint_orientation='GLOBAL')

        
        #subtract original from box
        box = scene.objects.active
        box.select = True
        bpy.ops.object.modifier_add(type='BOOLEAN')
        bpy.context.object.modifiers["Boolean"].operation = 'DIFFERENCE'
        bpy.context.object.modifiers["Boolean"].object = bpy.data.objects["model_object_name"]
        bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Boolean")
        
        #Delete the model_object after it has been subtracted
        box.select = False
        model_object.select = True
        bpy.ops.object.delete(use_global=False)
        
        
        #recenter object
        Chunker.recenter_object(box)
        
        #start chunking with the box
        Chunker.start_scaled_layers(machines, box)
        
 
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

        robot_parameters = machines[0].parameters
        robot_width = robot_parameters.width
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
        robot_parameters = machines[0].parameters
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
    #     chunk_bounds = SimulatorMath.calculateBounds(chunk)
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

        chunk_bounds = SimulatorMath.calculateBounds(chunk)

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




