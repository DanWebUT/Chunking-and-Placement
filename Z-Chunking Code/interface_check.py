'''
This file is designed to test the method to check layers for how many islands
they have and how many of those cannot have AG because they are too small.
Inputs are the success grids of the interface, and the model bounds to give 
scale
'''

import numpy as np
import pdb
import math
import cvxpy as cp

from scipy.optimize import minimize

class ChunkingParameters:
    max_reach_z = 125

class AGParameters:
    min_AG_rad = 5
    rad_inc = 5
    spacing = 5
    offset = 1

def interface_check(model_bounds, top_success, rad):
    '''
    Checks a chunk interface (as given by a grid of True or False values)
    and returns potential AG locations as well as the number of unique XY 
    chunks or 'islands' and the number of 'islands' that aren't connected
    by AG's
    '''
    #Import model Bounds
    z_max = model_bounds.z_max
    z_min = model_bounds.z_min
    y_max = model_bounds.y_max
    y_min = model_bounds.y_min
    x_max = model_bounds.x_max
    x_min = model_bounds.x_min
    x_extreme = max(abs(x_min), abs(x_max))
    y_extreme = max(abs(y_min), abs(y_max))
    
    #Find size of success grid and map to bounds to find coordinate of points
    x_index = np.size(top_success,1)
    y_index = np.size(top_success,0)
    x_step_size = abs(x_max-x_min)/(x_index-1)
    x_gap = math.ceil(rad/x_step_size)
    y_step_size = abs(y_max-y_min)/(y_index-1)
    y_gap = math.ceil(rad/y_step_size)
    
    # pdb.set_trace()
    
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
                
    # # FOR TESTING
    # numeric_grid = -np.random.rand(y_index,x_index)*1.4999
    # numeric_grid = np.around(numeric_grid, decimals=0)
    
    # for x in range(13,16):
    #     for y in range(13,16):
    #         numeric_grid[y,x] = 0
            
    # numeric_grid[14,14] = -1
            
    
    # #FOR TESTING
    # numeric_grid = -np.ones((y_index,x_index))
    # for i in range(0,x_index):
    #     numeric_grid[9,i] = 0
    #     numeric_grid[i,9] = 0
    
    # pdb.set_trace()
    
    
    #Set start point and 
    y_count = 0
    x_count = 0
    
    #start loop to check for all viable AG positions
    while x_count < x_index:
        while y_count < y_index:
            
            y_pos = y_min + (y_count)*y_step_size
            x_pos = x_min + (x_count)*x_step_size
            #skip points too close to boundary
            if abs(abs(y_pos)-y_extreme) < rad or abs(abs(x_pos)-x_extreme)< rad:
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
            
            
            
            # pdb.set_trace()
            
            y_count += 1
        y_count = 0
        x_count += 1

    # pdb.set_trace()    
    
    numeric_grid = connections(numeric_grid, x_index, y_index,-1,-1, 1, 2, 2)
    
    # pdb.set_trace()
    return(numeric_grid, x_index, y_index) 
    # return(success, numeric_grid) 

def generate_islands_grid(numeric_grid,x_index,y_index):
    '''
    This method creates an island map, counts the number of islands
    as well as the number of islands without an AG
    '''
    
    # pdb.set_trace()
    
    islands_grid = -(numeric_grid)*10
    island_number = 1
    no_AG_island_number = 0
    
    #Go once over entire grid
    for x in range(0,x_index):
        for y in range(0,y_index):
            if islands_grid[y,x] == -10 or islands_grid[y,x] == -20:
                islands_grid[y,x] = island_number
                islands_grid = connections(islands_grid, x_index, y_index, -10, -20, island_number, island_number, island_number)
                island_number += 1
            elif islands_grid[y,x] == 10:
                islands_grid[y,x] =  island_number
                islands_grid = connections(islands_grid, x_index, y_index, 10, 10, island_number, island_number, island_number)
                island_number += 1
                no_AG_island_number += 1
    #revert the last island number because none were actually created            
    island_number -= 1
    
    return(islands_grid, island_number, no_AG_island_number)
        

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
    z_max = model_bounds.z_max
    z_min = model_bounds.z_min
    y_max = model_bounds.y_max
    y_min = model_bounds.y_min
    x_max = model_bounds.x_max
    x_min = model_bounds.x_min
    x_extreme = max(abs(x_min), abs(x_max))
    y_extreme = max(abs(y_min), abs(y_max))
    
    #Find size of success grid and map to bounds to find coordinate of points
    x_index = np.size(TS,1)
    y_index = np.size(TS,0)
    x_step_size = abs(x_max-x_min)/(x_index-1)
    y_step_size = abs(y_max-y_min)/(y_index-1)
    spacing = max(AGParameters.spacing, 2*max_rad + AGParameters.offset*2)
    x_spacing_index = int(math.ceil(spacing/x_step_size))
    y_spacing_index = int(math.ceil(spacing/y_step_size))
    
    #generate interface data at minimum radius
    (numeric_grid, x_index_1, y_index_1) = interface_check(model_bounds,TS, min_rad)
    (islands_grid_min_rad, island_number_min_rad, no_AG_island_number) = generate_islands_grid(numeric_grid, x_index, y_index)
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
        (numeric_grid, x_index_1, y_index_1) = interface_check(model_bounds,TS, rad)
        (islands_grid, island_number, no_AG_island_number) = generate_islands_grid(numeric_grid, x_index, y_index)
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
    # pdb.set_traece()
    return(AG_locations_array)

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
    min_cuts = math.ceil(((layer_split_number+1)*layer_height)/ChunkingParameters.max_reach_z)-1
    layer_height_array = (np.array(range(0,layer_split_number+1))+1)*layer_height
    
    
    #Create ideal configuration to solve for
    config = cp.Variable(layer_split_number, boolean=True)
    
    #Create objective
    objective = sum(config*layer_islands_data[0])
    
    problem = cp.Problem(cp.Minimize(objective),[layer_height_constraint(layer_split_number,layer_height,config) <= ChunkingParameters.max_reach_z])
    
    #solve
    problem.solve(solver = cp.GLPK_MI)
    
    
    
    num_vertical_layers = 0
    
    # return(num_vertical_layers, conf)

#Layer Height Constraint
def layer_height_constraint(layer_split_number,layer_height,config):
    #create array of the absolute height (relative to ground) of each layer
    layer_height_array = (np.array(range(1,layer_split_number+1))+1)*layer_height
    
    #filter set inactive cuts to 0
    active_heights = layer_height_array * config
    
    #filter out all 0's
    active_heights_trim = active_heights[active_heights != 0]
    
    #insert top and bottom values
    active_heights = np.append(active_heights,[(layer_split_number+1)*layer_height])
    active_heights_trim = np.insert(active_heights,0,0)
    
    #take the difference between active cuts to find distance
    active_heights_diff = np.diff(active_heights_trim)
    
    #find the maximum of those differences
    max_height = max(active_heights_diff)
    
    return(max_height)

class model_bounds:
    z_max = 250
    z_min = 0
    y_max = 100
    y_min = -100
    x_max = 100
    x_min = -100

# #Main method
# data = [[1,1,2,1,2,2,3,1],[0,0,0,1,0,1,1,0]]
# #7.89
# # data = [[40,592,600,4,1105,1,1,1,224,392,4,252,208,1,1,1,292,279,244,199,83,86,96,84,112,104,84,84,84,112,76,56,56,104,1,4,1,12],[28,592,600,0,1105,0,0,0,0,224,392,0,252,208,0,0,0,282,279,244,199,82,85,96,84,112,104,84,84,84,112,76,56,104,0,4,0,12]]
# layer_islands_data = np.array(data)
# layer_split_number = np.size(layer_islands_data[0])
# (num_vertical_layers, split_locations_array) = layer_locations(layer_islands_data, layer_split_number,38)
# max_height = layer_height_constraint(layer_split_number,38,np.array([0,0,0,1,0,1,1,0]))






# create success arrays
y_size = 17
x_size = 17
min_rad = AGParameters.min_AG_rad #mm from robot accuracy
max_rad = 30
spacing = 30 #mm

#Chunking Plane 1
top_success = np.zeros((y_size,x_size), dtype = bool)
for x in range(-6,7):
    for y in range(-3, 4):
        top_success[8+y,8+x] = True
        
#Chunking Plane 2
for x in range(-1,2):
    for y in range(-3, 4):
        top_success[8+y,8+x] = False

# top_success = np.ones((y_size,x_size), dtype = bool)
# for x in range(-5,6):
#     for y in range(-5, 6):
#         if math.sqrt(x*x+y*y) <=5:
#             top_success[6+y,8+x] = True
#             top_success[20+y,19+x] = True
bottom_success = np.ones((y_size,x_size), dtype = bool)
(numeric_grid, x_index, y_index) = interface_check(model_bounds,top_success, min_rad)
(islands_grid, island_number, no_AG_island_number) = generate_islands_grid(numeric_grid, x_index, y_index)
AG_locations_array = AG_locations(model_bounds, top_success, min_rad, max_rad)

# for i in range(0,np.size(AG_locations_array[0])):
#     rad = AG_locations_array[0,i]
#     x_pos = AG_locations_array[1,i]
#     y_pos = AG_locations_array[2,i]
    
#     print('Radius of ' + str(rad) + ' at (' + str(x_pos) + ',' + str(y_pos)+ ')')


def layer_locations_old(layer_islands_data, layer_split_number, layer_height):
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
    
    
    
        
    #FOR TESTING
    pdb.set_trace()
    
    split_status = np.zeros([16777280,layer_split_number])
    feasibility_array = np.ones(16777280, dtype=bool)
    num_chunks_array = np.zeros([16777280,2])
        
    
    for i in range(0,(16777280)):
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
        
        #FOR TESTING
        
        #Check if combination feasible based on height
        current_height = layer_height
        for j in range(0, layer_split_number):
            if split_status[i,j] == 1:
                current_height = 0
            current_height += layer_height
            
            if current_height > ChunkingParameters.max_reach_z:
                feasibility_array[i] = False
                break
        
        #FOR TESTING
        # pdb.set_trace()
        
        #figure out number of islands at each configuration (if unfeasible,)
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