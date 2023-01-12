#This program is used to create the .yaml file of the floor

"""
As inputs, the program accepts a floor size (in number of floor tiles), robot
starting positions, robot goal positions, and any filled chunks (all based on 
grid position on floor)
"""

import math
import yaml

class sizes():
    #size multiplier for floor tiles to mm
    size_multiplier = 100 #mm
    #how large the robot is in the visualization
    robot_size = 0 #mm
    #not quite sure, but changes resolution of the visualization
    grid_size = 100
    robot_offset = 0

def conv_coord(grid_pos_x, grid_pos_y):
    mid = math.floor(sizes.size_multiplier/2)
    pos_x = grid_pos_x*sizes.size_multiplier
    pos_y = grid_pos_y*sizes.size_multiplier
    return(pos_x, pos_y, mid)


    
    
def make_floor(num_FT_x, num_FT_y, rob_start_pos, rob_goal_pos, filled_pos):
    num_robots = len(rob_start_pos)
    
    
    file_name = "AMBOTS_Floor"
    file_yaml = file_name + ".yaml"
    with open(file_yaml, 'w') as file:
        file.write("---\n")
        
        #robot goal positions
        file.write("GOAL:\n")
        for i in range(0,num_robots):
            (x, y, mid) = conv_coord(rob_goal_pos[i][0], rob_goal_pos[i][1])
            file.write("- !!python/tuple [" + str(int(x)) + ", " + \
                       str(int(y)) + "]\n")
                
        file.write("GRID_SIZE: " + str(sizes.grid_size) + "\n")
        file.write("RECT_OBSTACLES:\n")
        
        #Border
        (x_max, y_max, mid) = conv_coord(num_FT_x*2, num_FT_y*2)
        file.write("  0:\n")
        file.write("  - [" + str(0) + ", " + str(0) + "]\n")
        file.write("  - [" + str(x_max) + ", " + str(y_max) + "]\n")
        
        #filled chunks
        num_chunks = len(filled_pos)
        for i in range(0,num_chunks):
            file.write("  " + str(i+1)+ ":\n")
            (x, y, mid) = conv_coord(filled_pos[i][0], filled_pos[i][1])
            x = int(x + sizes.grid_size/2)
            y = int(y + sizes.grid_size/2)
            file.write("  - [" + str(x) + ", " + str(y) + "]\n")
            file.write("  - [" + str(x) + ", " + str(y) + "]\n")
            
        #robot radius
        file.write("ROBOT_RADIUS: " + str(sizes().robot_size) + "\n")
        
        #robot starting positions
        file.write("START:\n")
        for i in range(0,num_robots):
            (x, y, mid) = conv_coord(rob_start_pos[i][0], rob_start_pos[i][1])
            file.write("- !!python/tuple [" + str(int(x)) + ", " + \
                       str(int(y)) + "]\n")
        
        return(sizes.size_multiplier)

# #For testing of this method        
# rob_start_pos = [[0,0],[0,5]]
# rob_goal_pos = [[12,3],[14,3]]
# filled_pos = [[2,0]]
# make_floor(8, 6, rob_start_pos, rob_goal_pos, filled_pos)