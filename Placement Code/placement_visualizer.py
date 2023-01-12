#This program is used to visualize the placement of chunks on the factory floor

"""
Inputs: Size of space in floor tiles (2 grid squares per floor tile), location
of all chunks, and which job each chunk belongs to.

Output: a graphic in a new window
"""

import math
import cv2
import numpy as np

def placement_vis(floor_size, chunk_positions, chunk_job):
    disp_mult = 100
    size_multiplier = 75
    start_offest = 100   
    colors = [[0,0,0],[0,0,255],[0,255,0],[255,0,0],[0,150,150],[150,150,0],[150,0,150],[150,150,150]]
    
    #create window
    cv2.namedWindow('placement', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('placement', (1280, 720))
    canvas = np.ones((1080,1500,3), np.uint8)*255
    
    #calculate the total size of floor with multiplier
    x_end = floor_size[0]*2*size_multiplier + start_offest
    y_end = floor_size[1]*2*size_multiplier + start_offest
    
    #create floor grid
    cv2.rectangle(canvas, (start_offest, start_offest), (x_end,y_end), (0, 0, 255), thickness=3)
    #vertical lines
    for i in range(0,floor_size[0]*2):
        x = start_offest + i*size_multiplier
        cv2.line(canvas, (x,start_offest), (x, y_end), (0, 0, 255), thickness=3)
    #horizontal lines
    for i in range(0,floor_size[1]*2):
        y = start_offest + i*size_multiplier
        cv2.line(canvas, (start_offest, y), (x_end, y), (0, 0, 255), thickness=3)    
    
    job_number = max(chunk_job)[0] + 1
    for i in range(0,job_number):
        job_positions = []
        for j in range(0,len(chunk_positions)):
            if chunk_job[j][0] == i:
                job_positions.append(chunk_positions[j])
        x_positions = []
        y_positions = []        
        for j in range(0,len(job_positions)):
            x_positions.append(job_positions[j][0])
            y_positions.append(job_positions[j][1])
        min_x_grid = int(min(x_positions))
        max_x_grid = int(max(x_positions)+1)
        min_y_grid = int(min(y_positions))
        max_y_grid = int(max(y_positions)+1)
        
        min_x = min_x_grid*size_multiplier + start_offest
        min_y = min_y_grid*size_multiplier + start_offest
        max_x = max_x_grid*size_multiplier + start_offest
        max_y = max_y_grid*size_multiplier + start_offest
        
        #draw job
        # random_color = np.round(np.random.rand(1,3)*255)
        cv2.rectangle(canvas, (min_x, min_y), (max_x, max_y), (int(colors[i][0]),int(colors[i][1]),int(colors[i][2])), thickness=-1)
        
        #show initial chunk
        initial_x_grid_start = int(job_positions[0][0])
        initial_x_grid_end = int(job_positions[0][0] + 1)
        initial_y_grid_start = int(job_positions[0][1])
        initial_y_grid_end = int(job_positions[0][1] + 1)
        
        initial_x_start = initial_x_grid_start*size_multiplier + start_offest
        initial_x_end = initial_x_grid_end*size_multiplier + start_offest
        initial_y_start = initial_y_grid_start*size_multiplier + start_offest
        initial_y_end = initial_y_grid_end*size_multiplier + start_offest
        
        cv2.rectangle(canvas, (initial_x_start, initial_y_start), (initial_x_end, initial_y_end), (255,165,0), thickness=4)
    
        
    #show and delete image
    cv2.imshow('placement',canvas)
    cv2.waitKey(0)
    cv2.destroyWindow('placement')


# #TEST configuration 
# floor_size = [8,6]

# chunk_positions = [[7.0, 4.0],
#  [8.0, 4.0],
#  [5.0, 2.0],
#  [5.0, 1.0],
#  [2.0, 4.0],
#  [1.0, 4.0],
#  [3.0, 8.0],
#  [3.0, 9.0],
#  [7.0, 5.0],
#  [8.0, 5.0],
#  [6.0, 2.0],
#  [6.0, 1.0],
#  [2.0, 3.0],
#  [1.0, 3.0],
#  [2.0, 8.0],
#  [2.0, 9.0],
#  [7.0, 6.0],
#  [8.0, 6.0],
#  [7.0, 2.0],
#  [7.0, 1.0],
#  [2.0, 2.0],
#  [1.0, 2.0],
#  [1.0, 8.0],
#  [1.0, 9.0]]

# chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]

# # chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,3],[5,2],[9,4],[9,5],\
# #                     [0,4],[1,4],[13,2],[14,2],[6,3],[6,2],[10,4],[10,5],\
# #                     [0,3],[1,3],[13,3],[14,3],[7,3],[7,2],[11,4],[11,5],]

# # chunk_positions = [[6, 3], [5, 3], [6, 5], [7, 5], [12, 0], [13, 0], [2, 4], [1, 4], [6, 2], [5, 2], [6, 6], [7, 6], [12, 1], [13, 1], [2, 3], [1, 3], [6, 1], [5, 1], [6, 7], [7, 7], [12, 2], [13, 2], [2, 2], [1, 2]]

# # chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]

# placement_vis(floor_size, chunk_positions, chunk_job)



