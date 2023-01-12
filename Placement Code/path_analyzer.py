import numpy as np
"""
This method straightens all paths and calculates their lengths. It also generates
a visual path which is compatible with the visualizer method
"""

def analyze_paths(paths, robot_positions, robot_goal_positions, printing_chunks, moving_robots, robot_schedules, robot_last_chunk_printed):
    robot_path_lengths = np.zeros(len(robot_positions))

    #set robot schedules
    for count, path in enumerate(paths):
        #find robot
        start_position = path[0]
        start_position_converted = [int((start_position[0]-50)/100), int((start_position[1]-50)/100)]
        robot = robot_positions.index(start_position_converted)
        
        #find path length
        end_position = path[-1]
        for point in path:
            if point[0] != end_position[0] or point[1] != end_position[1]:
                robot_path_lengths[robot] += 1
        
        end_position = path[-1]
        end_position_converted = [int((end_position[0]-50)/100), int((end_position[1]-50)/100)]
        chunk = printing_chunks[robot_goal_positions.index(end_position_converted)]
        
        robot_schedules[robot].append(chunk)
        robot_last_chunk_printed[robot] = chunk
        robot_positions[robot] = end_position_converted
    
    
    return(robot_path_lengths,robot_schedules,robot_positions,robot_last_chunk_printed)
            
paths = np.array([[[  50 ,  50],  [ 150 ,  50],
  [ 250 ,  50],
  [ 250 , 150],
  [ 250 , 250],
  [ 250 , 350],
  [ 250 , 450],
  [ 250 , 550],
  [ 250 , 650],
  [ 250,  750],
  [ 250,  850],
  [ 250,  950],
  [ 250, 1050],
  [ 250, 1050]]
,
  [[ 150,   50],
  [ 250,   50],
  [ 350,   50],
  [ 450,   50],
  [ 450,  150],
  [ 450,  250],
  [ 450,  350],
  [ 450,  450],
  [ 450,  550],
  [ 450,  650],
  [ 450,  750],
  [ 450,  850],
  [ 450,  950],
  [ 450, 1050]]
,
  [[ 250,   50],
  [ 350,   50],
  [ 450,   50],
  [ 550,   50],
  [ 550,  150],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250],
  [ 550,  250]]
,
  [[ 350,   50],
  [ 450,   50],
  [ 550,   50],
  [ 550,  150],
  [ 550,  250],
  [ 550,  350],
  [ 550,  450],
  [ 550,  450],
  [ 550,  450],
  [ 550,  450],
  [ 550,  450],
  [ 550,  450],
  [ 550,  450],
  [ 550,  450]]])


robot_goal_positions = [[5, 2], [5, 4], [2, 10], [4, 10]]
robot_schedules = [[]]
for i in range(0,len(robot_goal_positions)-1):
    robot_schedules.append([])
robot_positions = [[0,0],[1,0],[2,0],[3,0]]
moving_robots = [3, 2, 1, 0]
printing_chunks = [6, 8, 0, 2]
robot_last_chunk_printed = np.zeros(len(robot_positions),int)

(robot_path_lengths,robot_schedules,robot_positions, robot_last_chunk_printed) = analyze_paths(paths, robot_positions, robot_goal_positions, printing_chunks, moving_robots, robot_schedules, robot_last_chunk_printed)