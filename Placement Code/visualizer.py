#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
import time
import sys
from copy import deepcopy
import cv2
import numpy as np
import yaml
import path_analyzer
import sys
import os

current_path = os.getcwd()
(PythonCBS, visualization) = os.path.split(current_path)
(Placement, PythonCBS) = os.path.split(PythonCBS)
filepath = os.path.join(Placement, 'PythonCBS\cbs_mapf')

sys.path.insert(0, filepath)

from planner import Planner

global visualizer_offset
visualizer_offset = 5

class Simulator:

    def __init__(self):
        # Set up a white 1080p canvas
        floor_size = (8,6)
        # self.canvas = np.ones((1200,1600,3), np.uint8)*255
        self.canvas = np.ones((((floor_size[1]*2)*100+15),((floor_size[0]*2)*100+15),3), np.uint8)*255
        # Draw the rectangluar obstacles on canvas
        for count, v in enumerate(RECT_OBSTACLES.values()):
            self.draw_rect(np.array([np.array(v)]), count)
        
        #draw grid on canvas
        self.draw_grid(floor_size)

        # Transform the vertices to be border-filled rectangles
        static_obstacles = self.vertices_to_obsts(RECT_OBSTACLES)

        # Call cbs-mapf to plan
        self.planner = Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
        before = time.time()
        prepath = self.planner.plan(START, GOAL, debug=False)
        after = time.time()
        # print('Time elapsed:', "{:.4f}".format(after-before), 'second(s)')
        
        # (robot_path_lengths,robot_paths,robot_visualize_paths) = path_scrubber.scrub_paths(prepath)
        # prepath = prepath
        
        self.path = prepath #robot_visualize_paths #
        # print(self.path)
        

        # Assign each agent a colour
        self.colours = self.assign_colour(len(self.path))

        # Put the path into dictionaries for easier access
        d = dict()
        for i, path in enumerate(self.path):
            self.draw_path(self.canvas, path, i)  # Draw the path on canvas
            d[i] = path
        self.path = d

    '''
    Transform opposite vertices of rectangular obstacles into obstacles
    '''
    @staticmethod
    def vertices_to_obsts(obsts):
        def drawRect(v0, v1):
            o = []
            base = abs(v0[0] - v1[0])
            side = abs(v0[1] - v1[1])
            for xx in range(0, base, 100):
                o.append((v0[0] + xx, v0[1]))
                o.append((v0[0] + xx, v0[1] + side - 1))
            o.append((v0[0] + base, v0[1]))
            o.append((v0[0] + base, v0[1] + side - 1))
            for yy in range(0, side, 100):
                o.append((v0[0], v0[1] + yy))
                o.append((v0[0] + base - 1, v0[1] + yy))
            o.append((v0[0], v0[1] + side))
            o.append((v0[0] + base - 1, v0[1] + side))
            return o
        static_obstacles = []
        for vs in obsts.values():
            static_obstacles.extend(drawRect(vs[0], vs[1]))
        return static_obstacles

    '''
    Randomly generate colours
    '''
    @staticmethod
    def assign_colour(num):
        def colour(x):
            x = hash(str(x+42))
            return ((x & 0xFF, (x >> 8) & 0xFF, (x >> 16) & 0xFF))
        colours = dict()
        for i in range(num):
            colours[i] = colour(i)
        return colours

    def draw_rect(self, pts_arr: np.ndarray, count) -> None:
        for pts in pts_arr:
            if count == 0:
                cv2.rectangle(self.canvas, tuple(pts[0]+visualizer_offset), tuple(pts[1]+visualizer_offset), (0, 0, 255), thickness=3)
            else:
                cv2.rectangle(self.canvas, tuple(pts[0]+visualizer_offset-50), tuple(pts[1]+visualizer_offset+50), (0, 0, 255), thickness=3)

    def draw_path(self, frame, xys, i):
        for x, y in xys:
            cv2.circle(frame, (int(x + visualizer_offset), int(y+visualizer_offset)), 9, self.colours[i], -1)
            
    def draw_grid(self, floor_size):
        #vertical lines
        x_end = floor_size[0]*2*100 
        y_end = floor_size[1]*2*100 
        for i in range(0,floor_size[0]*2):
            x =  i*100 + 50
            cv2.line(self.canvas, (x+visualizer_offset,0+visualizer_offset), (x+visualizer_offset, y_end+visualizer_offset), (0, 0, 0), thickness=2)
        #horizontal lines
        for i in range(0,floor_size[1]*2):
            y = i*100 + 50
            cv2.line(self.canvas, (0+visualizer_offset, y+visualizer_offset), (x_end, y+visualizer_offset), (0, 0, 0), thickness=2)    

    '''
    Press any key to start.
    Press 'q' to exit.
    '''
    def start(self):
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', (1280,720))
        wait = False
        try:
            i = 0
            while True:
                frame = deepcopy(self.canvas)
                for id_ in self.path:
                    x, y = tuple(self.path[id_][i])
                    cv2.circle(frame, (x+visualizer_offset, y+visualizer_offset), ROBOT_RADIUS+10, self.colours[id_], 5)
                cv2.imshow('frame', frame)
                if wait:
                    cv2.waitKey(0)
                    wait = False
                k = cv2.waitKey(250) & 0xFF 
                if k == ord('q'):
                    break
                i += 1
        except Exception:
            cv2.waitKey(0)
            cv2.destroyAllWindows()

def load_scenario(fd):
    with open(fd, 'r') as f:
        global GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL
        data = yaml.load(f, Loader=yaml.FullLoader)
        GRID_SIZE = data['GRID_SIZE']
        ROBOT_RADIUS = data['ROBOT_RADIUS']
        RECT_OBSTACLES = data['RECT_OBSTACLES']
        START = data['START']
        GOAL = data['GOAL']

'''
Use this function to show your START/GOAL configurations
'''
def show_pos(pos):
    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('frame', (1280, 720))
    #These values depend on floor size, I multiple by 100 for it to show up and then add 100 on either side to make sure it is all in frame
    floor_size = (8,6)
    frame = np.ones(((floor_size[1]*200+10),(floor_size[0]*200+10),3), np.uint8)*255
    for x, y in pos:
        cv2.circle(frame, (x+100, y+100), ROBOT_RADIUS+5, (0, 0, 0), 5)
    cv2.rectangle(frame, (0, 0), (floor_size[0]*200, floor_size[1]*200), (0, 0, 255),  thickness=5)
    cv2.imshow('frame', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # From command line, call:
    # python3 visualizer.py scenario1.yaml
    load_scenario("AMBOTS_floor.yaml")
    show_pos(START)
    r = Simulator()
    r.start()

def visualizer():
    load_scenario("AMBOTS_floor.yaml")
    r = Simulator()
    r.start()