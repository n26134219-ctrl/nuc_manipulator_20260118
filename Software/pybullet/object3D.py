import numpy as np

import time
from math import *
import threading
from scipy.spatial.transform import Rotation as R
from math import *


class Object3D(object):
    def __init__(self):
        self.object_list = []
        self.object_list_length = 0



    def new_object(self, object_name, position=[0,0,0],view="down", angle=0):
        object = {}
        object['name'] = object_name
        object['size'] = [0, 0, 0]
        object['position'] = position
        object['center'] = [position[0], position[1], position[2]]
        object['view'] = view
        object['color'] = [1, 1, 1, 1]  # white, last is transparency
        object['pick_pos'] = [0, 0, 0]
        object['pick_type'] = "side"
        if object_name == "cereal box":
            object['size'] = [10, 60, 60]
            object['color'] = [1, 1, 0, 1]
            if view == "down":
                object['center'] = [position[0], position[1], position[2] - 30]
                pick_x = object['size'][1]/2* cos(radians(angle)) + position[0]
                pick_y = object['size'][1]/2* sin(radians(angle)) + position[1]
                object['pick_pos'] = [pick_x, pick_y, position[2] , object['center'][2]]
                object['pick_type'] = "side"
            elif view == "side":
                object['center'] = [position[0], position[1], position[2] ]
                pick_x = object['size'][1]/2* cos(radians(angle)) + position[0]
                pick_y = object['size'][1]/2* sin(radians(angle)) + position[1]
                object['pick_pos'] = [pick_x, pick_y, position[2] , object['center'][2]]
                object['pick_type'] = "side"


        return object


    def add_object(self, object):
        self.object_list.append(object)
        self.object_list_length += 1
