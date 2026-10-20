import sys
import math
import numpy as np
from copy import deepcopy
from time import perf_counter, sleep
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from turtle_interface_soln import TurtleController

from rrt import rrt
from astar import Astar
from loadmap import loadmap

def main(args=None):
    rclpy.init(args=args)
    map_name = input("MAP NAME (../../lib/maps/<NAME>): ")
    map_struct = loadmap(map_name)
    method = input("astar or rrt")

    node = TurtleController()
    try:
        if not node.wait_for_pose():
            raise RuntimeError("No pose data received")

        start = np.array(node.pose) # already in the world frame
        goal = np.array([8.0, 5.0, 0.0])
        if method == "astar": 
            path = Astar(map_struct, start, goal, resolution=np.array([0.2, 0.2, 0.2]))
        else: 
            path = rrt(map_struct, start, goal)

        if path is None:
            raise RuntimeError("No path found")

        node.follow_path(path)
        
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
