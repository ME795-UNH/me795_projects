"""
Author / Responsible Party: Victoria Edwards
Date: 10/20/2026

AI Usage: I used Claude Sonnet 5 to update this solution for a ground vehicle. This code is heavily modified to work for student use

Purpose:
Implement RRT for a 2d ground vehicle.

The output should be a path of Nx3 waypoints that consist of [x, y, yaw].
Remember to  check to see if the robot collidies with a block from above.

It is up to your team to decide which directions in your graph to check (4 cardinal directions or 8 directions that include the moves along a 45degree angle in the graph).

You are welcome to implement any helper functions that you need. If you use additional files to do this please make sure to submit them. 
"""

import numpy as np
import math
import time
from copy import deepcopy
 
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
 
from loadmap import loadmap
from detect_collision import *
 
 
def rrt(map, start, goal, verbose=False, z_level=1.0,
        r_radius=0.18, inflation=0.35,  n_iter=200):
    """
    Parameters:
        map:               the map struct
        start:             [x, y, yaw] (yaw in radians)
        goal:              [x, y, yaw]
        z_level:           height at which the vehicle is planned and collision checked
        r_radius:          half-width of the robot's square footprint (map units)
        inflation:         extra obstacle padding (map units)
        n_iter:            number of iterations to run the algorithm for
    
    Output:
        path,        a list of shape (N,3) of [x, y, yaw]. First row is the start
                     last row is the goal. Rotation-in-place steps show up as
                     consecutive rows with the same x,y and a different yaw.
                     Returns a path of just the start and end location if no path
                     exists

    NOTE: If you choose to not use one of the variables provided please leave
    it in the function header. The autograder is expecting this form of the
    function.
    """
    start = np.asarray(start, dtype=float).ravel()
    goal = np.asarray(goal, dtype=float).ravel()
    if start.size != 3 or goal.size != 3:
        raise ValueError("start and goal must be [x, y, yaw]")
 
    boundary = map.boundary
    # XXX: STUDENT CODE GOES HERE
    path = None

    # XXX: STUDENT CODE ENDS HERE
    
    return(path) 
 
 
if __name__ == '__main__':
    map_struct = loadmap("maps/map2.txt")
    start = np.array([0.0, 0.0, 1.57])
    goal = np.array([8.0, 5.0, 0.0])
    for i in range(10):
        p = rrt(deepcopy(map), deepcopy(start), deepcopy(goal), z_level=z_level)

    # XXX: DO MORE TESTING HERE
