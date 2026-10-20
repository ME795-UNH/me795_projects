"""
Author / Responsible Party: Victoria Edwards
Date: 10/20/2026

AI Usage: I used Claude Sonnet 5 to update this solution for a ground vehicle. This code is heavily modified to work for student use

Purpose:
Implement A* for a 2d ground vehicle.

The output should be a path of Nx3 waypoints that consist of [x, y, yaw].
Remember to  check to see if the robot collidies with a block from above.

It is up to your team to decide which directions in your graph to check (4 cardinal directions or 8 directions that include the moves along a 45degree angle in the graph).

You are welcome to implement any helper functions that you need. If you use additional files to do this please make sure to submit them. 
"""
 
from heapq import heappush, heappop
import numpy as np
import math
from copy import deepcopy
 
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
 
from loadmap import loadmap
from detect_collision import *
 

def Astar(map, start, goal, resolution=np.array([5, 5, 5]), z_level=1.0, r_radius = 0.1, turn_weight=None):
    """
    Parameters:
        map,         struct containing map information
        start,       [x, y, yaw]  (yaw in radians)
        goal,        [x, y, yaw]
        resolution,  grid resolution (x, y, z)
        z_level,     the height of the robot to check collision with an upper boundary
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
    resolution = np.asarray(resolution, dtype=float)
 
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
 
    occ_map = OccupancyMap(map, resolution, 0.5, [map.boundary[0], map.boundary[1], map.boundary[2]])
    print("Finished Loading in Occupancy Map")

    # XXX: STUDENT CODE GOES HERE
    
    path = None

    # XXX: STUDENT CODE ENDS HERE
    
    return(path)
 
 
class OccupancyMap:
 
    def __init__(self, map, resolution, radius, origin):
        """
        Initialize Occupancy map. This would create the grid map based on the dimension and resolutions
        defined in the map file. It would then load the blocks and mark their corresponding grid as True
         as occupied.
        :param map: the map struct.
        """
        self.resolution = resolution
        self.origin = origin
 
        self.blocks = map.obstacles
        self.InflatedBlocks = BlockInflation(self.blocks, radius)
        self.boundary = map.boundary
 
        x_val = np.arange(self.boundary[0], self.boundary[3], resolution[0])
        y_val = np.arange(self.boundary[1], self.boundary[4], resolution[1])
        z_val = np.arange(self.boundary[2], self.boundary[5], resolution[2])

        # The occupancy grid at your specified resolution
        self.occ = np.zeros([len(x_val) + 1, len(y_val) + 1, len(z_val) + 1])

        # A dictionary containing if an index has been checked (since our graph is static)
        self.occ_check = {}
 
    def metric_to_index(self, metric):
        occ_index = (np.array(metric) - np.array(self.origin)) / self.resolution
        index = np.round(occ_index)
        return( index.astype('int') )
 
    def index_to_metric_center(self, ind):
        world_index = self.index_to_metric_negative_corner(ind) + np.array(self.resolution) / 2.0
        return( world_index )
 
    def index_to_metric_negative_corner(self, ind):
        idx = ind * np.array(self.resolution) + np.array(self.origin)
        return( idx)
 
    def is_occupied_index(self, ind, r):
        ind = tuple(int(i) for i in ind)
        if ind not in self.occ_check:
            self.process_index(ind, r)
        return( self.occ[ind] )
 
    def process_index(self, ind, r):
        """
        Marks the cell occupied if the robot, placed at the cell's metric
        position, collides with any inflated block.
        """
        robot_pose = self.index_to_metric_negative_corner(np.array(ind))
        # XXX: STUDENT CODE HERE
        # build a robot bounding box

        # for each block in the self.InflatedBlocks list
        # Check if the robot has collided with the block
        # If a collision happens then set the occ index to True and break out of loop

        # END STUDENT CODE
        
        self.occ_check[ind] = True
 
    def is_valid_index(self, ind):
        return (0 <= ind[0] < self.occ.shape[0] and
                0 <= ind[1] < self.occ.shape[1] and
                0 <= ind[2] < self.occ.shape[2])
 
if __name__ == "__main__":
    map_struct = loadmap("maps/map2.txt")
    start = np.array([0.0, 0.0, 1.57])
    goal = np.array([8.0, 5.0, 0.0])
    resolution = np.array([0.2, 0.2, 0.2])
    path = Astar(map_struct, start, goal, resolution)

    # XXX: DO MORE TESTING HERE
    
