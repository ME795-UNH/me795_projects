"""
Author / Responsible Party: Victoria Edwards
Date: 10/20/2026

AI Usage: I used Claude Sonnet 5 to update this solution for a ground vehicle.

Purpose:
Helper functions that could be used by RRT or A*. You do not have to use these functions if you do not want to. 
"""

import numpy as np
import math

def isRobotCollided(robot_pose, robot_bounding_box, InflatedBlocks):
    """
    Args:
        robot_pose: (3,) or (1, 3) array, reference point (x, y, z) of the robot
        robot_bounding_box: [xmin, ymin, zmin, xmax, ymax, zmax], relative to the
                            robot reference point (the pose origin)
        InflatedBlocks: (N, 6) array-like, rows [xmin, ymin, zmin, xmax, ymax, zmax]

    Returns: True if the robot collides with any block, else False
    """
    # XXX: STUDENT CODE GOES HERE IF CHOOSE TO UES THIS FUNCTION
    return()

def isPathCollided(q_sample, q, InflatedBlocks, steps= 30, r = 0.1):
    """

    Args:
        q_sample: random sample in free C-space
        q: q_a or q_b, closet node in start or goal tree
        InflatedBlocks:  blocks with inflation
        steps: number of pieces dividing the path from q_sample to q, set default value to 30
        r: the robot radius used for a bounding box
    
    Returns: bool, True if path between q_sample and q collides with obstacles, otherwise False

    """
    q_steps = [q_sample]
    # XXX: STUDENT CODE GOES HERE IF CHOOSE TO UES THIS FUNTION
    # generate a series of steps from q to the q_sample
    # check if any of those steps intersects with a block return True
    # otherwise return False
    
    return()

def BlockInflation(blocks, radius):
    """
    Inflate the given blocks to account for the robot, returns an (n, 6) array.
    """
    modifier = np.array([-radius, -radius, -radius, radius, radius, radius])
    return np.array([modifier + block for block in blocks])

