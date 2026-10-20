"""
Author/ Responsible Party: Victoria Edwards
Date: 10/20/2026

AI USAGE: AI was used for other parts of this assignment but most of this testing is compiled of work already done

Purpose: Compare the performance of astar or rrt

"""
import numpy as np
import time
from copy import deepcopy

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from loadmap import loadmap

import astar
import rrt

def plot_results(map, start, goal, path_astar, path_rrt):
    """
    Plot the results of the two different path planning methods

    Note: While A* will return the same path each time (up to your choice of
    heuristic) rrt will return different paths. It is good practice to show many
    examples of rrt paths
    """

    # XXX: STUDENT CODE GOES HERE
    return()
    

if __name__ == "__main__":
    # Compute path planning statistics
    # Notice this does not rely on ROS and can be done independently of that tool.
    # Check your method across different map sizes
    pass
