"""
Author / responsible party: Victoria Edwards
Date: 10/20/2026

Purpose: 
Test code for building the ground robot waypoint controller

AI Usage: I used Claude Sonnet 5 to help with the building of various aspects of this project. What some of you may have seen is that generative tools can give a solution but the usability of that code by students / people who are new to a tool is not always easy.

I have modified and updated the code to serve our classes needs. 
"""

import sys
import math
import numpy as np
import time
import matplotlib.pyplot as plt

import rclpy

from turtle_interface import TurtleController

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    # A square in an empty map 
    path = np.array([[0, 0, 0],
                     [2, 0, 0],
                     [2, 2, 0],
                     [0, 2, 0],
                     [0, 0, 0]])
    try:
        if not node.wait_for_pose():
            raise RuntimeError("No pose data received")

        # This sets the path and updates the control loop
        # Originally, I wrote this code so that you would not need to interface so closely with the ROS2.
        # Based on feedback from the class I am now having you develop the turtle_interface.py code
        node.follow_path(path)

    finally:
        # XXX: STUDENT PLOT PATH AND ROBOT PATH

        
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
