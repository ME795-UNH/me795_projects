"""
Author: Victoria Edwards
Date: 08/2026

Purpose: Look at hte workspace of our robot arm. 

To run this code call: 
$ python workspace.py

Acknowledgment: This code is modified from UPenn MEAM5200 Introduction to robotics. Credit is given to the teaching team who developed this work.

AI Usage Statement: I did not use AI for this code.
"""

from calculateFK import FK
from arm_interface import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


if __name__ == "__main__":
    fk = FK()

    # the dictionary below contains the data returned by calling arm.joint_limits()
    limits = [
        {'lower': -2.8973, 'upper': 2.8973},
        {'lower': -1.7628, 'upper': 1.7628},
        {'lower': -2.8973, 'upper': 2.8973},
        {'lower': -3.0718, 'upper': -0.0698},
        {'lower': -2.8973, 'upper': 2.8973},
        {'lower': -0.0175, 'upper': 3.7525},
        {'lower': -2.8973, 'upper': 2.8973}
    ]

    # TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
    # accounting for the joint limits.
    #
    # We've included some very basic plotting commands below, but you can find
    # more functionality at https://matplotlib.org/stable/index.html
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # TODO: update this with real results
    ax.scatter(1,1,1) # plot the point (1,1,1)
    
    plt.show()
