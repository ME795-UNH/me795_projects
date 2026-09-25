"""
Editor: <NAME>
Date: <DATE>

AI Usage Statement:

Purpose: In this code, you will implement velocity IK for the franka panda robot arm.

Acknowledgment: This code comes from the MEAM5200 course at UPenn. Credit is given to the teaching team who developed this assignment.
"""

import numpy as np
from calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE

    dq = np.zeros(7)

    return dq

if __name__ == "__main__":
    pass
