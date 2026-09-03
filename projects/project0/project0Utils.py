"""
This code was built for MEAM520 at the University of Pennsylvania in the Fall of 2021. I have modified it for our class purposes.

Editor: Victoria Edwards
Date: 09/02/2026

Instructions:
   - Implement the following three functions. 
   - There is simple test code at the bottom. Add to this test code as needed to help you debug your code.

   - To run the code, go to your terminal and type:
       $ cd me795_projects/projects/project0/
       $ python project0Utils.py

   - The outputs when you do the following in your terminal will be:
      [2.22044605e-16 5.00000000e-01]
       1.5707963267948966
      [5.00831259 4.40830292 2.93136762]

   - Consider what other test cases you could run that would help you know if your code is correct
"""

import numpy as np


def linear_solver(A, b):
    """
    Solve for x Ax=b. Assume A is invertible.
    Args:
        A: nxn numpy array
        b: 0xn numpy array

    Returns:
        x: 0xn numpy array
    """

    
    # Student code goes here


    return()

def angle_solver(v1, v2):
    """
    Solves for the magnitude of the angle between v1 and v2
    Args:
        v1: 0xn numpy array
        v2: 0xn numpy array

    Returns:
        theta = scalar >= 0 = angle in radians
    """

    # Student code goes here


    return()

def linear_euler_integration(A, x0, dt, nSteps):
    """
    Integrate the ode x'=Ax using euler integration where:
    x_{k+1} = dt (A x_k) + x_k
    Args:
        A: nxn np array describing linear differential equation
        x0: 0xn np array Initial condition
        dt: scalar, time step
        nSteps: scalar, number of time steps

    Returns:
        x: state after nSteps time steps (np array)
    """

    # Student code goes here


    return()

if __name__ == '__main__':
    # Example call for linear solver
    A = np.array([[1, 2], [3, 4]])
    b = np.array([1, 2])
    print(linear_solver(A, b))

    # Example call for angles between vectors
    v1 = np.array([1, 0])
    v2 = np.array([0, 1])
    print(angle_solver(v1, v2))

    # Example call for euler integration
    A = np.random.rand(3, 3)
    x0 = np.array([1, 1, 1])
    dt = 0.01
    nSteps = 100
    print(linear_euler_integration(A, x0, dt, nSteps))
