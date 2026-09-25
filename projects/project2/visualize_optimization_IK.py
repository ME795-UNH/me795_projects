import sys
from math import pi, sin, cos
import numpy as np
from time import perf_counter
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from arm_interface import ArmController

from solveIK import IK

#############################
##  Transformation Helpers ##
#############################

def trans(d):
    """
    Compute pure translation homogenous transformation
    """
    return np.array([
        [ 1, 0, 0, d[0] ],
        [ 0, 1, 0, d[1] ],
        [ 0, 0, 1, d[2] ],
        [ 0, 0, 0, 1    ],
    ])

def roll(a):
    """
    Compute homogenous transformation for rotation around x axis by angle a
    """
    return np.array([
        [ 1,     0,       0,  0 ],
        [ 0, cos(a), -sin(a), 0 ],
        [ 0, sin(a),  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def pitch(a):
    """
    Compute homogenous transformation for rotation around y axis by angle a
    """
    return np.array([
        [ cos(a), 0, -sin(a), 0 ],
        [      0, 1,       0, 0 ],
        [ sin(a), 0,  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def yaw(a):
    """
    Compute homogenous transformation for rotation around z axis by angle a
    """
    return np.array([
        [ cos(a), -sin(a), 0, 0 ],
        [ sin(a),  cos(a), 0, 0 ],
        [      0,       0, 1, 0 ],
        [      0,       0, 0, 1 ],
    ])

def transform(d,rpy):
    """
    Helper function to compute a homogenous transform of a translation by d and
    rotation corresponding to roll-pitch-yaw euler angles
    """
    return trans(d) @ roll(rpy[0]) @ pitch(rpy[1]) @ yaw(rpy[2])

#######################
##      ROS NODE     ##
#######################
class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")
        # Using your solution code
        self.ik = IK()
        self.tf_broad = TransformBroadcaster(self)
        
        
    #########################
    ##  RViz Communication ##
    #########################
        
    
    # Broadcasts a frame using the transform from given frame to world frame
    def show_pose(self, H, frame):
        # Create a blank TransformStamped container
        t = TransformStamped()

        # Populate the required header timestamps and frame names
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base"     # The parent coordinate frame
        t.child_frame_id = frame       # The child coordinate frame
        
        # Extract and assign translation elements from your matrix H [4x4]
        t.transform.translation.x = float(H[0, 3])
        t.transform.translation.y = float(H[1, 3])
        t.transform.translation.z = float(H[2, 3])
        
        # Extract and assign rotation elements from your matrix H [4x4]
        # Assuming you use standard scipy matrices for spatial transforms:
        r = Rotation.from_matrix(H[0:3, 0:3])
        q = r.as_quat() # Returns array format: [x, y, z, w]

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        # 5. Broadcast the complete message container
        self.tf_broad.sendTransform(t)

        
#################
##  IK Targets ##
#################

# TODO: Try testing your own targets!

# Note: below we are using some helper functions which make it easier to generate
# valid transformation matrices from a translation vector and Euler angles, or a
# sequence of successive rotations around z, y, and x. You are free to use these
# to generate your own tests, or directly write out transforms you wish to test.

targets = [
    transform( np.array([-.2, -.3, .5]), np.array([0,pi,pi])            ),
    transform( np.array([-.2, .3, .5]),  np.array([pi/6,5/6*pi,7/6*pi]) ),
    transform( np.array([.5, 0, .5]),    np.array([0,pi,pi])            ),
    transform( np.array([.7, 0, .5]),    np.array([0,pi,pi])            ),
    transform( np.array([.2, .6, 0.5]),  np.array([0,pi,pi])            ),
    transform( np.array([.2, .6, 0.5]),  np.array([0,pi,pi-pi/2])       ),
    transform( np.array([.2, -.6, 0.5]), np.array([0,pi-pi/2,pi])       ),
    transform( np.array([.2, -.6, 0.5]), np.array([pi/4,pi-pi/2,pi])    ),
    transform( np.array([.5, 0, 0.2]),   np.array([0,pi-pi/2,pi])       ),
    transform( np.array([.4, 0, 0.2]),   np.array([pi/2,pi-pi/2,pi])    ),
    transform( np.array([.4, 0, 0]),     np.array([pi/2,pi-pi/2,pi])    ),
]

####################
## Test Execution ##
####################

np.set_printoptions(suppress=True)

if __name__ == "__main__":
    rclpy.init()
    
    node = Visualizer()
    arm = ArmController()

    # Iterates through the given targets, using your IK solution
    # Try editing the targets list above to do more testing!
    for i, target in enumerate(targets):
        print("Target " + str(i) + " located at:")
        print(target)
        print("Solving... ")
        node.show_pose(target,"target")

        seed = arm.neutral_position()[0:-2] # use neutral configuration as seed

        start = perf_counter()

        q, success, rollout = node.ik.inverse(target, seed)

        # Account for fingers in the message (nessary feature of ROS2)
        if isinstance(q, np.ndarray):
            q = q.tolist()
        q.extend([0.0, 0.0])
        
        stop = perf_counter()
        dt = stop - start

        if success:
            print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt,it=len(rollout)))
            arm.move_position(q)
        else:
            print('IK Failed for this target using this seed.')


        if i < len(targets) - 1:
            input("Press Enter to move to next target...")
