"""
Author: Victoria Edwards
Date: 08/2026

Purpose: Allow for RViz visualization of the robot arm in different configurations for both FK and IK.

To run this code for Forward Kinematics type in a terminal:

$ python visualize.py FK

or with Inverse Kinematics type in a terminal:

$ python visualize.py IK

Acknowledgment: This code is modified from UPenn MEAM5200 Introduction to robotics. Credit is given to the teaching team who developed this work.

AI Usage Statement: I used AI to help convert this to ros2. In particular, this required moving the node into the class structure of ros2 and to update to the correct message types. 
"""

import sys
from math import pi
import numpy as np
 
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
 
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster
import tf_transformations
 
from arm_interface import ArmController
from calculateFK import FK
from PlanarIK import PlanarIK
 
 
class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
 
        # Using your solution code
        self.fk = FK()
        self.ik = PlanarIK()
 
        #########################
        ##  RViz Communication ##
        #########################
        self.tf_broad = TransformBroadcaster(self)
        self.point_pubs = [
            self.create_publisher(geometry_msgs.msg.PointStamped, '/vis/joint' + str(i), 10)
            for i in range(7)
        ]
 
    # Publishes the position of a given joint on the corresponding topic
    def show_joint_position(self, joints, i):
        msg = geometry_msgs.msg.PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.point.x = joints[i, 0]
        msg.point.y = joints[i, 1]
        msg.point.z = joints[i, 2]
        self.point_pubs[i].publish(msg)
 
    # Broadcasts a T0e as the transform from given frame to world frame
    def show_pose(self, T0e, frame):
        translation = tf_transformations.translation_from_matrix(T0e)
        quaternion = tf_transformations.quaternion_from_matrix(T0e)
 
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broad.sendTransform(t)
 
    # Uses the above methods to visualize the full results of your FK
    def show_all_FK(self, q):
        joints, T0e = self.fk.forward(q)
        self.show_pose(T0e, "endeffector")
        for i in range(7):
            self.show_joint_position(joints, i)
 
    # visualize the chosen IK target
    def show_target(self, target):
        x = target['o'][0]
        z = target['o'][1]
        theta = target['theta']
        T0_target = tf_transformations.translation_matrix(np.array([x, 0, z])) @ \
            tf_transformations.euler_matrix(0, -theta - pi / 2, pi)
        self.show_pose(T0_target, "target")
 
 
########################
##  FK Configurations ##
########################
# TODO: Try testing other configurations!
# The first configuration below matches the dimensional drawing in the handout
configurations = [
    [0,    0,     0, -pi / 2,     0, pi / 2, pi / 4, 0.0, 0.0],
    [pi / 2, 0,  pi / 4, -pi / 2, -pi / 2, pi / 2,    0, 0.0, 0.0],
    [0,    0, -pi / 2, -pi / 4,  pi / 2, pi,   pi / 4, 0.0, 0.0],
]
 
#################
##  IK Targets ##
#################
# TODO: Try testing your own targets!
targets = [
    {
        'o': np.array([0.1, 0.8]),
        'theta': 0
    },
    {
        'o': np.array([.4, .6]),
        'theta': pi / 4
    },
    {
        'o': np.array([0.4, -0.4]),
        'theta': pi / 2 + 0.2
    },
    {
        'o': np.array([.1, .3]),
        'theta': 5 * pi / 4
    }
]
 
####################
## Test Execution ##
####################
def main():
    if len(sys.argv) < 2:
        print("usage:\n\tpython visualize.py FK\n\tpython visualize.py IK")
        return
 
    rclpy.init()
    node = Visualizer()
 
    # ArmController is assumed to accept a node + callback in its ROS2 port.
    # Adjust this line to match your ArmController's ROS2 constructor signature.
    arm = ArmController(vis_node = node)
 
    if sys.argv[1] == 'FK':
        # Iterates through the given configurations, visualizing your FK solution
        # Try editing the configurations list above to do more testing!
        q = configurations[0]
        for i, q in enumerate(configurations):
            print("Moving to configuration " + str(i) + "...")
            #show_all_FK()
            arm.move_position(q)
            if i < len(configurations) - 1:
                input("Press Enter to move to next configuration...")
        arm.move_position(q)
 
    elif sys.argv[1] == 'IK':
        # Iterates through the given targets, using your IK solution
        # Try editing the targets list above to do more testing!
        for i, target in enumerate(targets):
            print("Moving to target " + str(i) + "...")
            node.show_target(target)
            solutions = node.ik.panda_ik(target)
            q = solutions[0, :].tolist()  # choose the first of multiple solutions
            arm.move_position(q)
            if i < len(targets) - 1:
                input("Press Enter to move to next target...")
 
    else:
        print("invalid option")
 
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
