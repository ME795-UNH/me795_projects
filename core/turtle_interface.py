#!/usr/bin/env python3
"""
Author / Responsible Party: Victoria Edwards
Contact: Victoria.Edwards@unh.edu
Date: 10/20/2026

Comment: 
This code was written with Claude Sonnet 5 and then heavily modified by myself to help expose students in a controlled way to ROS2. 

Subscribes:
    odom                    (nav_msgs/Odometry)  robot pose feedback in odom frame
    sim_ground_truth_pose   (nav_msgs/Odometry)  ground truth of robot in world frame

Publishes:
    cmd_vel        (geometry_msgs/TwistStamped, or Twist if use_stamped:=false)
    path_complete  (std_msgs/Bool)  True when the last waypoint is reached, False when a

Behavior: for each waypoint the robot turns in place if it is pointing far from the target, otherwise drives toward it while steering. Publishing a new path replaces the current one; publishing an empty path stops the robot.
"""

import math
import time
import threading
import numpy as np


import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

def wrap_angle(a):
    """
    Helper function to safely handle the wrapping of angles 
    """
    return math.atan2(math.sin(a), math.cos(a))

def clamp(x, lo, hi):
    """
    Helper function to ensure that x does not exceed the min or max value
    """
    return max(lo, min(hi, x))

def yaw_from_quaternion(q):
    """
    Helper function to take quaternion and compute the yaw angle
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class TurtleController(Node):
    def __init__(self):
        # This inherits all the properties of the Node class and supplies the argument 'waypoint_controller'
        super().__init__('waypoint_controller')

        defaults = {
            'goal_tolerance': 0.05,        # m, to count a waypoint as reached
            'max_linear': 0.30,            # m/s
            'max_angular': 1.0,            # rad/s
            'k_linear': 0.0,               # XXX: STUDENT TODO
            'k_angular': 0.0,              # XXX: STUDENT TODO
            'turn_in_place_angle': 0.0,    # XXX: STUDENT TODO
            'align_final_heading': False,  # turn to the last pose's yaw at the end
            'yaw_tolerance': 0.10,         # rad
            'control_rate': 20.0,          # Hz
            'odom_timeout': 1.0,           # s; stop if odometry goes stale
            'pose_source': 'odom', # ground truth or odom
        }
        
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.p = p

        # Create publishers
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
            
        self.done_pub = self.create_publisher(
            Bool, 'path_complete',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        # Best-effort subscription matches both reliable and best-effort odom publishers
        if p['pose_source'] == 'ground_truth':
            self.create_subscription(Odometry, 'sim_ground_truth_pose', self.gt_cb,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Odometry, 'odom', self.odom_cb, qos_profile_sensor_data)

        self.pose_odom = None            # (x, y, yaw) 
        self.odom_frame = ''
        self.last_odom_ns = 0

        self.pose_gt = None            # (x, y, yaw) 
        self.gt_frame = ''
        self.last_gt_ns = 0

        self.pose = None

        # XXX: STUDENT ADD LIST FOR POSES

        self.waypoints = []         # list of (x, y, yaw)
        self.idx = 0
        self.active = False
        self.aligning = False

        self.get_logger().info('Waypoint controller ready: send a nav_msgs/Path to "waypoints".')

        # This helps set consistent message properties 
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
        

    # ---------------------------------------------------------------- callbacks
    def odom_cb(self, msg):
        """
        msg: a nav_msgs/Odometry message
        https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        
        Objective: store the pose, frame, and time from the message

        Hint 1: You need to set the variables self.pose_odom, self.odom_frame, self.last_odom_ns
        Hint 2: To get yaw, use the function yaw_from_quaternion written above. The message returns
               orientation as a quaternion which is another representation of a rotation that we do not cover

        To see messages published run: 
        $ ros2 topic echo /odom

        To see information about this topic:
        $ ros2 topic info /odom -v

        Remember: store the pose information in the list self.poses
        """
        ## XXX: STUDENT CODE GOES HERE 
        self.pose = self.pose_odom
    
    def gt_cb(self, msg):
        """
        msg: a nav_msgs/Odometry message


        Hint 1: You need to set the variables self.pose_gt, self.gt_frame, self.last_gt_ns
        Hint 2: To get yaw, use the function yaw_from_quaternion written above. The message returns
               orientation as a quaternion which is another representation of a rotation that we do not cover

        To see messages published run: 
        $ ros2 topic echo /odom

        To see information about this topic:
        $ ros2 topic info /odom -v

        Remember: store the pose information in the list self.poses
        
        """
        ## XXX: STUDENT CODE GOES HERE 
        self.pose = self.pose_gt

    def wait_for_pose(self, timeout_sec=15.0):
        """
        This function ensures that we have sensor data before we try to do control
        """
        end = time.monotonic() + timeout_sec
        while rclpy.ok() and self.pose is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def set_path(self, path):
        """
        path: list of N x 3 waypoints of the form (x, y, yaw)

        Set the class specific properties to be used by the controller 
        """
        
        path = np.asarray(path, dtype=float).reshape(-1, 3)   # rows of x, y, yaw
        self.waypoints = [tuple(w) for w in path]

        # start waypoint id
        self.idx = 0
        
        self.aligning = False
        self.active = len(self.waypoints) > 0
        
        self.done_pub.publish(Bool(data=False))

        # make sure if the path is not long enough we do not start trying to do control 
        if not self.active:
            self.stop()

    def spin_for(self, duration):
        """
        Process callbacks (odom) for `duration` seconds, then return.
        This is what will allow us to write the controller in this way, which is similar to how the arm_interface.py works
        """
        end = time.monotonic() + duration
        while rclpy.ok():
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=remaining)

    def follow_path(self, path, timeout=None):
        """
        Blocking. Returns True if the path finished, False on timeout/shutdown.
        """
        self.set_path(path)
        period = 1.0 / self.p['control_rate']
        t0 = time.monotonic()
        while rclpy.ok() and self.active:
            self.spin_for(period)        # 1) update self.pose 
            self.control_loop()          # 2) one control step using the newest pose
            if timeout is not None and time.monotonic() - t0 > timeout:
                self.active = False
                self.stop()
                return False
        return not self.active and rclpy.ok()

    def control_loop(self):
        """
        This is the control loop for the waypoint controller

        As we discussed in class we will tradeoff between aligning and going straight

        Task: Implement the waypoint logic to determine the linear velocity, angular_velocity, and distance_to_goal

        Hint: Be careful with angle_wraping and checking for max and min velocities
        """
        if not self.active:
            return
        
        p = self.p
        age = (self.get_clock().now().nanoseconds - self.last_odom_ns) * 1e-9
        if self.pose is None or age > p['odom_timeout']:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn('No recent odometry: holding position.',
                                   throttle_duration_sec=2.0)
            return
        
        x, y, yaw = self.pose
        gx, gy, gyaw = self.waypoints[self.idx]

        # Check if this is the last waypoint
        if self.idx == len(self.waypoints) - 1:
            is_last = True
        else:
            is_last = False

        # XXX: STUDENT CODE GOES HERE
        v = 0
        w = 0

        # These are steps for a P-controller, if you want to use I and D terms you will need to add logic for that to work here
        
        # compute the euclidean distance between the two points (distance_to_goal)

        # compute the error between your current yaw and where you want to go ( be careful subtracting angles)
        # Hint: remember atan2(dy, dx) gives you the angle you need to move to get to where you want to go

        # Check if the angle error is greater than the tolerance see p
        # If it is then linear velocity is 0
        # Otherwise the linear velocity is the clamped value of k_lin * dist * math.cos(err)

        # compute the angular velocity using k_ang and make sure to clamp this value around max and min angular vel allowed.


        # STUDENT CODE ENDS HERE
        
        # Logic to publish the commands and update the waypoints
        if self.aligning:
            err = wrap_angle(gyaw - yaw)
            if abs(err) < p['yaw_tolerance']:
                self.finish()
            else:
                self.publish_cmd(0.0, clamp(p['k_angular'] * err,
                                            -p['max_angular'], p['max_angular']))
            return

        if dist < p['goal_tolerance']:
            self.get_logger().info(f'Reached waypoint {self.idx + 1}/{len(self.waypoints)}.')
            if not is_last:
                self.idx += 1
            elif p['align_final_heading']:
                self.aligning = True
            else:
                self.finish()
            return

        self.publish_cmd(v, w)

    def finish(self):
        """
        Clean up once the robot has achieved the path 
        """
        self.active = False
        self.aligning = False
        self.stop()
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info('Path complete.')

    # ------------------------------------------------------------------ output
    def publish_cmd(self, linear, angular):
        """
        linear: The desired linear velocity in x
        angualr: The desired angular velocity about z

        Objective: Publish a TwistStamped() message.

        Hint: Add 2 lines to the message to publish the linear and angular vels 
        """
        # Make message to be published
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        ## XXX: STUDENT CODE
        
        # publish the message 
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)





