#!/usr/bin/env python3
"""
Author / Responsible Party: Victoria Edwards
Contact: Victoria.Edwards@unh.edu
Date: 08/26/2026

Comment: This code was written with the aid of Claude, Sonnet 5. I ported this from an original ROS 1 package that is heavily simplified to only do what is needed for this class.As we will discuss in class, this is not a controller. The focus of this portion of the class is learning about coordinate frames and how to understand where the robot is in the world using Forward and Inverse Kinematics. The second half of the semester we will work with a Turtle Robot that will have a controller, sensing, and other features that more closely mimic a physical robot. 

File Name: arm_interface.py

From Claude: 
ROS2 Python wrapper to move a Franka Panda arm to a target joint
configuration, with actual interpolated motion.
 
  rviz    - No controller/action server. Interpolates from the
            arm's current /joint_states to the target and publishes
            /joint_commands at a fixed rate, so RViz shows smooth motion
            instead of a teleport. This is a visualization trick only --
            no physics, no real controller involved.

How you will see this in your code:

import ArmController

This will import the pythan package into the active file. ***If you ever get an error that says arm_interface is not recognized, this is because the python path is not configured correctly. Please email Torrie for help with this. ***
"""
 
import argparse
import sys
import subprocess
import time
from typing import List, Optional
 
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
 

DEFAULT_JOINT_NAMES =[
    'fr3_joint1',
    'fr3_joint2',
    'fr3_joint3',
    'fr3_joint4',
    'fr3_joint5',
    'fr3_joint6',
    'fr3_joint7',
    'fr3_finger_joint1',
    'fr3_finger_joint2']
 
# Common Franka "ready" pose, useful as a safe default target
READY_POSE = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398, 0.0, 0.0]

def kill_process_by_name(name: str) -> bool:
    """
    OS-level 'kill another node' helper. ROS2 has no built-in API for one
    node to kill another; this just finds process(es) whose command line
    matches `name` and sends SIGTERM via `pkill -f`. Useful for clearing
    out a `joint_state_publisher` that was only meant to be temporary,
    before this script starts publishing to the same /joint_states topic.
 
    Returns True if at least one matching process was found and signaled.
    """
    result = subprocess.run(["pkill", "-f", name], capture_output=True, text=True)
    # pkill: 0 = killed something, 1 = no matches, >1 = real error.
    if result.returncode == 0:
        print(f"Killed process(es) matching '{name}'.")
        return True
    elif result.returncode == 1:
        print(f"No running process matched '{name}'.")
        return False
    else:
        print(f"pkill error (code {result.returncode}): {result.stderr.strip()}")
        return False
    

class ArmController(Node):
    """
    For RViz-only setups with no controller/action server: fakes visible
    motion by interpolating from the current /joint_states to the target
    and republishing at a fixed rate. No physics, no real controller.
    """
 
    def __init__(self, joint_names: Optional[List[str]] = None, vis_node: [Optional[Node]] = None):
        super().__init__("rviz_position_interpolator")
        
        self.joint_names = joint_names or DEFAULT_JOINT_NAMES

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)

        self._current_positions: Optional[List[float]] = None

        self._js_sub = self.create_subscription(JointState, "/current_joint_states", self._joint_state_cb, 10)

        self.vis_node = vis_node

        self.move_position(READY_POSE)
        self._current_positions = READY_POSE
        self._cur_velocities = [0.0] * 9

    def _joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in self.joint_names):
            self._current_positions = [name_to_pos[j] for j in self.joint_names]
            
        name_to_pos = dict(zip(msg.name, msg.velocity))
        if all(j in name_to_pos for j in self.joint_names):
            self._cur_velocities = [name_to_pos[j] for j in self.joint_names]

    def get_position(self):
        return(self._current_positions)

    def get_velocity(self):
        return(self._cur_velocities)

    def neutral_position(self):
        return(READY_POSE)

    def open_gripper(self):
        gripper_pose = self._current_positions
        gripper_pose[-1] = 0.05
        gripper_pose[-2] = 0.05
        self.move_position(gripper_pose, duration_sec = 7.0)

    def close_gripper(self):
        gripper_pose = self._current_positions
        gripper_pose[-1] = 0.0
        gripper_pose[-2] = 0.0
        self.move_position(gripper_pose, duration_sec = 7.0)

    def go_home(self):
        self.move_position(READY_POSE)
    
    def wait_for_joint_state(self, timeout_sec: float = 3.0) -> bool:
        end = time.time() + timeout_sec
        while rclpy.ok() and self._current_positions is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            # Kill the other joint_state_publisher
            #kill_process_by_name("joint_state_publisher")
        return self._current_positions is not None
 
    def move_position(self, target_positions: List[float], duration_sec: float = 4.0, rate_hz: float = 50.0):
        self.wait_for_joint_state()
        
        start = self._current_positions or [0.0] * len(self.joint_names)
        if self._current_positions is None:
            self.get_logger().warn("No /joint_states received; assuming all-zero start.")
 
        steps = max(int(duration_sec * rate_hz), 1)
        dt = 1.0 / rate_hz

        for i in range(steps + 1):
            t = i / steps
            positions = [s + (g - s) * t for s, g in zip(start, target_positions)]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = positions
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
            self._current_positions = positions

            if self.vis_node != None:
                self.vis_node.show_all_FK(self._current_positions)
                
 
        self.get_logger().info("Interpolated motion complete.")
        self._current_positions = positions

    def set_joint_positions_velocities(self,  target_positions: List[float], target_vel: List[float], duration_sec: float = 4.0, rate_hz: float = 50.0):
        
        self.wait_for_joint_state()
        
        start = self._current_positions or [0.0] * len(self.joint_names)
        start_vel = self._cur_velocities or [0.0] * len(self.joint_names)
        if self._current_positions is None:
            self.get_logger().warn("No /joint_states received; assuming all-zero start.")
 
        steps = max(int(duration_sec * rate_hz), 1)
        dt = 1.0 / rate_hz

        for i in range(steps + 1):
            t = i / steps
            positions = [s + (g - s) * t for s, g in zip(start, target_positions)]
            velocities = [s + (g - s) * t for s, g in zip(start_vel, target_vel)]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = positions
            msg.velocity = velocities
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
            self._current_positions = positions
            self._cur_velocities = velocities
 
        self.get_logger().info("Interpolated motion complete.")
        self._current_positions = positions
        self._cur_velocities = velocities
        
 
def parse_args(argv):
    """ Useful if you are calling this function directly"""
    
    parser = argparse.ArgumentParser(description="Move Panda arm to a joint configuration.")
    parser.add_argument(
        "--mode", choices=["rviz"], default="rviz",
        help="gazebo: real FollowJointTrajectory motion. rviz: fake interpolation, no controller needed.",
    )
    parser.add_argument(
        "--joints", type=float, nargs=9, default=READY_POSE,
        help="9 target joint positions (radians), panda_joint1..9",
    )
    parser.add_argument("--duration", type=float, default=4.0, help="Motion duration (s)")
    parser.add_argument(
        "--action-name", type=str,
        default="/panda_arm_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action server name (gazebo mode only)",
    )
    return parser.parse_args(argv)
 
 
def main(argv=None):
    rclpy.init(args=argv)
    args = parse_args(sys.argv[1:] if argv is None else argv)
 
    interp = ArmController()
    try:
        interp.move_position(args.joints, duration_sec=args.duration)
    finally:
        interp.destroy_node()
 
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
