#!/usr/bin/env python3
"""
A node to help manage a constant tf tree
"""
 
import sys
 
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

"""
class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self._latest = None

        # continuous idle/default state
        self.create_subscription(JointState, '/current_joint_states', self._on_current, 10)
        # commanded moves, published by your third node
        self.create_subscription(JointState, '/joint_commands', self._on_command, 10)

    def _on_current(self, msg):
        # only forward "current" state if nothing has actively commanded a move
        #if self._latest is None:
        self.pub.publish(msg)

    def _on_command(self, msg):
        self._latest = msg
        self.pub.publish(msg)

"""
class JointStateBridge(Node):
    def __init__(self, publish_rate_hz: float = 50.0):
        super().__init__('joint_state_bridge')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self._latest_current = None
        self._latest_command = None

        self.create_subscription(JointState, '/current_joint_states', self._on_current, 10)
        self.create_subscription(JointState, '/joint_commands', self._on_command, 10)

        # continuously publish at a fixed rate, regardless of when inputs arrive
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest)

    def _on_current(self, msg):
        self._latest_current = msg

    def _on_command(self, msg):
        self._latest_command = msg

    def _publish_latest(self):
        # commands take priority over background current state
        msg = self._latest_command if self._latest_command is not None else self._latest_current
        if msg is None:
            return  # nothing received yet on either topic

        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
