from math import pi
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
 
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations  # ROS2 port of tf.transformations
 
from arm_interface import ArmController
from calcJacobian import calcJacobian 
 
class VelocityVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer")
 
        #########################
        ##  RViz Communication ##
        #########################
        self.twist_pub = self.create_publisher(TwistStamped, '/vis/twist', 10)
        self.joint_pub = self.create_publisher(TwistStamped, '/vis/jointvel', 10)
 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
 
    # rotate vector v by quaternion q
    @staticmethod
    def qv_mult(q, v):
        v = [v[0], v[1], v[2], 0]
        return tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(q, v),
            tf_transformations.quaternion_conjugate(q)
        )[:3]

    def shutdown(self):
        self._executor.shutdown()
        self._spin_thread.join(timeout=1.0)
    
    # Publishes the linear and angular velocity of a frame on the corresponding topic
    def show_twist(self, pub, velocity, frame):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist.linear.x = velocity[0]
        msg.twist.linear.y = velocity[1]
        msg.twist.linear.z = velocity[2]
        msg.twist.angular.x = velocity[3]
        msg.twist.angular.y = velocity[4]
        msg.twist.angular.z = velocity[5]
        pub.publish(msg)
 
    # Publishes the velocity of the end effector on the corresponding topic
    def show_end_effector_velocity(self, velocity):
        print(velocity)
        self.show_twist(self.twist_pub, velocity, 'fr3_hand_tcp')
 
    # Publishes the velocity of a given joint on the corresponding topic
    def show_joint_velocity(self, qdot, i):
        print(qdot, i)
        # joints are always along z axis
        self.show_twist(self.joint_pub, qdot[i] * np.array([0, 0, 0, 0, 0, 1]), 'fr3_link' + str(i))
 
    # Uses the above methods to visualize the joint velocities and end effector velocity using your jacobian
    def show_all_velocity(self, q, i):
        qdot = np.zeros(7)
        qdot[i] = 1
        J = calcJacobian(q)
        velocity = J @ qdot
 
        # frame conversion
        try:
            t = self.tf_buffer.lookup_transform('base', 'fr3_hand_tcp', Time())
            #self.get_logger().info(f'TF lookup succeeded: {t}')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            #self.get_logger().warn(f'TF lookup failed: {type(e).__name__}: {e}')
            return

        rot = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ]
        
        quat = tf_transformations.quaternion_conjugate(rot)
        velocity[0:3] = self.qv_mult(quat, velocity[0:3])
        velocity[3:6] = self.qv_mult(quat, velocity[3:6])
 
        self.show_end_effector_velocity(velocity)
        self.show_joint_velocity(qdot, i)
 
 
#####################
##  Configurations ##
#####################
# TODO: Try testing other configurations!
# The first configuration below matches the dimensional drawing in the handout
configurations = [
    [0,    0,     0, -pi / 2,     0, pi / 2, pi / 4, 0.0, 0.0],
    [pi / 2, 0,  pi / 4, -pi / 2, -pi / 2, pi / 2,    0, 0.0, 0.0],
    [0,    0, -pi / 2, -pi / 4,  pi / 2, pi,   pi / 4, 0.0, 0.0],
]
 
 
####################
## Test Execution ##
####################
def main():
    rclpy.init()
    node = VelocityVisualizer()

    arm = ArmController()
 
    # Iterates through the given configurations, visualizing the velocities
    # Try editing the configurations list above to do more testing!
    for i, q in enumerate(configurations):
        print("Moving to configuration " + str(i) + "...")
        arm.move_position(q)
        # iterate thru each joint, activating each, one at a time
        for j in range(7):
            node.show_all_velocity(q, j)
            if j < 6:
                input("Press Enter to move to next joint...")
        if i < len(configurations) - 1:
            input("Press Enter to move to next configuration...")
 
    print("Done!")
 
    node.destroy_node()
    node.shutdown()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 

