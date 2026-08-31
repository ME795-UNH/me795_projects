"""
Date: 07/24/2026

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

** Note **: I am taking advantage of the word controller here. The ArmController class is doing a visualization trick to move the arm in RViz but is not directly going to a controller that is using physics. For our purposes of doing FK and IK in simulation this is fine. If we wanted to improve this we would look at ros2_control package. Ask me about this. I have a preliminary version working that ultimately I felt was not stable enough for class this year.

** Note **: Because we are not using a controller it is possible to command positions outside of the feasible workspace of the arm! Be careful and thoughtful about picking valid configurations. Food for thought: what happens if you command a configuration the robot cannot reach with a controller?

Joint Limits in radians:
joint_1: [-2.8973, 2.8973]
joint_2: [-1.7628, 1.7628]
joint_3: [-2.8973, 2.8973]
joint_4: [-3.0718, -0.0698]
joint_5: [-2.8973, 2.8973]
joint_6: [-0.0175, 3.7525]
joint_7: [-0.0175, 3.7525]

Try changing the target position to see what the arm does!

"""

import math
import rclpy
from arm_interface import ArmController
import time
 
def main(args=None):
    rclpy.init(args=args)
 
    arm = ArmController() 
    
    try:
        # Set the desired joint value (see limits above)
        joints = [0,-1 ,0,-2,0,1,1, 0, 0]

        # set desired duration to move to that position (Remember no controller)
        dur = 4.0

        # Move to the joints provided in the time set.
        # Question: If you make duration very small or very large what happens?
        #           How would a controller take care of this for you? 
        arm.move_position(joints, dur)
        
        # Pause before next command
        time.sleep(1)

        # Neutral or home position
        arm.go_home()
        time.sleep(1)

        # One more example position
        joints = [0.5,-1 ,1,-0.5,0,1,1.5, 0.07, 0.07]
        dur = 4.0
        arm.move_position(joints, dur)
        time.sleep(1)

        arm.go_home()
        time.sleep(1)
        
        # Pause before next command
        time.sleep(1)

        # Try additional configurations here!
        # These will go in your report
        

        arm.open_gripper()
        arm.close_gripper()
        
    finally:
        arm.destroy_node()
 
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
