#  Copyright (c) 2023 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

NEUTRAL_POSE = {
    'fr3_joint1': 0.0,
    'fr3_joint2': -0.785398,
    'fr3_joint3': 0.0,
    'fr3_joint4': -2.356194,
    'fr3_joint5': 0.0,
    'fr3_joint6': 1.570796,
    'fr3_joint7': 0.785398,
    'fr3_finger_joint1': 0.0,
    'fr3_finger_joint2': 0.0
}

def robot_state_publisher_spawner(context: LaunchContext, robot_type, load_gripper, ee_id):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    ee_id_str = context.perform_substitution(ee_id)
    franka_xacro_filepath = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        robot_type_str,
        robot_type_str + '.urdf.xacro',
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath, mappings={'hand': load_gripper_str,
                                         'ee_id': ee_id_str},
    ).toprettyxml(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
    ]


def generate_launch_description():
    load_gripper_parameter_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)

    ee_id_parameter_name = 'ee_id'
    ee_id = LaunchConfiguration(ee_id_parameter_name)

    robot_type_parameter_name = 'robot_type'
    robot_type = LaunchConfiguration(robot_type_parameter_name)

    rviz_file = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz',
        'visualize_franka.rviz',
    )

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_state_publisher_spawner, args=[robot_type, load_gripper, ee_id]
    )

    msg_dict = {
        "header": {
            "stamp": now,
            "frame_id": ""
        },
        "name": [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", 
            "fr3_joint4", "fr3_joint5", "fr3_joint6", 
            "fr3_joint7", "fr3_finger_joint1", "fr3_finger_joint2"
        ],
        "position": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04],
        "velocity": [],
        "effort": []
    }
    msg_data = json.dumps(msg_dict)

    raw_command = """ros2 topic pub -l /joint_states sensor_msgs/msg/JointState \
    "{header: {stamp: now, frame_id: ''}, name: ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7', 'fr3_finger_joint1', 'fr3_finger_joint2'], position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04]}"
    """
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value='true',
                description='Use end-effector if true. Default value is franka hand. '
                'Robot is loaded without end-effector otherwise',
            ),
            DeclareLaunchArgument(
                ee_id_parameter_name,
                default_value='franka_hand',
                description='ID of the type of end-effector used. Supported values: '
                'none, franka_hand, cobot_pump',
            ),
            DeclareLaunchArgument(
                robot_type_parameter_name,
                description='ID of the type of robot used. Supported values: '
                'fer, fr3, fp3, fr3v2, fr3v2_1, tmrv0_2, fr3v2_duo, mobile_fr3_duo_v0_2',
            ),

            robot_state_publisher_spawner_opaque_function,
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
            )
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['--display-config', rviz_file],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{
                    'source_list': [], # Read from other sources if needed
                    'dependent_joints': {
                        'fr3_joint1': {'hold': 0.0},
                        'fr3_joint2': {'hold': -0.785},
                        'fr3_joint3': {'hold': 0.0},
                        'fr3_joint4': {'hold': -2.356},
                        'fr3_joint5': {'hold': 0.0},
                        'fr3_joint6': {'hold': 1.571},
                        'fr3_joint7': {'hold': 0.785},
                        'fr3_finger_joint1': {'hold': 0.04},
                        'fr3_finger_joint2': {'hold': 0.04}
                    }
                }],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[raw_command
                    ],
                shell=True
                output='screen'
            ),
        ]
    )
