#!/usr/bin/env python3
"""Bare-bones TurtleBot 4 sim: Gazebo + custom block world + robot + RViz.
 
Edit PKG_NAME / WORLD_FILE below (WORLD_FILE is only the default), then:
    ros2 launch turtlebot turtlebot4_block_world.launch.py
    ros2 launch turtlebot turtlebot4_block_world.launch.py x:=-2.0 y:=-2.0 yaw:=0.0
    ros2 launch turtlebot turtlebot4_block_world.launch.py world:=map2.sdf
    ros2 launch turtlebot turtlebot4_block_world.launch.py world:=/path/to/any_map.sdf
"""
 
import os
from pathlib import Path
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
 
PKG_NAME = 'turtlebot'   # package that installs the worlds/ folder
WORLD_FILE = 'block_world.sdf'  # default world when no world:= argument is given
RVIZ_FILE = 'navigation.rviz'
 
def generate_launch_description():
    worlds_dir = os.path.join(get_package_share_directory(PKG_NAME), 'worlds')
    tb4_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
 
    # Lets Gazebo resolve the robot's meshes (model:// / package:// URIs)
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            str(Path(get_package_share_directory('turtlebot4_description')).parent),
            str(Path(get_package_share_directory('irobot_create_description')).parent),
        ]))
 
    # Gazebo, running, with the world from the 'world' argument. The value is resolved
    # to an absolute path (so no Fuel lookup) and checked before Gazebo starts.
    def start_gazebo(context):
        requested = LaunchConfiguration('world').perform(context)
        candidates = [os.path.abspath(os.path.expanduser(requested)),
                      os.path.join(worlds_dir, requested)]
        world = next((c for c in candidates if os.path.isfile(c)), None)
        if world is None:
            raise RuntimeError(
                f'World file not found: "{requested}". Tried: ' + ', '.join(candidates))
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'{world} -r -v 2'}.items())]
 
    gazebo = OpaqueFunction(function=start_gazebo)
 
    # /clock so use_sim_time nodes work
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    rviz_config = os.path.join(get_package_share_directory(PKG_NAME), 'rviz', RVIZ_FILE)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # Stock spawn launch: robot_state_publisher, robot + dock spawn, ROS-Gazebo bridge,
    # Create 3 / TurtleBot 4 nodes (cmd_vel, odom, tf), and RViz
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_bringup, 'launch', 'turtlebot4_spawn.launch.py'])),
        launch_arguments={
            'spawn_dock': 'false',
            'rviz': 'false',
            'world': LaunchConfiguration('world_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())


    remove_dock = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/block_world/remove',
             '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', 'name: "standard_dock" type: MODEL'],
        output='screen')

    remove_dock_timer = TimerAction(period=10.0, actions=[remove_dock])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(worlds_dir, WORLD_FILE),
            description='World .sdf: absolute path, path relative to the current '
                        'directory, or a file name in the package worlds/ folder'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('world_name', default_value='block_world',
                      description='Name in the SDF <world name="..."> tag (no dots, no .sdf)'),
        resource_path,
        gazebo,
        clock_bridge,
        robot,
        rviz,
        remove_dock_timer,
    ])
