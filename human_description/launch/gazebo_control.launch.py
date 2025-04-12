# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE_NAME = 'human_description'


def robot_nodes_spawner(context: LaunchContext, robot_id, command_interface, load_groups, base_parent_link, runtime_config_package, controllers_file, use_sim_time, log_level):
    robot_id_str = context.perform_substitution(robot_id)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(PACKAGE_NAME),
                 "robots", robot_id_str, robot_id_str + ".urdf.xacro"]
            ),
            " ",
            "ros2_control_plugin:=",
            "gazebo",
            " ",
            "command_interface:=",
            command_interface,
            " ",
            "base_parent_link:=",
            base_parent_link,
            # "load_groups:=",
            # load_groups,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            robot_id_str,
            controllers_file,
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[robot_controllers, robot_description,
                    {'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0', '-Y', '3.1416',
                   "--ros-args",
                   "--log-level",
                   log_level,],
        output='screen',
        remappings=[
            ("motion_control_handle/target_frame", "target_frame"),
            ("cartesian_motion_controller/target_frame", "target_frame"),
            ("cartesian_compliance_controller/target_frame", "target_frame"),
            ("cartesian_force_controller/target_wrench", "target_wrench"),
            ("cartesian_compliance_controller/target_wrench", "target_wrench"),
            ("cartesian_force_controller/ft_sensor_wrench", "ft_sensor_wrench"),
            ("cartesian_compliance_controller/ft_sensor_wrench", "ft_sensor_wrench"),
        ],
    )

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdf_file = PathJoinSubstitution(
        [
            get_package_share_directory(PACKAGE_NAME),
            'worlds/gazebo',
            'factory.sdf'
        ]
    )
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args':  [' -r -v 4 ', sdf_file]}.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/head_camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_camera/infra1/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_camera/infra1/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_camera/infra2/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_camera/infra2/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/trunk_camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/trunk_camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/trunk_camera/infra1/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/trunk_camera/infra1/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/trunk_camera/infra2/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/trunk_camera/infra2/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/trunk_camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/trunk_camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_front_left_fisheye/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_front_left_fisheye/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_front_right_fisheye/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_front_right_fisheye/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_left_fisheye/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_left_fisheye/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/head_right_fisheye/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/head_right_fisheye/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/l_wrist_camera1/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/l_wrist_camera1/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/l_wrist_camera2/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/l_wrist_camera2/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/r_wrist_camera1/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/r_wrist_camera1/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/r_wrist_camera2/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/r_wrist_camera2/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/model/Coke/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose'],
        output='screen'
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory(PACKAGE_NAME), 'rviz',
                             'visualize.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     parameters=[
                         {'use_sim_time': use_sim_time},
                         #  moveit_config.robot_description,
                         #  moveit_config.robot_description_semantic,
                         #  moveit_config.robot_description_kinematics,
                         #  moveit_config.planning_pipelines,
                         #  moveit_config.joint_limits,
                     ],
                     arguments=['--display-config', rviz_file, '-f', 'world'],
                     )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_dual_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'dual_arm_controller'],
        output='screen'
    )

    body_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'body_controller'],
        output='screen'
    )

    return [
        robot_state_publisher,
        spawn,
        gazebo_empty_world,

        rviz_node,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[body_controller],
            )
        ),
    ]


def prepare_launch_description():
    # Configure ROS nodes for launch
    robot_id_name = 'robot_id'
    runtime_config_package_name = 'runtime_config_package'
    controllers_file_name = 'controllers_file'
    use_sim_time_name = 'use_sim_time'
    world_name = 'world'

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_id_name,
            default_value='M92UW',
            description='Available values: GR1T2, M92UW and M92UW'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "base_parent_link",
            default_value=["base_footprint"],
            description="base_parent_link",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            use_sim_time_name,
            default_value='true',
            description='If true, use simulated clock'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            runtime_config_package_name,
            default_value=PACKAGE_NAME,
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            controllers_file_name,
            default_value='ros2_controllers.yaml',
            description='YAML file with the controllers configuration.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value=['position'],
            description="The output control command interface provided by ros2_control \
            ['position', 'velocity', 'effort', 'stiffness', 'damping'].",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "load_groups",
            default_value=['left_leg', 'right_leg', 'waist', 'head',
                           'left_arm', 'right_arm', 'left_hand', 'right_hand'],
            description="The output load_groups provided by ros2_control \
            ['left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'].",
        )
    )

    runtime_config_package = LaunchConfiguration(runtime_config_package_name)
    controllers_file = LaunchConfiguration(controllers_file_name)
    use_sim_time = LaunchConfiguration(use_sim_time_name)
    robot_id = LaunchConfiguration(robot_id_name)
    log_level = LaunchConfiguration("log_level")
    command_interface = LaunchConfiguration("command_interface")
    load_groups = LaunchConfiguration("load_groups")
    base_parent_link = LaunchConfiguration("base_parent_link")

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_nodes_spawner, args=[
            robot_id, command_interface, load_groups, base_parent_link, runtime_config_package, controllers_file, use_sim_time, log_level]
    )

    nodes = [
        robot_state_publisher_spawner_opaque_function,
    ]
    return LaunchDescription(declared_arguments + nodes)


def generate_launch_description():
    launch_description = prepare_launch_description()

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(PACKAGE_NAME), 'resource'))

    launch_description.add_action(set_env_vars_resources)

    return launch_description
