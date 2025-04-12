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
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'tendon_description'


def robot_nodes_spawner(context: LaunchContext, robot_id, command_interface, runtime_config_package, controllers_file, use_sim_time, log_level, transmission_type):
    robot_id_str = context.perform_substitution(robot_id)
    transmission_type_str = context.perform_substitution(transmission_type)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(PACKAGE_NAME),
                 "xacro", robot_id_str, robot_id_str + ".urdf.xacro"]
            ),
            " ",
            "ros2_control_plugin:=",
            "mujoco",
            " ",
            "command_interface:=",
            command_interface,
            " ",
            "transmission_type:=",
            transmission_type,
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

    mujoco_model_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "mjcf",
            robot_id_str,
            robot_id_str + '.mujoco.scene.xml',
        ]
    )

    ros2_control_config = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            robot_id_str,
            "mujoco_ros2_control_params.yaml",
        ]
    )

    # Visualize in RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "rviz", "mujoco.rviz"]
    )
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['--display-config', rviz_config_file],
                )

    mujoco_node = Node(
        package="mujoco_simulate",
        executable="mujoco_simulate_node",
        # namespace="",
        parameters=[robot_description, robot_controllers, ros2_control_config],
        output="both",
        # prefix=['xterm -e gdb -ex run -args'],
        # prefix=['gnome-terminal --wait -- gdb -ex run --args'],
        arguments=[
            mujoco_model_file,
            "--ros-args",
            "--log-level",
            log_level,
        ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    finger_controller = transmission_type_str + '_controller'
    
    load_body_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             finger_controller],
        output='screen'
    )

    rqt_traj_node = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        # namespace="",
        name="rqt_joint_trajectory_controller",
        output="log",
    )

    return [
        robot_state_publisher,
        rviz,
        mujoco_node,
        load_joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_body_controller],
            )
        ),
        rqt_traj_node,
    ]


def prepare_launch_description():
    # Configure ROS nodes for launch
    robot_id_name = 'robot_id'
    runtime_config_package_name = 'runtime_config_package'
    controllers_file_name = 'controllers_file'
    use_sim_time_name = 'use_sim_time'
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_id_name,
            default_value='finger',
            description='Available values: GR1T2, M92UW and M92UW'
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
            "ros2_control_config_file",
            default_value="mujoco_ros2_control_params.yaml",
            description="YAML file with the mujoco ros2 control params configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value=['position', 'velocity',
                           'effort', 'stiffness', 'damping'],
            description="The output control command interface provided by ros2_control \
            ['position', 'velocity', 'effort', 'stiffness', 'damping'].",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "transmission_type",
            default_value="simple",
            description="simple or tendon",
        )
    )

    runtime_config_package = LaunchConfiguration(runtime_config_package_name)
    controllers_file = LaunchConfiguration(controllers_file_name)
    use_sim_time = LaunchConfiguration(use_sim_time_name)
    robot_id = LaunchConfiguration(robot_id_name)
    transmission_type = LaunchConfiguration("transmission_type")

    log_level = LaunchConfiguration("log_level")
    command_interface = LaunchConfiguration("command_interface")

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_nodes_spawner, args=[
            robot_id, command_interface, runtime_config_package, controllers_file, use_sim_time, log_level, transmission_type]
    )

    nodes = [
        robot_state_publisher_spawner_opaque_function,
    ]
    return LaunchDescription(declared_arguments + nodes)


def generate_launch_description():
    launch_description = prepare_launch_description()

    return launch_description
