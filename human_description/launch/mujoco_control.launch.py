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
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
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

PACKAGE_NAME = 'human_description'


def robot_nodes_spawner(
        context: LaunchContext,
        robot_id, robot_description_param, use_rviz, command_interface, load_groups,
        controllers_to_activate, runtime_config_package, controllers_file, use_sim_time, log_level, mujoco_scene_file
):
    robot_id_str = context.perform_substitution(robot_id)

    robot_description_param_str = context.perform_substitution(robot_description_param)
    if robot_description_param_str:
        robot_description = {
            "robot_description": robot_description_param_str
        }
    else:
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
                "mujoco",
                " ",
                "command_interface:=",
                command_interface,
                " ",
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
    mujoco_scene_file_str = context.perform_substitution(mujoco_scene_file)
    if mujoco_scene_file_str:
        mujoco_model_file = mujoco_scene_file_str
        print(f"Using mujoco_model_file: \"{mujoco_model_file}\"")
    else:
        mujoco_model_file = PathJoinSubstitution(
            [
                FindPackageShare(runtime_config_package),
                "mjcf",
                robot_id_str,
                robot_id_str + '.mujoco.scene.xml',
            ]
        )
        print(f"Using mujoco_model_file: \"{context.perform_substitution(mujoco_model_file)}\"")

    controllers_to_activate_str = context.perform_substitution(controllers_to_activate)
    controllers_names_to_activate = []
    if controllers_to_activate_str:
        print(f"Using controllers_to_activate_str: \"{eval(controllers_to_activate_str)}\"")
        for controller_name in eval(controllers_to_activate_str):
            print(f"to activate controller name: {controller_name}")
            controllers_names_to_activate.append(controller_name)
    else:
        controllers_names_to_activate = [
            # 'pelvis_imu_broadcaster',
            'body_controller',
        ]
        print(f"Using default controllers_to_activate: \"{controllers_names_to_activate}\"")

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
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
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

    diagnostic_aggregator_config = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            robot_id_str,
            "diagnostic_analyzer.yaml",
        ]
    )
    diagnostic_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        # namespace="",
        name="aggregator_node",
        output="log",
        arguments=[
            "--ros-args",
            "--log-level",
            "error",
        ],
        parameters=[diagnostic_aggregator_config],
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    wait_for_clock_process = ExecuteProcess(
        cmd="ros2 topic echo /clock rosgraph_msgs/msg/Clock --once".split(" "),
        output="screen",
    )

    timer_action = TimerAction(
        period=10.0,
        actions = [
            wait_for_clock_process,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wait_for_clock_process,
                    on_exit=[load_joint_state_broadcaster]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[controller_name_to_activate, "-c", "/controller_manager"],
                        ) for controller_name_to_activate in controllers_names_to_activate
                    ],
                )
            )
        ]
    )

    return [
        robot_state_publisher,
        rviz,
        mujoco_node,
        # diagnostic_node,
        timer_action
    ]

def prepare_launch_description():
   # Configure ROS nodes for launch
    robot_id_name = 'robot_id'
    runtime_config_package_name = 'runtime_config_package'
    controllers_file_name = 'controllers_file'
    use_sim_time_name = 'use_sim_time'
    world_name = 'world'

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        DeclareLaunchArgument(
            robot_id_name,
            default_value='GR1T2',
            description='Available values: GR1T2, M92UW and M92UW'
        ),
        DeclareLaunchArgument(
            use_sim_time_name,
            default_value='true',
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            runtime_config_package_name,
            default_value=PACKAGE_NAME,
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.'
        ),
        DeclareLaunchArgument(
            controllers_file_name,
            default_value='ros2_controllers.yaml',
            description='YAML file with the controllers configuration.'
        ),
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value="mujoco_ros2_control_params.yaml",
            description="YAML file with the mujoco ros2 control params configuration.",
        ),
        DeclareLaunchArgument(
            "command_interface",
            default_value=['position', 'velocity',
                           'effort', 'stiffness', 'damping'],
            description="The output control command interface provided by ros2_control \
            ['position', 'velocity', 'effort', 'stiffness', 'damping'].",
        ),
        DeclareLaunchArgument(
            "load_groups",
            default_value=['left_leg', 'right_leg', 'waist', 'head',
                           'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'],
            description="The output load_groups provided by ros2_control \
            ['left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'].",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value='true',
            description="Weather launch rviz.",
        ),
        DeclareLaunchArgument(
            "controllers_to_activate",
            default_value='',
            description="The output controllers_to_activate provided by ros2_control \
            ['left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'].",
        ),
        DeclareLaunchArgument(
            "robot_description",
            default_value='',
            description="The robot_description xml with ros2 control label",
        ),
        DeclareLaunchArgument(
            "mujoco_scene_file",
            default_value="",
            description="The scene file for mujoco loading.",
        ),
    ]

    runtime_config_package = LaunchConfiguration(runtime_config_package_name)
    controllers_file = LaunchConfiguration(controllers_file_name)
    use_sim_time = LaunchConfiguration(use_sim_time_name)
    robot_id = LaunchConfiguration(robot_id_name)
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    log_level = LaunchConfiguration("log_level")
    command_interface = LaunchConfiguration("command_interface")
    load_groups = LaunchConfiguration("load_groups")
    use_rviz = LaunchConfiguration("use_rviz")
    controllers_to_activate = LaunchConfiguration("controllers_to_activate")
    robot_description = LaunchConfiguration("robot_description")
    mujoco_scene_file = LaunchConfiguration("mujoco_scene_file")

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_nodes_spawner, args=[
            robot_id, robot_description, use_rviz, command_interface, load_groups, controllers_to_activate,
            runtime_config_package, controllers_file, use_sim_time, log_level, mujoco_scene_file
        ]
    )

    return LaunchDescription(declared_arguments + [robot_state_publisher_spawner_opaque_function])


def generate_launch_description():
    launch_description = prepare_launch_description()

    return launch_description
