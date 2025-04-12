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

PACKAGE_NAME = 'human_description'


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
            default_value='GR1T2',
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
            world_name,
            default_value='fr3_world.wbt',
            description='Choose one of the world files from `./worlds/webots` directory.'
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
    world = LaunchConfiguration(world_name)
    command_interface = LaunchConfiguration(
        "command_interface")
    load_groups = LaunchConfiguration("load_groups")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(runtime_config_package),
                 "robots", robot_id, "GR1T2.urdf.xacro"]
            ),
            " ",
            "ros2_control_plugin:=",
            "webots",
            " "
            "command_interface:=",
            command_interface,
            " ",
            "load_groups:=",
            load_groups,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Get robot description
    package_dir = get_package_share_directory(PACKAGE_NAME)

    robot_xacro_filepath = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "robots",
        "GR1T2",
        "GR1T2" + ".urdf.xacro",
    )

    robot_description_config = xacro.process_file(
        robot_xacro_filepath,
        mappings={
            'ros2_control_plugin': 'webots',
            'command_interface': "['position']",
            'load_groups': "['left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand']",
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

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
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            "GR1T2",
            controllers_file,
        ]
    )
    ros2_control_params = os.path.join(
        package_dir, 'config', 'ros2_controllers.yaml')
    # Starts Webots
    # Define your URDF robots here
    # The name of an URDF robot has to match the WEBOTS_CONTROLLER_URL of the driver node
    # You can specify the URDF file to use with "urdf_path"
    spawn_URDF_robot = URDFSpawner(
        name='GR1T2',
        robot_description=robot_description_config.toxml(),
        translation='0 0 0.62',
        rotation='0 0 1 -1.5708',
    )
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds/webots', world]),
        ros2_supervisor=True
    )
    robot_driver = WebotsController(
        robot_name='GR1T2',
        namespace='',
        # prefix=['gnome-terminal --wait -- gdb -ex run --args'],
        parameters=[
            {'robot_description': robot_xacro_filepath},
            {'xacro_mappings': ['ros2_control:=true',
                                'command_interface:=[position]',
                                # 'load_groups:=[left_leg, right_leg, waist, head, left_arm, right_arm, left_hand, right_hand]',
                                'ros2_control_plugin:=webots']},
            {'use_sim_time': True},
            {'set_robot_state_publisher': False},
            ros2_control_params
        ],
        respawn=True
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory(PACKAGE_NAME), 'rviz',
                             'visualize.rviz')
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': use_sim_time}],
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

    nodes = [
        robot_state_publisher,
        spawn_URDF_robot,
        webots,
        webots._supervisor,
        rviz,
        robot_driver,
        load_joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_dual_arm_controller],
            )
        ),
    ]
    return LaunchDescription(declared_arguments + nodes)


def generate_launch_description():
    launch_description = prepare_launch_description()

    return launch_description
