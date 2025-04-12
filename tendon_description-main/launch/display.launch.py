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

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
PACKAGE_NAME = 'tendon_description'

def robot_state_publisher_spawner(context: LaunchContext, robot_id):
    robot_id_str = context.perform_substitution(robot_id)

    robot_xacro_filepath = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "xacro",
        robot_id_str,
        robot_id_str + ".urdf.xacro",
    )
    robot_description = xacro.process_file(
        robot_xacro_filepath, mappings={}
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():

    robot_id_parameter_name = "robot_id"
    robot_id = LaunchConfiguration(robot_id_parameter_name)

    rviz_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "rviz",
        "visualize.rviz",
    )

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_state_publisher_spawner, args=[robot_id]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_id_parameter_name,
                default_value="finger",
                description="ID of the type of arm used. Supporter values: "
                "M92C, M92UW,M92U0",
            ),
            robot_state_publisher_spawner_opaque_function,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            ),
        ]
    )
