#  Copyright (c) 2024 Franka Robotics GmbH
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

import argparse
import os

import xacro
from distutils.util import strtobool


def str_to_bool(s):
    return s.lower() in ["true", "1", "yes", "y"]


def convert_xacro_to_urdf(xacro_file, robot_id, hand_id, base_parent_link, finger_joint_fixed, head_joint_fixed, ros2_control, ros2_control_plugin, command_interface, load_groups):
    """Convert xacro file into a urdf file."""
    urdf = xacro.process_file(
        xacro_file, mappings={
            "robot_id": str(robot_id), 
            "hand_id": str(hand_id), 
            "base_parent_link": str(base_parent_link), 
            "finger_joint_fixed": str(finger_joint_fixed),
            "head_joint_fixed": str(head_joint_fixed),
            "ros2_control": str(ros2_control),
            "ros2_control_plugin": str(ros2_control_plugin),
            "command_interface": command_interface,
            "load_groups": load_groups
        }
    )
    urdf_file = urdf.toprettyxml(indent="  ")
    return urdf_file


def convert_package_name_to_absolute_path(package_name, package_path, urdf_file):
    """Replace a ROS package names with the absolute paths."""
    urdf_file = urdf_file.replace("package://{}".format(package_name), package_path)
    return urdf_file


def urdf_generation(package_path, xacro_file, file_name, ROBOT_ID, HAND_ID, BASE_PARENT_LINK, FINGER_JOINT_FIXED, HEAD_JOINT_FIXED, ROS2_CONTROL, ROS2_CONTROL_PLUGIN, COMMAND_INTERFACE, LOAD_GROUPS, ABSOLUTE_PATHS, HOST_DIR):
    """Generate URDF file and save it."""
    xacro_file = os.path.join(package_path, xacro_file)
    urdf_file = convert_xacro_to_urdf(xacro_file,  ROBOT_ID, HAND_ID, BASE_PARENT_LINK, FINGER_JOINT_FIXED, HEAD_JOINT_FIXED, ROS2_CONTROL, ROS2_CONTROL_PLUGIN, COMMAND_INTERFACE, LOAD_GROUPS)
    if ABSOLUTE_PATHS and HOST_DIR is None:
        urdf_file = convert_package_name_to_absolute_path(
            package_name, package_path, urdf_file
        )
    elif ABSOLUTE_PATHS:
        urdf_file = convert_package_name_to_absolute_path(
            package_name, HOST_DIR, urdf_file
        )
    save_urdf_to_file(package_path, urdf_file, file_name)


def save_urdf_to_file(package_path, urdf_file, robot):
    """Save URDF into a file."""
    # Check if the folder exists, and if not, create it
    folder_path = f"{package_path}/urdf"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    with open(f"{package_path}/urdf/{robot}.urdf", "w") as f:
        f.write(urdf_file)
    
    print(f"URDF file for {robot} created and save as {package_path}/urdf/{robot}.urdf successfully!\n")


if __name__ == "__main__":
    package_name = "human_description"

    if os.getcwd().split("/")[-1] != package_name:
        print("Call the script from human_description root folder")
        exit()

    ROBOTS = ["GR1T2", "M92C", "M92U0", "M92UW"]

    HANDS = ["fourier_hand", "inspire_hand", "m92c_hand", "m92u_hand"]

    DEFAULT_COMMAND_INTERFACE = ["position", "velocity"]

    DEFAULT_LOAD_GROUPS = ['left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors']

    parser = argparse.ArgumentParser(
        description="""
            Generate urdf model for a given robot.
            """
    )

    robots_str = ", ".join([f"{item}" for item in ROBOTS])
    parser.add_argument(
        "robot_id",
        type=str,
        help="id of the robot model (accepted values are: {})".format(robots_str),
    )
    hand_str = ", ".join([f"{item}" for item in HANDS])
    parser.add_argument(
        "hand_id",
        type=str,
        help="id of the robot end effector (accepted values are: {})".format(hand_str),
    )
    parser.add_argument(
        "--abs-path", help="Use absolute paths.", action="store_const", const=True
    )
    parser.add_argument(
        "--host-dir", help="Provide a host directory for the absolute path."
    )
    parser.add_argument(
        "--ros2-control",
        help="Is the robot being controlled with ros2_control? (default: True, use 'true' or 'false')",
        type=lambda x: bool(strtobool(x)),  
        default=True,
    )
    parser.add_argument(
        "--ros2-control-plugin",
        type=str,
        default="gazebo", 
        help="The ros2_control plugin to be loaded for the manipulator (default: 'gazebo'). "
            "Options: 'fake', 'gazebo', 'webots', 'mujoco', 'isaac', 'real', or a custom value."
    )
    parser.add_argument(
        "--command-interface",
        type=str,
        default=",".join(DEFAULT_COMMAND_INTERFACE),  
        help="The output control command interface provided by ros2_control (default: ['position', 'velocity'])."
            "Options: 'position', 'velocity', 'effort', or combinations like 'stiffness, damping'. "  
            "Example: '--command-interface 'effort'' to set the control command interface to 'effort'."
    )
    parser.add_argument(
        "--load-groups",
        type=str,
        default=",".join(DEFAULT_LOAD_GROUPS),  
        help="Comma-separated list of robot components to load (default: all components). "
            "Options: 'left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm', 'left_hand', 'right_hand', 'sensors'. "
            "Example: '--load-groups 'left_leg,right_leg'' to load left and right legs."
    )
    parser.add_argument(
        "--finger-joint-fixed",
        help="Set to fix the finger joints (default finger joints are revolute).",
        action="store_true",  
    )
    parser.add_argument(
        "--head-joint-fixed",
        help="Set to fix the head joints (default head joints are revolute).",
        action="store_true",  
    )
    parser.add_argument(
        "--base-parent-link",
        type=str,
        help="Specify the link to which the robot base should be connected. "
            "If not provided, the robot remains unconnected. "
            "Example: '--base-parent-link world' to fix the robot to a 'world' link."
    )

    args = parser.parse_args()

    ROBOT_ID = args.robot_id if args.robot_id is not None else "GR1T2"
    HAND_ID = args.hand_id if args.hand_id is not None else "fourier_hand"
    ABSOLUTE_PATHS = args.abs_path if args.abs_path is not None else False
    HOST_DIR = args.host_dir
    ROS2_CONTROL = args.ros2_control  
    ROS2_CONTROL_PLUGIN = args.ros2_control_plugin
    COMMAND_INTERFACE = args.command_interface
    LOAD_GROUPS = args.load_groups
    FINGER_JOINT_FIXED = args.finger_joint_fixed
    HEAD_JOINT_FIXED = args.head_joint_fixed
    BASE_PARENT_LINK = args.base_parent_link if args.base_parent_link is not None else ""

    assert ROBOT_ID in ROBOTS 

    package_path = os.getcwd()

    print(f"\n*** Creating URDF for {ROBOT_ID} ***")
    xacro_file = f"robots/{ROBOT_ID}/{ROBOT_ID}.urdf.xacro"
    file_name = f"{ROBOT_ID}"
    urdf_generation(package_path, xacro_file, file_name, ROBOT_ID, HAND_ID, BASE_PARENT_LINK, FINGER_JOINT_FIXED, HEAD_JOINT_FIXED, ROS2_CONTROL, ROS2_CONTROL_PLUGIN, COMMAND_INTERFACE, LOAD_GROUPS, ABSOLUTE_PATHS, HOST_DIR)
