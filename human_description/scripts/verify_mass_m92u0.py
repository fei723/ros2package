#!/usr/bin/env python3

import os
import xacro
import numpy as np
from lxml import etree as ET
from ament_index_python.packages import get_package_share_directory
from prettytable import PrettyTable

measured_mass_base = 3.3753
measured_mass_left_leg = 11.1474
measured_mass_right_leg = 11.1474
measured_mass_head = 2.4767
measured_mass_waist = 24.4302
measured_mass_left_arm = 9.6663
measured_mass_right_arm = 9.6663
measured_mass_left_hand = 0.0012
measured_mass_right_hand = 0.0012

base_group = [
    "base_link"
]

left_leg_group = [
    "l_leg1_hip_y_link",
    "l_leg2_hip_r_link",
    "l_leg3_hip_p_link",
    "l_leg4_knee_p_link",
    "l_leg5_ankle_p_link",
    "l_leg6_ankle_r_link",
]

right_leg_group = [
    "r_leg1_hip_y_link",
    "r_leg2_hip_r_link",
    "r_leg3_hip_p_link",
    "r_leg4_knee_p_link",
    "r_leg5_ankle_p_link",
    "r_leg6_ankle_r_link",
]

waist_group = [
    "waist1_y_link",
    "waist2_p_link",
]

head_group = [
    "head1_y_link",
    "head2_p_link",
]

left_arm_group = [
    "l_arm1_shoulder_p_link",
    "l_arm2_shoulder_r_link",
    "l_arm3_shoulder_y_link",
    "l_arm4_elbow_p_link",
    "l_arm5_wrist_y_link",
    "l_arm6_wrist_p_link",
    "l_arm7_wrist_r_link"
]

right_arm_group = [
    "r_arm1_shoulder_p_link",
    "r_arm2_shoulder_r_link",
    "r_arm3_shoulder_y_link",
    "r_arm4_elbow_p_link",
    "r_arm5_wrist_y_link",
    "r_arm6_wrist_p_link",
    "r_arm7_wrist_r_link"
]

left_hand_group = [
    "l_hand_palm_link",
    "l_thumb_mcp_yaw_link",
    "l_thumb_mcp_pitch_link",
    "l_thumb_pip_link",
    "l_thumb_dip_link",
    "l_index_mcp_link",
    "l_index_dip_link",
    "l_middle_mcp_link",
    "l_middle_dip_link",
    "l_ring_mcp_link",
    "l_ring_dip_link",
    "l_pinky_mcp_link",
    "l_pinky_dip_link",
]

right_hand_group = [
    "r_hand_palm_link",
    "r_thumb_mcp_yaw_link",
    "r_thumb_mcp_pitch_link",
    "r_thumb_pip_link",
    "r_thumb_dip_link",
    "r_index_mcp_link",
    "r_index_dip_link",
    "r_middle_mcp_link",
    "r_middle_dip_link",
    "r_ring_mcp_link",
    "r_ring_dip_link",
    "r_pinky_mcp_link",
    "r_pinky_dip_link",
]


def calculate_error(design_value, measured_value):
    # 计算绝对误差
    absolute_error = abs(design_value - measured_value)

    # 计算相对误差（百分比形式）
    relative_error = (absolute_error / abs(design_value)) * 100

    return relative_error


def prepare_robot_description():

    robot_xacro_filepath = os.path.join(
        get_package_share_directory("human_description"),
        "robots",
        "M92U0",
        "M92U0.urdf.xacro",
    )
    robot_description_content = xacro.process_file(
        robot_xacro_filepath, mappings={'ros2_control_plugin': 'mujoco',
                                        'command_interface': "['position']", }
    ).toprettyxml(indent="  ")

    return robot_description_content


def verify_urdf():
    robot_description_content = prepare_robot_description()
    urdf_root = ET.fromstring(robot_description_content)

    total_mass = 0.0
    base_group_mass = 0.0
    left_leg_group_mass = 0.0
    right_leg_group_mass = 0.0
    waist_group_mass = 0.0
    head_group_mass = 0.0
    left_arm_group_mass = 0.0
    right_arm_group_mass = 0.0
    left_hand_group_mass = 0.0
    right_hand_group_mass = 0.0

    # 查找所有 link 元素并收集其名称和 inertial 信息
    for link in urdf_root.findall('link'):
        link_name = link.get('name')

        # 查找 inertial 元素
        inertial = link.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value')) if inertial.find(
                'mass') is not None else None
            # 累加每个连杆的质量
            total_mass += mass
            if link_name in base_group:
                base_group_mass += mass
            if link_name in left_leg_group:
                left_leg_group_mass += mass
            if link_name in right_leg_group:
                right_leg_group_mass += mass
            if link_name in waist_group:
                waist_group_mass += mass
            if link_name in head_group:
                head_group_mass += mass
            if link_name in left_arm_group:
                left_arm_group_mass += mass
            if link_name in right_arm_group:
                right_arm_group_mass += mass
            if link_name in left_hand_group:
                left_hand_group_mass += mass
            if link_name in right_hand_group:
                right_hand_group_mass += mass

            # 检查质量是否大于零
            if mass <= 0:
                print(f"Error: Link {link_name} has non-positive mass: {mass}")

    # create table for term information
    measured_mass_total = 0.0

    table = PrettyTable(float_format=".3f")
    table.title = f"Link Mass Validation Result ({urdf_root.get('name')})"
    table.field_names = [
        "Index",
        "Group",
        "Design Value (Kg)",
        "Actual Value (Kg)",
        "Error (%)",
        "Result (Pass/Fail)"
    ]
    # set alignment of table columns
    table.align["Name"] = "l"
    table.add_row([
        1,
        "base",
        round(base_group_mass, 4),
        round(measured_mass_base, 4),
        round(calculate_error(measured_mass_base, base_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_base

    table.add_row([
        2,
        "left_leg",
        round(left_leg_group_mass, 4),
        round(measured_mass_left_leg, 4),
        round(calculate_error(measured_mass_left_leg, left_leg_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_left_leg

    table.add_row([
        3,
        "left_leg",
        round(right_leg_group_mass, 4),
        round(measured_mass_right_leg, 4),
        round(calculate_error(measured_mass_right_leg, right_leg_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_right_leg

    table.add_row([
        4,
        "waist",
        round(waist_group_mass, 4),
        round(measured_mass_waist, 4),
        round(calculate_error(measured_mass_waist, waist_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_waist

    table.add_row([
        5,
        "head",
        round(head_group_mass, 4),
        round(measured_mass_head, 4),
        round(calculate_error(measured_mass_head, head_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_head

    table.add_row([
        6,
        "left arm",
        round(left_arm_group_mass, 4),
        round(measured_mass_left_arm, 4),
        round(calculate_error(measured_mass_left_arm, left_arm_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_left_arm

    table.add_row([
        7,
        "right arm",
        round(right_arm_group_mass, 4),
        round(measured_mass_right_arm, 4),
        round(calculate_error(measured_mass_right_arm, right_arm_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_right_arm

    table.add_row([
        8,
        "left hand",
        round(left_hand_group_mass, 4),
        round(measured_mass_left_hand, 4),
        round(calculate_error(measured_mass_left_hand, left_hand_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_left_hand

    table.add_row([
        9,
        "right hand",
        round(right_hand_group_mass, 4),
        round(measured_mass_right_hand, 4),
        round(calculate_error(measured_mass_right_hand, right_hand_group_mass), 4),
        "Pass",
    ])
    measured_mass_total += measured_mass_right_hand

    table.add_row([
        10,
        "Total",
        round(total_mass, 4),
        round(measured_mass_total, 4),
        round(calculate_error(measured_mass_total, total_mass), 4),
        "Pass",
    ])

    print(table.get_string())


def main(args=None):
    verify_urdf()


if __name__ == "__main__":
    main()
