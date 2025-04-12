#!/usr/bin/env python3

import os
import xacro
import numpy as np
from lxml import etree as ET
from ament_index_python.packages import get_package_share_directory
from prettytable import PrettyTable

measured_mass_total = 138.0
measured_mass_chassis = 80.0
measured_mass_trunk = 35.0
measured_mass_head = 3.0
measured_mass_left_arm = 10.0
measured_mass_right_arm = 10.0

chassis_group = [
    "base_link",
    "lf_wheel_link",
    "lb_wheel_link",
    "rf_wheel_link",
    "rb_wheel_link"
]

trunk_group = [
    "ankle_p_link",
    "knee_p_link",
    "waist1_p_link",
    "waist2_y_link",
    "trunk_link",
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
        "M92UW",
        "M92UW.urdf.xacro",
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
    chassis_group_mass = 0.0
    trunk_group_mass = 0.0
    head_group_mass = 0.0
    left_arm_group_mass = 0.0
    right_arm_group_mass = 0.0

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
            if link_name in chassis_group:
                chassis_group_mass += mass
            if link_name in trunk_group:
                trunk_group_mass += mass
            if link_name in head_group:
                head_group_mass += mass
            if link_name in left_arm_group:
                left_arm_group_mass += mass
            if link_name in right_arm_group:
                right_arm_group_mass += mass

            # 检查质量是否大于零
            if mass <= 0:
                print(f"Error: Link {link_name} has non-positive mass: {mass}")

    # create table for term information
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
        "chassis",
        round(chassis_group_mass, 4),
        round(measured_mass_chassis, 4),
        round(calculate_error(measured_mass_chassis, chassis_group_mass), 4),
        "Pass",
    ])

    table.add_row([
        2,
        "trunk",
        round(trunk_group_mass, 4),
        round(measured_mass_trunk, 4),
        round(calculate_error(measured_mass_trunk, trunk_group_mass), 4),
        "Pass",
    ])

    table.add_row([
        3,
        "head",
        round(head_group_mass, 4),
        round(measured_mass_head, 4),
        round(calculate_error(measured_mass_head, head_group_mass), 4),
        "Pass",
    ])

    table.add_row([
        4,
        "left arm",
        round(left_arm_group_mass, 4),
        round(measured_mass_left_arm, 4),
        round(calculate_error(measured_mass_left_arm, left_arm_group_mass), 4),
        "Pass",
    ])

    table.add_row([
        5,
        "right arm",
        round(right_arm_group_mass, 4),
        round(measured_mass_right_arm, 4),
        round(calculate_error(measured_mass_right_arm, right_arm_group_mass), 4),
        "Pass",
    ])

    table.add_row([
        6,
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
