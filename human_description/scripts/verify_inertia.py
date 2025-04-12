#!/usr/bin/env python3

import os
import sys
import xacro
import numpy as np
from lxml import etree as ET
from ament_index_python.packages import get_package_share_directory
from prettytable import PrettyTable
from gz.math7 import Vector3d, MassMatrix3d


def prepare_robot_description(robot_id):

    robot_xacro_filepath = os.path.join(
        get_package_share_directory("human_description"),
        "robots",
        robot_id,
        robot_id + ".urdf.xacro",
    )
    robot_description_content = xacro.process_file(
        robot_xacro_filepath, mappings={'ros2_control_plugin': 'mujoco',
                                        'command_interface': "['position']", }
    ).toprettyxml(indent="  ")

    return robot_description_content


def verify_urdf(robot_id):
    robot_description_content = prepare_robot_description(robot_id)
    urdf_root = ET.fromstring(robot_description_content)
    verify_pass = True

    # 查找所有 link 元素并收集其名称和 inertial 信息
    for link in urdf_root.findall('link'):
        link_name = link.get('name')

        # 查找 inertial 元素
        inertial = link.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value')) if inertial.find(
                'mass') is not None else None

            # 检查质量是否大于零
            if mass <= 0:
                print(f"Error: Link {link_name} has non-positive mass: {mass}")

            inertia = inertial.find('inertia')

            if inertia is not None:
                ixx = float(inertia.get('ixx'))
                ixy = float(inertia.get('ixy'))
                ixz = float(inertia.get('ixz'))
                iyy = float(inertia.get('iyy'))
                iyz = float(inertia.get('iyz'))
                izz = float(inertia.get('izz'))

                ixxyyzz = Vector3d(ixx, iyy, izz)
                ixyxzyz = Vector3d(ixy, ixz, iyz)

                mass_matrix = MassMatrix3d(float(mass), ixxyyzz, ixyxzyz)
                if not mass_matrix.is_valid():
                    verify_pass = False
                    print(f"Error: Link {link_name} has invalid inertia!")

                if ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
                    verify_pass = False
                    print(
                        f"Error: Link {link_name} inertia must satisfy A + B >= C !")

    if verify_pass:
        print(f"All links have passed the inertia parameter check!")


def main(args=None):
    robot_id = "M92U0"

    if len(sys.argv) > 1:
        # 获取第二个命令行参数并检查其是否为字符串
        arg2 = sys.argv[1]
        if isinstance(arg2, str):
            robot_id = arg2
            print("robot_id has been set to '%s'" % robot_id)
        else:
            print("The second argument is not a string!")
            sys.exit(1)

    verify_urdf(robot_id)


if __name__ == "__main__":
    main()
