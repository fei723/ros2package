#!/usr/bin/env python3

import sys
from io import StringIO
from os import listdir, path
from lxml import etree as ET
from xmlformatter import Formatter
import yaml
import logging


logging.getLogger().setLevel(logging.INFO)


def main():
    urdf_file = "GR1T2.urdf"

    if len(sys.argv) > 1:
        # 获取第二个命令行参数并检查其是否为字符串
        arg2 = sys.argv[1]
        if isinstance(arg2, str):
            urdf_file = arg2
            print("config file has been set to '%s'" % urdf_file)
        else:
            print("The second argument is not a string!")
            sys.exit(1)

    print(
        "Checking limits properties for each link according to the default configuration '%s'" % urdf_file)

    urdf_file_dir = path.join(
        path.dirname(path.dirname(path.realpath(__file__))), "urdf"
    )
    logging.info("urdf_file_dir: %s" % urdf_file_dir)

    urdf_path = path.join(urdf_file_dir, urdf_file)

    with open(urdf_path, "r") as f:
        original_tree = ET.parse(f)

    original_root = original_tree.getroot()
    verify_pass = True
    # 查找所有 joint 元素并收集其名称和 limits 信息
    for joint in original_root.findall('joint'):
        joint_name = joint.get('name')

        # 查找 limits 元素
        limits = joint.find('limits')
        if limits is not None:
            lower = float(limits.get('lower'))
            upper = float(limits.get('upper'))

            if lower >= upper:
                verify_pass = False
                print(f"Error: Joint {joint_name} has invalid limits: lower limit ({lower}) should be less than upper limit ({upper})!")
            
            if not (lower < 0 < upper):
                verify_pass = False
                print(f"Error: Joint {joint_name} has invalid limits: lower limit ({lower}) should be less than 0 and upper limit ({upper}) should be greater than 0!")

            safety_controller = joint.find('safety_controller')

            soft_lower_limit = float(safety_controller.get('soft_lower_limit'))
            soft_upper_limit = float(safety_controller.get('soft_upper_limit'))

            if soft_lower_limit >= soft_upper_limit:
                verify_pass = False
                print(f"Error: Joint {joint_name} has invalid soft limits: soft lower limit ({soft_lower_limit}) should be less than soft upper limit ({soft_upper_limit})!")

            if not (lower < soft_lower_limit < soft_upper_limit < upper):
                verify_pass = False
                print(f"Error: Joint {joint_name} has invalid soft limits: The soft limits ({soft_lower_limit}, {soft_upper_limit}) should be within the range ({lower}, {upper})!")
    
    if verify_pass:
        print(f"All joints have passed the limits parameter check!")



if __name__ == "__main__":
    main()
