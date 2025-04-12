#!/usr/bin/env python3

import sys
from io import StringIO
from os import listdir, path
from lxml import etree as ET
import logging
from gz.math7 import Vector3d, MassMatrix3d


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
        "Verifying inertial properties for each link according to the default configuration '%s'" % urdf_file)

    urdf_file_dir = path.join(
        path.dirname(path.dirname(path.realpath(__file__))), "urdf"
    )
    logging.info("urdf_file_dir: %s" % urdf_file_dir)

    urdf_path = path.join(urdf_file_dir, urdf_file)

    with open(urdf_path, "r") as f:
        original_tree = ET.parse(f)

    original_root = original_tree.getroot()
    verify_pass = True
    # 查找所有 link 元素并收集其名称和 inertial 信息
    for link in original_root.findall('link'):
        link_name = link.get('name')

        # 查找 inertial 元素
        inertial = link.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value')) if inertial.find(
                'mass') is not None else None
            
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

    


if __name__ == "__main__":
    main()
