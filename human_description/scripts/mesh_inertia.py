#!/usr/bin/env python3

import sys
from io import StringIO
from os import listdir, path
from lxml import etree as ET
from xmlformatter import Formatter
import yaml
import logging
import math
import trimesh
import re
from ament_index_python.packages import get_package_share_directory

logging.getLogger().setLevel(logging.INFO)

# 从 mesh 路径中提取包名


def extract_package_name(mesh_file):
    match = re.match(r'package://([^/]+)/', mesh_file)
    if match:
        return match.group(1)
    return None


def main():
    urdf_file = "GR1T2.urdf"
    inertial_data_calculation = {}

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
        "Calculationing inertial properties for each link according to the default configuration '%s'" % urdf_file)

    urdf_file_dir = path.join(
        path.dirname(path.dirname(path.realpath(__file__))), "urdf"
    )
    logging.info("urdf_file_dir: %s" % urdf_file_dir)

    urdf_path = path.join(urdf_file_dir, urdf_file)

    with open(urdf_path, "r") as f:
        original_tree = ET.parse(f)

    original_root = original_tree.getroot()
    # 查找所有 link 元素并收集其名称和 inertial 信息
    for link in original_root.findall('link'):
        link_name = link.get('name')
        link_info = {}
        calculation_link_info = {}

        # 查找 inertial 元素
        inertial = link.find('inertial')
        if inertial is not None:
            mass = inertial.find('mass').get(
                'value') if inertial.find('mass') is not None else None
            inertia = inertial.find('inertia')

            if inertia is not None:
                for visual in link.findall('.//visual'):
                    for geometry in visual.findall('.//geometry'):
                        mesh = geometry.find('mesh')
                        if mesh is not None:
                            filename = mesh.get('filename')
                            if filename:
                                mesh_file = filename

                                if mesh_file.startswith('package://'):
                                    # package_path = get_package_share_directory()
                                    package_name = extract_package_name(
                                        mesh_file)
                                    package_share_dir = get_package_share_directory(
                                        package_name)
                                    mesh_file_path = mesh_file.replace(
                                        f"package://{package_name}/", path.join(package_share_dir, ""))
                                    mesh = trimesh.load(
                                        mesh_file_path, force="mesh", ignore_materials=True)
                                    mesh.apply_scale([1e-3, 1e-3, 1e-3])
                                    mesh.density = float(mass) / mesh.volume
                                    inertia_new = mesh.moment_inertia
                                    centre_of_mass = mesh.center_mass
                                    calculation_link_info['mass'] = mass
                                    calculation_link_info['origin'] = {
                                        'xyz': ' '.join('{:.8F}'.format(v)
                                                        for v in centre_of_mass),
                                        'rpy': '0 0 0',
                                    }
                                    calculation_link_info['inertia'] = {
                                        'xx': ''.join('{:.8E}'.format(inertia_new[0][0])),
                                        'xy': ''.join('{:.8E}'.format(inertia_new[0][1])),
                                        'xz': ''.join('{:.8E}'.format(inertia_new[0][2])),
                                        'yy': ''.join('{:.8E}'.format(inertia_new[1][1])),
                                        'yz': ''.join('{:.8E}'.format(inertia_new[1][2])),
                                        'zz': ''.join('{:.8E}'.format(inertia_new[2][2])),
                                    }
                                    inertial_data_calculation[link_name] = calculation_link_info

                                    # 对比惯性矩阵
                                    for key, value in calculation_link_info['inertia'].items():
                                        original_value = inertia.get(key, None)
                                        if original_value and abs(float(original_value) - float(value)) > 1e-3:
                                            print(f"Link '{link_name}': Inertia {key} mismatch. Original: {original_value}, Calculated: {value}")

        else:
            link_info['inertial'] = 'No inertial information found.'

    # 将数据写入 YAML 文件
    calculation_inertial_yaml_file_path = 'calculation_inertial.yaml'
    with open(calculation_inertial_yaml_file_path, 'w') as yaml_file:
        yaml.dump(inertial_data_calculation, yaml_file, sort_keys=False)

    print(f"YAML file '{calculation_inertial_yaml_file_path}' created successfully.")


if __name__ == "__main__":
    main()
