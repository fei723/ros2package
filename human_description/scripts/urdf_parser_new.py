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
    inertial_data = {}
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

    urdf_path = path.join(urdf_file)

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
                ixx = inertia.get('ixx')
                ixy = inertia.get('ixy')
                ixz = inertia.get('ixz')
                iyy = inertia.get('iyy')
                iyz = inertia.get('iyz')
                izz = inertia.get('izz')

                link_info['mass'] = mass

                # 查找 inertial 中的 origin 元素
                inertial_origin = inertial.find('origin')
                if inertial_origin is not None:
                    xyz = inertial_origin.get('xyz')
                    rpy = inertial_origin.get('rpy')
                    link_info['origin'] = {
                        'xyz': xyz,
                        'rpy': rpy,
                    }
                else:
                    link_info['origin'] = 'No inertial origin information found.'

                link_info['inertia'] = {
                    'xx': ixx,
                    'xy': ixy,
                    'xz': ixz,
                    'yy': iyy,
                    'yz': iyz,
                    'zz': izz,
                }

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

        else:
            link_info['inertial'] = 'No inertial information found.'

        inertial_data[link_name] = link_info

    # 将数据写入 YAML 文件
    inertial_yaml_file_path = 'links_inertial.yaml'
    with open(inertial_yaml_file_path, 'w') as yaml_file:
        yaml.dump(inertial_data, yaml_file, sort_keys=False)

    calculation_inertial_yaml_file_path = 'calculation_inertial.yaml'
    with open(calculation_inertial_yaml_file_path, 'w') as yaml_file:
        yaml.dump(inertial_data_calculation, yaml_file, sort_keys=False)

    print(f"YAML file '{inertial_yaml_file_path}' created successfully.")

    limits_data = {}
    kinematic_data = {}
    dynamic_data = {}
    initial_positions_data = {}
    initial_positions_info = {}
    for joint in original_root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        kinematic_info = {}

        axis = joint.find('axis').get('xyz') if joint.find(
            'axis') is not None else '0 0 1'
        axis_values = [int(float(value)) for value in axis.split()]
        axis_str = ' '.join(map(str, axis_values))
        origin = joint.find('origin')
        if origin is not None:
            rpy_values = origin.get('rpy')
            rpy_list = rpy_values.split()
            xyz_values = origin.get('xyz')
            xyz_list = xyz_values.split()
            kinematic_info['kinematic'] = {
                'x': float(xyz_list[0]),
                'y': float(xyz_list[1]),
                'z': float(xyz_list[2]),
                'roll': float(rpy_list[0]),
                'pitch': float(rpy_list[1]),
                'yaw': float(rpy_list[2]),
                'axis': axis_str,
            }
            kinematic_data[joint_name] = kinematic_info

        dynamic_info = {}
        limits_info = {}

        if joint_type != "fixed":
            dynamics = joint.find('dynamics')
            if dynamics is not None:
                dynamic_info['dynamic'] = {
                    'damping': float(dynamics.get('damping')),
                    'friction': float(dynamics.get('friction')),
                }

                dynamic_data[joint_name] = dynamic_info
            limit = joint.find('limit')
            if limit is not None:
                limits_info['limit'] = {
                    'upper': round(float(limit.get('upper')) * 180 / math.pi, 1),
                    'lower': round(float(limit.get('lower')) * 180 / math.pi, 1),
                    'velocity': float(limit.get('velocity')),
                    'effort': float(limit.get('effort')),

                    'k_position': 121.0,
                    'k_velocity': 540.0,
                    'safety_margin': 3.0,
                }
                limits_data[joint_name] = limits_info
            initial_positions_info[joint_name.replace("_joint", "")] = 0.0
    initial_positions_data["initial_positions"] = initial_positions_info

    # 将数据写入 YAML 文件
    kinematic_yaml_file_path = 'kinematic.yaml'
    with open(kinematic_yaml_file_path, 'w') as yaml_file:
        yaml.dump(kinematic_data, yaml_file, sort_keys=False)

    print(f"YAML file '{kinematic_yaml_file_path}' created successfully.")

    dynamic_yaml_file_path = 'dynamic.yaml'
    with open(dynamic_yaml_file_path, 'w') as yaml_file:
        yaml.dump(dynamic_data, yaml_file, sort_keys=False)

    print(f"YAML file '{dynamic_yaml_file_path}' created successfully.")

    limit_yaml_file_path = 'joint_limits.yaml'
    with open(limit_yaml_file_path, 'w') as yaml_file:
        yaml.dump(limits_data, yaml_file, sort_keys=False)

    print(f"YAML file '{limit_yaml_file_path}' created successfully.")

    initial_positions_yaml_file_path = 'initial_positions.yaml'
    with open(initial_positions_yaml_file_path, 'w') as yaml_file:
        yaml.dump(initial_positions_data, yaml_file, sort_keys=False)

    print(
        f"YAML file '{initial_positions_yaml_file_path}' created successfully.")


if __name__ == "__main__":
    main()
