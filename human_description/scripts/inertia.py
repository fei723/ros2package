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
    links_data = {}

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
                link_info['inertia'] = {
                    'xx': ixx,
                    'xy': ixy,
                    'xz': ixz,
                    'yy': iyy,
                    'yz': iyz,
                    'zz': izz,
                }
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
        else:
            link_info['inertial'] = 'No inertial information found.'

        links_data[link_name] = link_info

    # 将数据写入 YAML 文件
    yaml_file_path = 'links_inertial.yaml'
    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(links_data, yaml_file, default_flow_style=False)

    print(f"YAML file '{yaml_file_path}' created successfully.")


if __name__ == "__main__":
    main()
