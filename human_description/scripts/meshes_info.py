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
    meshes_info_collision = {}
    meshes_info_visual = {}
    visual_file_size_bytes = 0
    collision_file_size_bytes = 0
    visual_faces = 0
    collision_faces = 0

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
        mesh_info = {}

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
                            mesh_load = trimesh.load(
                                mesh_file_path, force="mesh", ignore_materials=True)
                            mesh_load.apply_scale([1e-3, 1e-3, 1e-3])
                            file_size_bytes = path.getsize(
                                mesh_file_path)
                            visual_file_size_bytes += file_size_bytes
                            file_size_mb = file_size_bytes / (1024 * 1024)
                            mesh_info['size'] = f"{file_size_mb:.2f} MB"
                            mesh_info['faces'] = len(mesh_load.faces)
                            visual_faces += len(mesh_load.faces)

                            meshes_info_visual[link_name] = mesh_info

                            bounding_box = mesh_load.bounds
                            dimensions = bounding_box[1] - bounding_box[0]
                            center = (bounding_box[0] + bounding_box[1]) / 2
                            print(f"{link_name} dimensions: {dimensions}, center: {center}")
    total_meshes_info_visual = {}
    visual_file_size_mb = visual_file_size_bytes / (1024 * 1024)
    total_meshes_info_visual['total_file_size'] = f"{visual_file_size_mb:.2f} MB"
    total_meshes_info_visual['total_faces'] = visual_faces
    meshes_info_visual['total'] = total_meshes_info_visual
    meshes_info_file_path = 'meshes_info_visual.yaml'
    with open(meshes_info_file_path, 'w') as yaml_file:
        yaml.dump(meshes_info_visual, yaml_file, sort_keys=False)

    for link in original_root.findall('link'):
        link_name = link.get('name')
        mesh_info = {}
        for collision in link.findall('.//collision'):
            for geometry in collision.findall('.//geometry'):
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
                            mesh_load = trimesh.load(
                                mesh_file_path, force="mesh", ignore_materials=True)

                            file_size_bytes = path.getsize(
                                mesh_file_path)
                            collision_file_size_bytes += file_size_bytes
                            file_size_mb = file_size_bytes / (1024 * 1024)
                            mesh_info['size'] = f"{file_size_mb:.2f} MB"
                            mesh_info['faces'] = len(mesh_load.faces)
                            collision_faces += len(mesh_load.faces)

                            meshes_info_collision[link_name] = mesh_info
    total_meshes_info_collision = {}
    collision_file_size_mb = collision_file_size_bytes / (1024 * 1024)
    total_meshes_info_collision['total_file_size'] = f"{collision_file_size_mb:.2f} MB"
    total_meshes_info_collision['total_faces'] = collision_faces
    meshes_info_collision['total'] = total_meshes_info_collision

    meshes_info_file_path = 'meshes_info_collision.yaml'
    with open(meshes_info_file_path, 'w') as yaml_file:
        yaml.dump(meshes_info_collision, yaml_file, sort_keys=False)


if __name__ == "__main__":
    main()
