#!/usr/bin/env python3

import sys
import os
from io import StringIO
from os import listdir, path
from lxml import etree as ET
from xmlformatter import Formatter
import yaml
import logging
import mujoco
import xacro
from ament_index_python.packages import get_package_share_directory

logging.getLogger().setLevel(logging.INFO)


def prepare_robot_description(robot_id):

    robot_xacro_filepath = os.path.join(
        get_package_share_directory("tendon_description"),
        "xacro",
        robot_id,
        robot_id + ".urdf.xacro",
    )
    robot_description_content = xacro.process_file(
        robot_xacro_filepath, mappings={'ros2_control_plugin': 'mujoco',
                                        'command_interface': "['position']", }
    ).toprettyxml(indent="  ")

    return robot_description_content


def main(args=None):
    robot_id = "finger"

    if len(sys.argv) > 1:
        # 获取第二个命令行参数并检查其是否为字符串
        arg2 = sys.argv[1]
        if isinstance(arg2, str):
            robot_id = arg2
            print("robot_id has been set to '%s'" % robot_id)
        else:
            print("The second argument is not a string!")
            sys.exit(1)

    # srdf_file = robot_id + ".srdf"
    output_mjcf_file = robot_id + ".mujoco.xml"
    output_original_mjcf_file = robot_id + ".mujoco.original.xml"

    # srdf_file_dir = path.join(
    #     path.dirname(path.dirname(path.dirname(
    #         path.realpath(__file__)))), "srdf", robot_id
    # )
    mjcf_file_dir = path.join(
        path.dirname(path.dirname(
            path.realpath(__file__))), "mjcf", robot_id
    )
    robot_description_content = prepare_robot_description(robot_id)

    # srdf_path = path.join(srdf_file_dir, srdf_file)
    output_path = path.join(mjcf_file_dir, output_mjcf_file)
    # output_original_mjcf_path = path.join(
    #     mjcf_file_dir, output_original_mjcf_file)

    # with open(srdf_path, "r") as f:
    #     srdf_tree = ET.parse(f)

    urdf_root = ET.fromstring(robot_description_content)
    # Create a new XML with one model
    robot_name = urdf_root.get('name')
    mujoco_tree = ET.Element('mujoco', model=robot_name)
    actuator_tree = ET.Element('actuator')
    contact_tree = ET.Element('contact')

    # srdf_root = srdf_tree.getroot()

    spec = mujoco.MjSpec.from_string(robot_description_content)
    spec.compile()
    # Add freejoint.
    # base_footprint = spec.find_body('base_footprint')
    # freejoint = base_footprint.add_freejoint(align=True)
    # freejoint.name = "base_footprint_free_joint"
    # spec.compile()

    # Add a site to the body with user data and read it back.
    base_site1_link = spec.find_body('base_site1_link')
    base_site1_link.add_site(
        name='base_site1', pos=[0, 0, 0],
    )

    mcp_site1_link = spec.find_body('mcp_site1_link')
    mcp_site1_link.add_site(
        name='mcp_site1', pos=[0, 0, 0],
    )

    pip_site1_link = spec.find_body('pip_site1_link')
    pip_site1_link.add_site(
        name='pip_site1', pos=[0, 0, 0],
    )
    pip_site2_link = spec.find_body('pip_site2_link')
    pip_site2_link.add_site(
        name='pip_site2', pos=[0, 0, 0],
    )
    pip_site3_link = spec.find_body('pip_site3_link')
    pip_site3_link.add_site(
        name='pip_site3', pos=[0, 0, 0],
    )

    dip_site1_link = spec.find_body('dip_site1_link')
    dip_site1_link.add_site(
        name='dip_site1', pos=[0, 0, 0],
    )
    dip_site2_link = spec.find_body('dip_site2_link')
    dip_site2_link.add_site(
        name='dip_site2', pos=[0, 0, 0],
    )

    tendon_site1_link = spec.find_body('tendon_site1_link')
    tendon_site1_link.add_site(
        name='tendon_site1', pos=[0, 0, 0],
    )

    # Add sensor.
    # sensor_gyro = spec.add_sensor(
    #     name="pelvis_imu_gyro",
    #     objname="pelvis_imu_site",
    #     cutoff=34.9,
    #     type=mujoco.mjtSensor.mjSENS_GYRO,
    #     objtype=mujoco.mjtObj.mjOBJ_SITE
    # )
    # sensor_framequat = spec.add_sensor(
    #     name="pelvis_imu_quaternion",
    #     objname="pelvis_imu_site",
    #     type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
    #     objtype=mujoco.mjtObj.mjOBJ_SITE
    # )
    # sensor_acc = spec.add_sensor(
    #     name="pelvis_imu_accelerometer",
    #     objname="pelvis_imu_site",
    #     type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
    #     objtype=mujoco.mjtObj.mjOBJ_SITE
    # )
    # spec.compile()

    mjcf = spec.to_xml()

    mechanical_reduction = {}
    for ros2_control in urdf_root.findall('ros2_control'):
        for transmission in ros2_control.findall('transmission'):
            joint = transmission.find('joint')
            joint_name = joint.get('name')
            gear = joint.find('mechanical_reduction').text
            mechanical_reduction[joint_name] = gear
            # print(f"joint_name: {joint_name}, gear: {gear}")

    for joint in urdf_root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        if joint_type == 'revolute' or joint_type == 'prismatic':
            # motor_name = joint_name + '_actuator'
            motor_name = joint_name.replace("_joint", "_actuator")
            motor_tree = ET.Element('motor')
            motor_tree.set('name', motor_name)
            gear = mechanical_reduction.get(joint_name, '1.0')
            motor_tree.set('gear', '20.0')
            motor_tree.set('joint', joint_name)
            limit = joint.find('limit')
            effort = float(limit.get('effort')) / float(gear)
            ctrlrange_str = f"{-effort} {effort}"
            motor_tree.set('ctrlrange', ctrlrange_str)
            actuator_tree.append(motor_tree)
            actuator = spec.add_actuator(name=motor_name, target=joint_name, trntype=mujoco.mjtTrn.mjTRN_JOINT, gear=[float(gear), 0, 0, 0, 0, 0],
                                         ctrlrange=[-effort, effort])
            # actuator.gainprm[0] = 1
            # actuator.dyntype = mujoco.mjtDyn.mjDYN_NONE
            # actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            # actuator.biastype = mujoco.mjtBias.mjBIAS_NONE

    mujoco_tree.append(actuator_tree)

    # 查找所有 link 元素并收集其名称和 inertial 信息
    urdf_links = []
    for link in urdf_root.findall('link'):
        link_name = link.get('name')
        urdf_links.append(link_name)

    # for disable_collisions in srdf_root.findall('disable_collisions'):
    #     link1 = disable_collisions.get('link1')
    #     link2 = disable_collisions.get('link2')
    #     if link1 in urdf_links and link2 in urdf_links:
    #         exclude_tree = ET.Element('exclude')
    #         exclude_tree.set('body1', link1)
    #         exclude_tree.set('body2', link2)
    #         contact_tree.append(exclude_tree)
    #         spec.add_exclude(bodyname1=link1, bodyname2=link2)
    #     else:
    #         # 如果不在 urdf_links 中，打印 link1 和 link2
    #         print(f"未找到link: link1 = {link1}, link2 = {link2}")
    # mujoco_tree.append(contact_tree)
    spec.compile()

    mjcf_string = spec.to_xml()
    # Write into output file
    xml_string = ET.tostring(
        mujoco_tree, pretty_print=True, xml_declaration=True)

    formatter = Formatter(
        indent="1",
        indent_char="\t",
        encoding_output="ISO-8859-1",
        selfclose=True,
        blanks=True,
    )

    formatted_xml = formatter.format_string(xml_string).decode("utf-8")
    formatted_mjcf = formatter.format_string(mjcf_string).decode("utf-8")

    with open(output_path, "w") as f:
        f.write(mjcf_string)
    print('Results written into "%s"' % output_path)

    # with open(output_original_mjcf_path, "w") as f:
    #     f.write(mjcf_string)
    # print('Results written into "%s"' % output_original_mjcf_path)


if __name__ == "__main__":
    main()
