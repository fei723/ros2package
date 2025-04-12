#!/usr/bin/env python3

import sys
import os
from io import StringIO
from os import listdir, path
from lxml import etree as ET
from xmlformatter import Formatter
import yaml
import math
import logging
import mujoco
import xacro
from ament_index_python.packages import get_package_share_directory

logging.getLogger().setLevel(logging.INFO)

# sensor_name: camera topic base name
camera_topic_dict = {
    "head_front_left_fisheye_color": "/camera/head_front/left_rgb",
    "head_front_right_fisheye_color": "/camera/head_front/right_rgb",
    "head_left_fisheye_color": "head_left_fisheye_color",
    "head_right_fisheye_color": "head_right_fisheye_color",
    "head_camera_color": "head_camera_color",
    "head_camera_depth": "head_camera_depth",
    "l_wrist_camera1_color": "l_wrist_camera1_color",
    "l_wrist_camera2_color": "l_wrist_camera2_color",
    "r_wrist_camera1_color": "r_wrist_camera1_color",
    "r_wrist_camera2_color": "r_wrist_camera2_color",
}


def prepare_robot_description(robot_id):

    robot_xacro_filepath = os.path.join(
        get_package_share_directory("human_description"),
        "robots",
        robot_id,
        robot_id + ".urdf.xacro",
    )
    robot_description_content = xacro.process_file(
        robot_xacro_filepath,
        mappings={
            "ros2_control_plugin": "mujoco",
            "command_interface": "['position']",
            "base_parent_link": "base_footprint",
        },
    ).toprettyxml(indent="  ")

    return robot_description_content


def calculate_vfov(width, height, hfov_rad):
    f = width / (2 * math.tan(hfov_rad / 2))
    vfov_rad = 2 * math.atan(height / (2 * f))
    vfov = math.degrees(vfov_rad)
    return vfov


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

    srdf_file = robot_id + ".srdf"
    output_mjcf_file = robot_id + ".mujoco.xml"
    output_original_mjcf_file = robot_id + ".mujoco.original.xml"

    srdf_file_dir = path.join(
        path.dirname(path.dirname(path.dirname(path.realpath(__file__)))),
        "srdf",
        robot_id,
    )
    mjcf_file_dir = path.join(
        path.dirname(path.dirname(path.dirname(path.realpath(__file__)))),
        "mjcf",
        robot_id,
    )
    robot_description_content = prepare_robot_description(robot_id)

    srdf_path = path.join(srdf_file_dir, srdf_file)
    output_path = path.join(mjcf_file_dir, output_mjcf_file)
    output_original_mjcf_path = path.join(mjcf_file_dir, output_original_mjcf_file)

    with open(srdf_path, "r") as f:
        srdf_tree = ET.parse(f)

    urdf_root = ET.fromstring(robot_description_content)
    # Create a new XML with one model
    robot_name = urdf_root.get("name")
    mujoco_tree = ET.Element("mujoco", model=robot_name)
    actuator_tree = ET.Element("actuator")
    contact_tree = ET.Element("contact")
    sensor_plugins = []

    srdf_root = srdf_tree.getroot()

    spec = mujoco.MjSpec.from_string(robot_description_content)
    spec.compile()
    # Add freejoint.
    base_footprint = spec.find_body("base_footprint")
    freejoint = base_footprint.add_freejoint(align=True)
    freejoint.name = "base_footprint_free_joint"
    spec.compile()

    # Add a site to the body with user data and read it back.
    for sensor in urdf_root.findall("sensor"):
        type = sensor.get("type")
        if type == "force_torque":
            sensor_name = sensor.get("name")
            parent = sensor.find("parent")
            link = parent.get("link")

            print(f"sensor_name: {sensor_name}, link: {link}")
            sensor_frame = spec.find_body(link)
            site = sensor_frame.add_site(
                name=sensor_name + "_site",
                pos=[0, 0, 0],
                # quat=[0, 0.707107, -0.707107, 0],
                # type=mujoco.mjtGeom.mjGEOM_BOX,
            )
            force = spec.add_sensor(
                name=sensor_name + "_force",
                objname=sensor_name + "_site",
                type=mujoco.mjtSensor.mjSENS_FORCE,
                objtype=mujoco.mjtObj.mjOBJ_SITE,
            )
            torque = spec.add_sensor(
                name=sensor_name + "_torque",
                objname=sensor_name + "_site",
                type=mujoco.mjtSensor.mjSENS_TORQUE,
                objtype=mujoco.mjtObj.mjOBJ_SITE,
            )
            spec.compile()
        elif type == "imu":
            sensor_name = sensor.get("name")
            parent = sensor.find("parent")
            link = parent.get("link")
            print(f"sensor_name: {sensor_name}, link: {link}")
            sensor_frame = spec.find_body(link)
            site = sensor_frame.add_site(
                name=sensor_name + "_site",
                pos=[0, 0, 0],
                # quat=[0, 0.707107, -0.707107, 0],
                # type=mujoco.mjtGeom.mjGEOM_BOX,
            )
            sensor_gyro = spec.add_sensor(
                name=sensor_name + "_gyro",
                objname=sensor_name + "_site",
                cutoff=34.9,
                type=mujoco.mjtSensor.mjSENS_GYRO,
                objtype=mujoco.mjtObj.mjOBJ_SITE,
            )
            sensor_framequat = spec.add_sensor(
                name=sensor_name + "_quaternion",
                objname=sensor_name + "_site",
                type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                objtype=mujoco.mjtObj.mjOBJ_SITE,
            )
            sensor_acc = spec.add_sensor(
                name=sensor_name + "_accelerometer",
                objname=sensor_name + "_site",
                type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                objtype=mujoco.mjtObj.mjOBJ_SITE,
            )
        elif type == "camera":
            sensor_name = sensor.get("name")
            parent = sensor.find("parent")
            image = sensor.find("camera").find("image")
            width = image.get("width")
            height = image.get("height")
            hfov = image.get("hfov")
            fovy = calculate_vfov(float(width), float(height), float(hfov))
            link = parent.get("link")
            print(f"sensor_name: {sensor_name}, link: {link}")
            sensor_frame = spec.find_body(link)
            camera_in_mujoco_frame = sensor_frame.add_body(
                name=f"{sensor_name}_mujoco_frame", euler=[math.pi / 2, -math.pi / 2, 0]
            )
            camera_in_mujoco_frame.add_camera(
                name=sensor_name,
                mode=mujoco.mjtCamLight.mjCAMLIGHT_FIXED,
                fovy=fovy,
                resolution=[int(width), int(height)],
            )
            plugin_tree = ET.Element("plugin")
            plugin_tree.set("name", f"image_publisher_{sensor_name}")
            plugin_tree.set("plugin", "MujocoRosUtils::ImagePublisher")
            plugin_tree.set("objtype", "camera")
            plugin_tree.set("objname", sensor_name)
            config_dict = {
                "frame_id": sensor_name,
                "info_topic_name": f"{camera_topic_dict[sensor_name]}/camera_info",
                "width": width,
                "height": height,
                "publish_rate": "30",
            }
            if "depth" in sensor_name:
                config_dict["depth_topic_name"] = (
                    f"{camera_topic_dict[sensor_name]}/depth_raw"
                )
            else:
                config_dict["color_topic_name"] = (
                    f"{camera_topic_dict[sensor_name]}/image_raw"
                )
            for key, value in config_dict.items():
                tmp_element = ET.Element("config")
                tmp_element.set("key", key)
                tmp_element.set("value", value)
                plugin_tree.append(tmp_element)
            sensor_plugins.append(plugin_tree)
        else:
            sensor_name = sensor.get("name")
            print(f"Unsupported sensor type: {type} from {sensor_name}")

    mjcf = spec.to_xml()

    mechanical_reduction = {}
    for ros2_control in urdf_root.findall("ros2_control"):
        for transmission in ros2_control.findall("transmission"):
            joint = transmission.find("joint")
            joint_name = joint.get("name")
            gear = joint.find("mechanical_reduction").text
            mechanical_reduction[joint_name] = gear
            # print(f"joint_name: {joint_name}, gear: {gear}")

    for joint in urdf_root.findall("joint"):
        joint_name = joint.get("name")
        joint_type = joint.get("type")
        if joint_type == "revolute":
            # motor_name = joint_name + '_actuator'
            motor_name = joint_name.replace("_joint", "_actuator")
            motor_tree = ET.Element("motor")
            motor_tree.set("name", motor_name)
            gear = mechanical_reduction.get(joint_name, "1.0")
            motor_tree.set("gear", "20.0")
            motor_tree.set("joint", joint_name)
            limit = joint.find("limit")
            effort = float(limit.get("effort")) / float(gear)
            ctrlrange_str = f"{-effort} {effort}"
            motor_tree.set("ctrlrange", ctrlrange_str)
            actuator_tree.append(motor_tree)
            actuator = spec.add_actuator(
                name=motor_name,
                target=joint_name,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                gear=[float(gear), 0, 0, 0, 0, 0],
                ctrlrange=[-effort, effort],
            )
            # actuator.gainprm[0] = 1
            # actuator.dyntype = mujoco.mjtDyn.mjDYN_NONE
            # actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            # actuator.biastype = mujoco.mjtBias.mjBIAS_NONE

    mujoco_tree.append(actuator_tree)

    # 查找所有 link 元素并收集其名称和 inertial 信息
    urdf_links = []
    for link in urdf_root.findall("link"):
        link_name = link.get("name")
        urdf_links.append(link_name)

    for disable_collisions in srdf_root.findall("disable_collisions"):
        link1 = disable_collisions.get("link1")
        link2 = disable_collisions.get("link2")
        if link1 in urdf_links and link2 in urdf_links:
            exclude_tree = ET.Element("exclude")
            exclude_tree.set("body1", link1)
            exclude_tree.set("body2", link2)
            contact_tree.append(exclude_tree)
            spec.add_exclude(bodyname1=link1, bodyname2=link2)
        else:
            # 如果不在 urdf_links 中，打印 link1 和 link2
            print(f"未找到link: link1 = {link1}, link2 = {link2}")
    mujoco_tree.append(contact_tree)
    spec.compile()

    mjcf_string = spec.to_xml()
    mjcf_string_root = ET.fromstring(mjcf_string)
    sensor_root = mjcf_string_root.find("sensor")
    for sensor_plugin in sensor_plugins:
        sensor_root.append(sensor_plugin)
    mjcf_string = ET.tostring(mjcf_string_root, pretty_print=True, xml_declaration=True)
    # Write into output file
    xml_string = ET.tostring(mujoco_tree, pretty_print=True, xml_declaration=True)

    formatter = Formatter(
        indent="2",
        indent_char=" ",
        encoding_output="ISO-8859-1",
        selfclose=True,
        blanks=True,
    )

    formatted_xml = formatter.format_string(xml_string).decode("utf-8")
    formatted_mjcf = formatter.format_string(mjcf_string).decode("utf-8")

    with open(output_path, "w") as f:
        f.write(formatted_mjcf)
    print('Results written into "%s"' % output_path)

    # with open(output_original_mjcf_path, "w") as f:
    #     f.write(mjcf_string)
    # print('Results written into "%s"' % output_original_mjcf_path)


if __name__ == "__main__":
    main()
