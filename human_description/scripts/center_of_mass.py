#!/usr/bin/env python3

import os
import sys
import xacro
import rclpy
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


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

    print("Generate a robot description of the '%s'" % robot_id)

    return robot_description_content

class RobotID:
    def __init__(self, robot_id: str):
        self.robot_id = robot_id

class CenterOfMassPublisher(Node, RobotID):
    def __init__(self, robot_id: str):
        super().__init__('center_of_mass_publisher')

        # 创建发布器，发布 PointStamped 消息到 /robot_center_of_mass 主题
        self.publisher_ = self.create_publisher(
            PointStamped, '/robot_center_of_mass', 10)

        # 加载机器人模型和URDF文件
        robot_description_content = prepare_robot_description(robot_id)

        self.model = pin.buildModelFromXML(
            robot_description_content, pin.JointModelFreeFlyer())

        self.data = self.model.createData()

        # print("Model parameters:")
        # print(f"  model.nv = {self.model.nv}")
        # print(f"  model.nq = {self.model.nq}")
        # print(f"  model.njoints = {self.model.njoints}")
        # print(f"  model.nbodies = {self.model.nbodies}")
        # print(f"  model.nframes = {self.model.nframes}\n")

        com = pin.centerOfMass(self.model, self.data)
        total_mass = pin.computeTotalMass(self.model)

        # 打印机器人总质量
        self.get_logger().info(f"Total mass of the robot: {total_mass:.6f} kg")
        self.get_logger().info(
            f"Center of Mass: x={com[0]:.6f}, y={com[1]:.6f}, z={com[2]:.6f}")

        # 定时器，定时发布质心信息
        self.timer = self.create_timer(1.0, self.publish_com)

    def publish_com(self):
        # do whatever, e.g. compute the center of mass position expressed in the world frame
        com = pin.centerOfMass(self.model, self.data)

        # 创建 PointStamped 消息
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = com[0]
        msg.point.y = com[1]
        msg.point.z = com[2]

        # 发布质心数据
        self.publisher_.publish(msg)


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

    rclpy.init(args=args)

    com_publisher = CenterOfMassPublisher(robot_id)

    try:
        rclpy.spin(com_publisher)
    except KeyboardInterrupt:
        pass

    # 清理
    com_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
