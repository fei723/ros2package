import argparse
import os
import math
import os
import xacro
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="Tutorial on spawning and interacting with an articulation.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
# fmt: off
import torch

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdLux, UsdPhysics
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    nucleus,
    prims,
    rotations,
    stage,
    viewports,
)

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
import omni.graph.core as og  # noqa E402
from omni.isaac.version import get_version
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from omni.isaac.sensor import _sensor
from omni.isaac.sensor import IMUSensor
# from omni.isaac.lab.sensors import ImuCfg
from omni.isaac.lab.devices import Se3Keyboard
import omni
import numpy as np

##
# Pre-defined configs
##
# from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip
from humanoid_lab.assets.fftai import FFTAI_GR1T2_CFG  # isort: skip

# fmt: on

GR1T2_STAGE_PATH = "/GR1T2"
GRAPH_PATH = "/ActionGraph"

teleop_interface = Se3Keyboard(pos_sensitivity=0.1, rot_sensitivity=0.1)
need_reset = False

joint_stiffness = {
    "l_hip_roll_joint": 251.625,
    "l_hip_yaw_joint": 362.5214,
    "l_hip_pitch_joint": 200,
    "l_knee_pitch_joint": 200,
    "l_ankle_pitch_joint": 10.9805,
    "l_ankle_roll_joint": 0.25,

    "r_hip_roll_joint": 251.625,
    "r_hip_yaw_joint": 362.5214,
    "r_hip_pitch_joint": 200,
    "r_knee_pitch_joint": 200,
    "r_ankle_pitch_joint": 10.9805,
    "r_ankle_roll_joint": 0.25,

    "waist_yaw_joint": 362.5214,
    "waist_pitch_joint": 362.5214,
    "waist_roll_joint": 362.5214,

    "neck_roll_joint": 10.0,
    "neck_pitch_joint": 10.0,
    "neck_yaw_joint": 10.0,

    "l_shoulder_pitch_joint": 92.85,
    "l_shoulder_roll_joint": 92.85,
    "l_elbow_yaw_joint": 112.06,
    "l_elbow_pitch_joint": 112.06,
    "l_wrist_yaw_joint": 10.0,
    "l_wrist_roll_joint": 10.0,
    "l_wrist_pitch_joint": 10.0,

    "r_shoulder_pitch_joint": 92.85,
    "r_shoulder_roll_joint": 92.85,
    "r_elbow_yaw_joint": 112.06,
    "r_elbow_pitch_joint": 112.06,
    "r_wrist_yaw_joint": 10.0,
    "r_wrist_roll_joint": 10.0,
    "r_wrist_pitch_joint": 10.0,
}

joint_damping = {
    "l_hip_roll_joint": 14.72,
    "l_hip_yaw_joint": 10.0833,
    "l_hip_pitch_joint": 11,
    "l_knee_pitch_joint": 11,
    "l_ankle_pitch_joint": 0.5991,
    "l_ankle_roll_joint": 0.01,

    "r_hip_roll_joint": 14.72,
    "r_hip_yaw_joint": 10.0833,
    "r_hip_pitch_joint": 11,
    "r_knee_pitch_joint": 11,
    "r_ankle_pitch_joint": 0.5991,
    "r_ankle_roll_joint": 0.01,

    "waist_yaw_joint": 10.0833,
    "waist_pitch_joint": 10.0833,
    "waist_roll_joint": 10.0833,

    "neck_roll_joint": 1.0,
    "neck_pitch_joint": 1.0,
    "neck_yaw_joint": 1.0,

    "l_shoulder_pitch_joint": 2.575,
    "l_shoulder_roll_joint": 2.575,
    "l_elbow_yaw_joint": 3.1,
    "l_elbow_pitch_joint": 3.1,
    "l_wrist_yaw_joint": 1.0,
    "l_wrist_roll_joint": 1.0,
    "l_wrist_pitch_joint": 1.0,

    "r_shoulder_pitch_joint": 2.575,
    "r_shoulder_roll_joint": 2.575,
    "r_elbow_yaw_joint": 3.1,
    "r_elbow_pitch_joint": 3.1,
    "r_wrist_yaw_joint": 1.0,
    "r_wrist_roll_joint": 1.0,
    "r_wrist_pitch_joint": 1.0,
}

def prepare_robot_description():

    robot_xacro_filepath = os.path.join(
        get_package_share_directory("human_description"),
        "robots",
        "GR1T2",
        "GR1T2.urdf.xacro",
    )
    robot_description_content = xacro.process_file(
        robot_xacro_filepath, mappings={'ros2_control_plugin': 'isaac',
                                        'command_interface': "['position']", }
    ).toprettyxml(indent="  ")

    return robot_description_content


def get_drive(prim_path, drive_type):
    joint_prim = get_prim_at_path(prim_path)
    # check if prim has joint drive applied on it
    drive = UsdPhysics.DriveAPI.Get(joint_prim, drive_type)
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)
    return drive


def find_prim_path_by_name(start_stage, prim_name):
    for prim in start_stage.GetAllChildren():
        if prim.GetName() == prim_name:
            return prim.GetPath().pathString
        else:
            ret = find_prim_path_by_name(prim, prim_name)
            if not ret == None:
                return ret


def update_drive():
    robot_description_content = prepare_robot_description()
    urdf_root = ET.fromstring(robot_description_content)

    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break
    print('Robot name: "%s"' % robot_name)
    urdf_joints = []
    joint_type = []

    for child in urdf_root.findall('./joint'):
        if child.attrib["type"] == "continuous":
            urdf_joints.append(child)
            joint_type.append("angular")
        elif child.attrib["type"] == "revolute":
            urdf_joints.append(child)
            joint_type.append("angular")
        elif child.attrib["type"] == "prismatic":
            urdf_joints.append(child)
            joint_type.append("linear")

    stage_handle = omni.usd.get_context().get_stage()
    for index, joint in enumerate(urdf_joints):
        joint_prim_path = find_prim_path_by_name(stage_handle.GetPrimAtPath(
            "/" + robot_name), joint.attrib["name"])
        print("joint_prim_path: ", joint_prim_path)
        drive = get_drive(joint_prim_path, joint_type[index])

        if not drive.GetStiffnessAttr():
            drive.CreateStiffnessAttr(joint_stiffness[joint.attrib["name"]])
        else:
            drive.GetStiffnessAttr().Set(joint_stiffness[joint.attrib["name"]])
        if not drive.GetDampingAttr():
            drive.CreateDampingAttr(joint_damping[joint.attrib["name"]])
        else:
            drive.GetDampingAttr().Set(joint_damping[joint.attrib["name"]])


def reset_cb():
    global need_reset
    """Dummy callback function executed when the key 'TAB' is pressed."""
    need_reset = True
    print("Reset callback")


def design_scene() -> tuple[dict]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    teleop_interface.add_callback("TAB", reset_cb)

    # Articulation
    robot_cfg = FFTAI_GR1T2_CFG.copy()
    robot_cfg.prim_path = GR1T2_STAGE_PATH
    robot_cfg.spawn.articulation_props.enabled_self_collisions = False

    # robot_cfg.spawn.articulation_props.fix_root_link = True

    gr1t2 = Articulation(cfg=robot_cfg)

    pelvis_imu = IMUSensor(
        prim_path="/GR1T2/pelvis_imu_frame/pelvis_imu",
        name="pelvis_imu",
        frequency=60,
        translation=np.array([0, 0, 0]),
        # orientation=np.array([1, 0, 0, 0]),
        linear_acceleration_filter_size=10,
        angular_velocity_filter_size=10,
        orientation_filter_size=10,
    )

    # imu = ImuCfg(
    #     prim_path="/GR1T2/pelvis_imu_frame/pelvis_imu",
    #     update_period=0.1,
    # )

    # return the scene information
    scene_entities = {"GR1T2": gr1t2, "pelvis_imu": pelvis_imu}
    return scene_entities


def create_action_graph():

    # Check the major version number of Isaac Sim to see if it's four digits, corresponding
    # to Isaac Sim 2023.1.1 or older.  The version numbering scheme changed with the
    # Isaac Sim 4.0 release in 2024.
    is_legacy_isaacsim = len(get_version()[2]) == 4
    try:
        ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
        print("Using ROS_DOMAIN_ID: ", ros_domain_id)
    except ValueError:
        print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
        ros_domain_id = 0
    except KeyError:
        print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
        ros_domain_id = 0
    # Create an action graph with ROS component nodes
    try:
        og_keys_set_values = [
            ("Context.inputs:domain_id", ros_domain_id),
            # Set the /Franka target prim to Articulation Controller node
            ("ArticulationController.inputs:robotPath", GR1T2_STAGE_PATH),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("ReadIMU.inputs:imuPrim", "/GR1T2/pelvis_imu_frame/pelvis_imu"),
            ("PublishImu.inputs:frameId", "pelvis_imu_frame"),
            ("PublishImu.inputs:topicName", "pelvis_imu"),
        ]

        # In older versions of Isaac Sim, the articulation controller node contained a
        # "usePath" checkbox input that should be enabled.
        if is_legacy_isaacsim:
            og_keys_set_values.insert(
                1, ("ArticulationController.inputs:usePath", True))

        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    (
                        "SubscribeJointState",
                        "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "ArticulationController",
                        "omni.isaac.core_nodes.IsaacArticulationController",
                    ),
                    ("ReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                    ("PublishImu", "omni.isaac.ros2_bridge.ROS2PublishImu"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick",
                     "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",
                     "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "ArticulationController.inputs:execIn",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishImu.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("Context.outputs:context", "PublishImu.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishJointState.inputs:timeStamp",
                    ),
                    ("ReadSimTime.outputs:simulationTime",
                     "PublishClock.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime",
                     "PublishImu.inputs:timeStamp"),
                    (
                        "SubscribeJointState.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                    ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                    ("ReadIMU.outputs:linAcc",
                     "PublishImu.inputs:linearAcceleration"),
                    ("ReadIMU.outputs:orientation",
                     "PublishImu.inputs:orientation"),
                ],
                og.Controller.Keys.SET_VALUES: og_keys_set_values,
            },
        )
    except Exception as e:
        print(e)


def quaternion_to_euler(w, x, y, z):
    """
    将四元数转换为欧拉角（偏航角、俯仰角、滚转角）。

    :param w: 四元数的实部
    :param x: 四元数的x分量
    :param y: 四元数的y分量
    :param z: 四元数的z分量
    :return: 欧拉角（偏航角、俯仰角、滚转角）的元组
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# def is_imbalance(orientation):


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation]):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["GR1T2"]

    pelvis_imu = entities["pelvis_imu"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    global need_reset
    # Simulation loop
    while simulation_app.is_running():
        if need_reset:
            need_reset = False
            root_state = robot.data.default_root_state.clone()
            print(root_state)
            robot.write_root_state_to_sim(root_state)
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(
            ), robot.data.default_joint_vel.clone()
            print(joint_pos)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            sim.step(render=True)
            print("[INFO]: Resetting robot state...")

        sim.step(render=True)
        # Update buffers
        # robot.update(sim_dt)


def main():
    """Main function."""
    extensions.enable_extension("omni.isaac.ros2_bridge")
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device='cpu')
    sim = SimulationContext(sim_cfg)
    # Design scene
    scene_entities = design_scene()
    # scene_origins = torch.tensor(scene_origins, device=sim.device)
    simulation_app.update()
    create_action_graph()
    set_target_prims(
        primPath="/ActionGraph/PublishJointState", targetPrimPaths=["/GR1T2/base_link"]
    )
    simulation_app.update()
    # update_drive()
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # sim.initialize_physics()
    # sim.play()
    # Run the simulator
    run_simulator(sim, scene_entities)
    sim.stop()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
