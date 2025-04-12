import sys
import re
import os
import xml.etree.ElementTree as ET
from xmlformatter import Formatter
import xacro
import carb
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import (
#     Command,
#     FindExecutable,
#     PathJoinSubstitution,
# )
# from launch_ros.substitutions import FindPackageShare
# In older versions of Isaac Sim (prior to 4.0), SimulationApp is imported from
# omni.isaac.kit rather than isaacsim.
try:
    from isaacsim import SimulationApp
except:
    from omni.isaac.kit import SimulationApp

GR1T2_STAGE_PATH = "/GR1T2"
GR1T2_USD_PATH = "/FFTAI/GR1T2/GR1T2.usd"
CAMERA_PRIM_PATH = f"{GR1T2_STAGE_PATH}/panda_hand/geometry/realsense/realsense_camera"
BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Hospital/hospital.usd"
BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"
GRAPH_PATH = "/ActionGraph"
REALSENSE_VIEWPORT_NAME = "realsense_viewport"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

# fmt: off
from omni.isaac.version import get_version

# Check the major version number of Isaac Sim to see if it's four digits, corresponding
# to Isaac Sim 2023.1.1 or older.  The version numbering scheme changed with the
# Isaac Sim 4.0 release in 2024.
is_legacy_isaacsim = len(get_version()[2]) == 4


# More imports that need to compare after we create the app
from omni.isaac.core import SimulationContext  # noqa E402
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdLux, UsdPhysics
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    nucleus,
    prims,
    rotations,
    stage,
    viewports,
)

from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf, UsdGeom  # noqa E402
import omni.graph.core as og  # noqa E402
import omni.isaac.lab.sim as sim_utils
import omni
# fmt: on

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

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


def design_scene():
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

    # Locate Isaac Sim assets folder to load environment and robot stages
    assets_root_path = nucleus.get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()

    # Loading the simple_room environment
    # stage.add_reference_to_stage(
    #     assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
    # )
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Loading the robot USD
    robot_usd_filepath = os.path.join(
        get_package_share_directory("human_description"),
        "usd",
    )

    prims.create_prim(
        GR1T2_STAGE_PATH,
        "Xform",
        position=np.array([0, 0, 0.93]),
        orientation=rotations.gf_rotation_to_np_array(
            Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
        usd_path=robot_usd_filepath + GR1T2_USD_PATH,
    )

    simulation_app.update()

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
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick",
                     "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",
                     "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "ArticulationController.inputs:execIn",
                    ),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishJointState.inputs:timeStamp",
                    ),
                    ("ReadSimTime.outputs:simulationTime",
                     "PublishClock.inputs:timeStamp"),
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
                ],
                og.Controller.Keys.SET_VALUES: og_keys_set_values,
            },
        )
    except Exception as e:
        print(e)

    # Setting the /Franka target prim to Publish JointState node
    set_target_prims(
        primPath="/ActionGraph/PublishJointState", targetPrimPaths=["/GR1T2/base_link"]
    )

    simulation_app.update()

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

    # need to initialize physics getting any articulation..etc
    simulation_context.initialize_physics()

    simulation_context.play()


def main():
    """Main function."""
    design_scene()

    while simulation_app.is_running():

        # Run with a fixed step size
        simulation_context.step(render=True)

        # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
        # og.Controller.set(
        #     og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
        # )

    simulation_context.stop()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
