import sys
import re
import os

import carb
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# In older versions of Isaac Sim (prior to 4.0), SimulationApp is imported from
# omni.isaac.kit rather than isaacsim.
try:
    from isaacsim import SimulationApp
except:
    from omni.isaac.kit import SimulationApp

GR1T2_STAGE_PATH = "/GR1T2"
GR1T2_USD_PATH = "/FFTAI/GR1T2/GR1T2.usd"
CAMERA_PRIM_PATH = f"{GR1T2_STAGE_PATH}/panda_hand/geometry/realsense/realsense_camera"
BACKGROUND_STAGE_PATH = "/World/defaultGroundPlane"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Hospital/hospital.usd"
# NUCLEUS_ASSET_ROOT_DIR = carb.settings.get_settings().get(
#     "/persistent/isaac/asset_root/cloud")
# ISAAC_NUCLEUS_DIR = f"{NUCLEUS_ASSET_ROOT_DIR}/Isaac"
# ISAACLAB_NUCLEUS_DIR = f"{ISAAC_NUCLEUS_DIR}/IsaacLab"

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

# print(assets_root_path + BACKGROUND_USD_PATH)

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
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
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

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    # og.Controller.set(
    #     og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    # )

simulation_context.stop()
simulation_app.close()
