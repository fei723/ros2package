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
from pxr import Gf, UsdGeom  # noqa E402
import omni
import numpy as np

##
# Pre-defined configs
##
# from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip
from humanoid_lab.assets.m92u0 import MI_M92U0_CFG  # isort: skip

# fmt: on

M92U0_STAGE_PATH = "/M92U0"
GRAPH_PATH = "/ActionGraph"

teleop_interface = Se3Keyboard(pos_sensitivity=0.1, rot_sensitivity=0.1)
need_reset = False


def reset_cb():
    global need_reset
    """Dummy callback function executed when the key 'TAB' is pressed."""
    need_reset = True
    print("Reset callback")


def design_scene() -> tuple[dict]:
    """Designs the scene."""
    assets_root_path = nucleus.get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()
    print(assets_root_path)

    stage.add_reference_to_stage(
        assets_root_path +
        "/Isaac/Environments/Simple_Room/simple_room.usd", "/World/defaultGroundPlane"
    )

    viewports.set_camera_view(eye=np.array([3.0, 3.0, 2.0]), target=np.array([0, 0, 0]))
    # Ground-plane
    # cfg = sim_utils.GroundPlaneCfg()
    # cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    teleop_interface.add_callback("TAB", reset_cb)

    # Articulation
    robot_cfg = MI_M92U0_CFG.copy()
    robot_cfg.prim_path = M92U0_STAGE_PATH
    robot_cfg.spawn.articulation_props.enabled_self_collisions = False

    robot_cfg.spawn.articulation_props.fix_root_link = True

    robot_cfg.init_state.pos = np.array([0, 1.5, 0.938-0.76957])
    robot_cfg.init_state.rot = rotations.gf_rotation_to_np_array(
        Gf.Rotation(Gf.Vec3d(0, 0, 1), -90))

    robot = Articulation(cfg=robot_cfg)

    # pelvis_imu = IMUSensor(
    #     prim_path="/M92U0/pelvis_imu_frame/pelvis_imu",
    #     name="pelvis_imu",
    #     frequency=60,
    #     translation=np.array([0, 0, 0]),
    #     # orientation=np.array([1, 0, 0, 0]),
    #     linear_acceleration_filter_size=10,
    #     angular_velocity_filter_size=10,
    #     orientation_filter_size=10,
    # )

    # imu = ImuCfg(
    #     prim_path="/M92U0/pelvis_imu_frame/pelvis_imu",
    #     update_period=0.1,
    # )
    prims.create_prim(
        "/World/cracker_box",
        "Xform",
        position=np.array([-0.2, -0.25, 0.15]),
        orientation=rotations.gf_rotation_to_np_array(
            Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
        usd_path=assets_root_path
        + "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd",
    )

    prims.create_prim(
        "/World/sugar_box",
        "Xform",
        position=np.array([-0.07, -0.25, 0.1]),
        orientation=rotations.gf_rotation_to_np_array(
            Gf.Rotation(Gf.Vec3d(0, 1, 0), -90)),
        usd_path=assets_root_path
        + "/Isaac/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd",
    )

    prims.create_prim(
        "/World/soup_can",
        "Xform",
        position=np.array([0.1, -0.25, 0.10]),
        orientation=rotations.gf_rotation_to_np_array(
            Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
        usd_path=assets_root_path
        + "/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
    )
    prims.create_prim(
        "/World/mustard_bottle",
        "Xform",
        position=np.array([0.0, 0.15, 0.12]),
        orientation=rotations.gf_rotation_to_np_array(
            Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
        usd_path=assets_root_path
        + "/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd",
    )

    # return the scene information
    scene_entities = {"M92U0": robot}
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
            ("ArticulationController.inputs:robotPath", M92U0_STAGE_PATH),
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
                    (
                        "OnPlaybackTick.outputs:tick",
                        "ArticulationController.inputs:execIn",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
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


# def is_imbalance(orientation):


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation]):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["M92U0"]

    # pelvis_imu = entities["pelvis_imu"]
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
        primPath="/ActionGraph/PublishJointState", targetPrimPaths=["/M92U0"]
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
