import argparse
import os
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on M92UW.")
parser.add_argument("--num_envs", type=int, default=1,
                    help="Number of environments to spawn.")
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


from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    rotations,
    stage,
)

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import RigidObjectCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.utils.assets as assets_utils


import omni.graph.core as og  # noqa E402
from omni.isaac.version import get_version
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
# from omni.isaac.sensor import _sensor
from omni.isaac.sensor import IMUSensor
from omni.isaac.lab.devices import Se3Keyboard
from pxr import Gf, UsdGeom  # noqa E402
import omni
import numpy as np

##
# Pre-defined configs
##
from humanoid_lab.assets.m92uw import MI_M92UW_CFG  # isort: skip


# fmt: on

M92UW_STAGE_PATH = "/M92UW"
GRAPH_PATH = "/ActionGraph"
CAMERA_VIEWPORT_NAME = "camera_viewport"

teleop_interface = Se3Keyboard(pos_sensitivity=0.1, rot_sensitivity=0.1)
need_reset = False


def reset_cb():
    global need_reset
    """Dummy callback function executed when the key 'TAB' is pressed."""
    need_reset = True
    print("Reset callback")


@configclass
class SensorsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_utils.ISAAC_NUCLEUS_DIR}/Environments/Simple_Room/simple_room.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                  kinematic_enabled=True),
        ),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    teleop_interface.add_callback("TAB", reset_cb)

    # Articulation
    robot_cfg = MI_M92UW_CFG.copy()
    robot_cfg.prim_path = M92UW_STAGE_PATH
    robot_cfg.spawn.articulation_props.enabled_self_collisions = False

    # robot_cfg.spawn.articulation_props.fix_root_link = True

    robot_cfg.init_state.pos = np.array([0, 1.5, -0.76957])
    robot_cfg.init_state.rot = rotations.gf_rotation_to_np_array(
        Gf.Rotation(Gf.Vec3d(0, 0, 1), -90))

    robot = robot_cfg

    # head_imu = IMUSensor(
    #     prim_path="/M92UW/head_imu_frame/head_imu",
    #     name="head_imu",
    #     frequency=60,
    #     translation=np.array([0, 0, 0]),
    #     # orientation=np.array([1, 0, 0, 0]),
    #     linear_acceleration_filter_size=10,
    #     angular_velocity_filter_size=10,
    #     orientation_filter_size=10,
    # )

    # head_imu_frame: ImuCfg = ImuCfg(
    #     prim_path="/M92UW/head_imu_frame",
    #     gravity_bias=(0.0, 0.0, 9.81),
    # )
    # imu = Imu(cfg=head_imu_frame)

    # head_color_camera_cfg = TiledCameraCfg(
    #     height=1920,
    #     width=1080,
    #     offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, 4.0), rot=(
    #         1.0, 0.0, 0.0, 0.0), convention="ros"),
    #     prim_path="/M92UW/head_camera_color_frame/head_camera",
    #     data_types=["rgb", "distance_to_camera"],
    #     spawn=sim_utils.PinholeCameraCfg(
    #         focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
    #     ),
    # )
    # TiledCamera(head_color_camera_cfg)

    cracker_box_cfg = AssetBaseCfg(
        prim_path="/World/cracker_box",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_utils.ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.2, 0.25, 0.15), rot=rotations.gf_rotation_to_np_array(
                Gf.Rotation(Gf.Vec3d(1, 0, 0), -90))),
    )

    sugar_box_cfg = AssetBaseCfg(
        prim_path="/World/sugar_box",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_utils.ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                  kinematic_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.07, 0.25, 0.1), rot=rotations.gf_rotation_to_np_array(
                Gf.Rotation(Gf.Vec3d(0, 1, 0), -90))),
    )

    soup_can_cfg = AssetBaseCfg(
        prim_path="/World/soup_can",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_utils.ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                  kinematic_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.1, 0.25, 0.10), rot=rotations.gf_rotation_to_np_array(
                Gf.Rotation(Gf.Vec3d(1, 0, 0), -90))),
    )

    mustard_bottle_cfg = AssetBaseCfg(
        prim_path="/World/mustard_bottle",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{assets_utils.ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                  kinematic_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, -0.15, 0.12), rot=rotations.gf_rotation_to_np_array(
                Gf.Rotation(Gf.Vec3d(1, 0, 0), -90))),
    )


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
            ("ArticulationController.inputs:robotPath", M92UW_STAGE_PATH),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("ReadIMU.inputs:imuPrim", "/M92UW/head_imu_frame/head_imu"),
            ("PublishImu.inputs:frameId", "head_imu_frame"),
            ("PublishImu.inputs:topicName", "head_imu"),
            ("createViewport.inputs:name", CAMERA_VIEWPORT_NAME),
            ("createViewport.inputs:viewportId", 1),
            ("cameraHelperRgb.inputs:frameId", "head_camera"),
            ("cameraHelperRgb.inputs:topicName", "head_camera/color"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            ("cameraHelperInfo.inputs:frameId", "head_camera"),
            ("cameraHelperInfo.inputs:topicName", "head_camera/camera_info"),
            ("cameraHelperInfo.inputs:type", "camera_info"),
            ("cameraHelperDepth.inputs:frameId", "head_camera"),
            ("cameraHelperDepth.inputs:topicName", "head_camera/depth"),
            ("cameraHelperDepth.inputs:type", "depth"),
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
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    (
                        "getRenderProduct",
                        "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
                    ),
                    ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
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
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut",
                     "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport",
                     "getRenderProduct.inputs:viewport"),
                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    (
                        "getRenderProduct.outputs:renderProductPath",
                        "setCamera.inputs:renderProductPath",
                    ),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("setCamera.outputs:execOut",
                     "cameraHelperDepth.inputs:execIn"),
                    ("Context.outputs:context", "cameraHelperRgb.inputs:context"),
                    ("Context.outputs:context", "cameraHelperInfo.inputs:context"),
                    ("Context.outputs:context", "cameraHelperDepth.inputs:context"),
                    (
                        "getRenderProduct.outputs:renderProductPath",
                        "cameraHelperRgb.inputs:renderProductPath",
                    ),
                    (
                        "getRenderProduct.outputs:renderProductPath",
                        "cameraHelperInfo.inputs:renderProductPath",
                    ),
                    (
                        "getRenderProduct.outputs:renderProductPath",
                        "cameraHelperDepth.inputs:renderProductPath",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: og_keys_set_values,
            },
        )
    except Exception as e:
        print(e)


# def is_imbalance(orientation):


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = scene["robot"]

    # pelvis_imu = entities["pelvis_imu"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    global need_reset
    # Simulation loop
    while simulation_app.is_running():
        if need_reset:
            need_reset = False
            root_state = robot.data.default_root_state.clone()
            # print(root_state)
            robot.write_root_state_to_sim(root_state)
            joint_pos, joint_vel = (
                robot.data.default_joint_pos.clone(),
                robot.data.default_joint_vel.clone(),)
            # print(joint_pos)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            scene.reset()
            sim.step(render=True)
            print("[INFO]: Resetting robot state...")

        sim.step(render=True)
        scene.update(sim_dt)
        # Update buffers
        # robot.update(sim_dt)


def main():
    """Main function."""
    extensions.enable_extension("omni.isaac.ros2_bridge")
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device='cpu')
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=np.array(
        [3.0, 3.0, 3.0]), target=np.array([0, 0, 0.5]))
    # Design scene
    scene_cfg = SensorsSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    IMUSensor(
        prim_path="/M92UW/head_imu_frame/head_imu",
        name="head_imu",
        frequency=60,
        translation=np.array([0, 0, 0]),
        # orientation=np.array([1, 0, 0, 0]),
        linear_acceleration_filter_size=10,
        angular_velocity_filter_size=10,
        orientation_filter_size=10,
    )
    simulation_app.update()
    create_action_graph()
    set_target_prims(
        primPath="/ActionGraph/PublishJointState", targetPrimPaths=["/M92UW/base_footprint"]
    )

    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath(GRAPH_PATH + "/setCamera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[
            "/M92UW/head_front_left_fishye_color_frame/head_front_left_fishye"],
    )
    simulation_app.update()

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    sim.initialize_physics()
    sim.play()
    # Dock the second camera window
    viewport = omni.ui.Workspace.get_window("Viewport")
    rs_viewport = omni.ui.Workspace.get_window(CAMERA_VIEWPORT_NAME)
    # rs_viewport.dock_in(viewport, omni.ui.DockPosition.RIGHT)

    realsense_prim = camera_prim = UsdGeom.Camera(
        stage.get_current_stage().GetPrimAtPath(
            "/M92UW/head_camera_color_frame/head_camera_color")
    )
    realsense_prim.GetHorizontalApertureAttr().Set(20.955)
    realsense_prim.GetVerticalApertureAttr().Set(15.7)
    realsense_prim.GetFocalLengthAttr().Set(18.8)
    realsense_prim.GetFocusDistanceAttr().Set(400)
    # Run the simulator
    run_simulator(sim, scene)
    sim.stop()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
