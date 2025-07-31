from isaacsim import SimulationApp

app_config = {
        "headless": True,
        "hide_ui": False,
        "width": 1280,
        "height": 720,
        "renderer": "RaytracedLighting"  # Can also be PathTracing
    }

simulation_app = SimulationApp(launch_config=app_config)

# streaming via WebRTC (optional), useful when using headless mode
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")

import argparse
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from lib.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import SurfaceGripper
# from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
# asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
    )
)
ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))
cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([0.3, 0.3, 0.3]),
        prim_path="/World/Cube",
        scale=np.array([0.0515, 0.0515, 0.0515]),
        size=1.0,
        color=np.array([1, 0, 0]),
    )
)
my_world.scene.add_default_ground_plane()
ur10.gripper.set_default_state(opened=True)
my_world.reset()

my_controller = PickPlaceController(name="pick_place_controller", gripper=ur10.gripper, robot_articulation=ur10)
articulation_controller = ur10.get_articulation_controller()

# Add camera to end-effector
camera = Camera(
    prim_path="/World/UR10/ee_link/Camera",
    name="ee_camera",
    translation=np.array([0.0, 0.0, 0.22]),  # offset from ee_link
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 45, 0]), degrees=True),
)

my_world.scene.add(camera)
camera.initialize()
# Configure end-effector camera intrinsics
camera.set_projection_mode("perspective")
# Example: set a 12 mm focal length and a 6.4x4.8 mm sensor (like 1/2" class)
camera.set_horizontal_aperture(20.955)
camera.set_vertical_aperture(15.2908)
camera.set_focal_length(12.0)
print("[EE Camera] focal length (mm):", camera.get_focal_length())
print("[EE Camera] sensor (mm):", camera.get_horizontal_aperture(), "x", camera.get_vertical_aperture())

# Add fixed world camera viewing the robot base (fixed in /World)
world_cam = Camera(
    prim_path="/World/FixedCamera",
    name="fixed_camera",
    position=np.array([1.5, 1.5, 0.4]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0,0,-135]), extrinsic=False, degrees=True), # ZYX
)
my_world.scene.add(world_cam)
world_cam.initialize()
# Configure fixed world camera intrinsics
world_cam.set_projection_mode("perspective")
# Example: set a 25 mm focal length and a 36x24 mm sensor (full-frame)
world_cam.set_horizontal_aperture(36.0)
world_cam.set_vertical_aperture(24.0)
world_cam.set_focal_length(18.0)
print("[World Camera] focal length (mm):", world_cam.get_focal_length())
print("[World Camera] sensor (mm):", world_cam.get_horizontal_aperture(), "x", world_cam.get_vertical_aperture())

i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=cube.get_local_pose()[0],
            placing_position=np.array([0.7, 0.7, 0.0515 / 2.0]),
            current_joint_positions=ur10.get_joint_positions(),
            end_effector_offset=np.array([0, 0, 0.02]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
    if args.test is True:
        break


simulation_app.close()
