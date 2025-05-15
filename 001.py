from isaacsim import SimulationApp

# app configuration
# for more details:
# /isaac-sim/exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py
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

#------------------------------------

# add World (inherits from SimulationContext)
# more details: 
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.api/docs/index.html#isaacsim.core.api.world.World
from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

#-----------------
# get assets root path, and import robots from usd
import sys
import carb
import numpy as np
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.api.robots import Robot

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
print(f"assets_root_path: {assets_root_path}")

# https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_robots.html 
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
# asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
# TODO: find out why ur10.usd cant be loaded in different position, world pose is set but UI remains at origin

add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
print("added references to stage")
articulated_system_1 = world.scene.add(Robot(prim_path="/World/Franka", name="my_franka"))
print("added articulated systems to world")
articulated_system_1.set_world_pose(position=np.array([0.0, 1.0, 0.0]) / get_stage_units())
print("Franka pose after set_world_pose:", articulated_system_1.get_world_pose())

#-----------------
# urdf import

import omni.kit.commands
# from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics, UsdGeom
from omni.kit.viewport.utility.camera_state import ViewportCameraState


# Setting up import configuration:
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.distance_scale = 1.0

# Get path to extension data:
# Import URDF, prim_path contains the path the path to the usd prim in the stage.
# TODO: find how to spawn in a specific location in the world
robot_urdf_path = "/isaac-sim/my_examples/assets/ur10/urdf/ur10.urdf"
# robot_urdf_path = "/isaac-sim/my_examples/assets/turtlebot3_waffle_pi/urdf/turtlebot3_waffle_pi.urdf"
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=robot_urdf_path,
    import_config=import_config,
    get_articulation_root=True,
)

camera_state = ViewportCameraState("/OmniverseKit_Persp")
camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 0.5), True)
camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.0), True)

stage = omni.usd.get_context().get_stage()

# Enable physics
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
# Set gravity
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
# Set solver settings
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS")

#-----------------------------




#----------------------
# keep it running
while simulation_app._app.is_running() and not simulation_app.is_exiting():
    # Run in realtime mode, we don't specify the step size
    simulation_app.update()

simulation_app.close()  # Cleanup application