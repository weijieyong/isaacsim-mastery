# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#  
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

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
import os

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

# Get the path to the UR10e USD file
ur10e_usd_path = "/isaac-sim/ur10e/ur10e_robotiq2f-140.usd"

if not os.path.exists(ur10e_usd_path):
    carb.log_error(f"Could not find UR10e USD file at: {ur10e_usd_path}")
    simulation_app.close()
    sys.exit()

print(f"Loading UR10e from: {ur10e_usd_path}")

# Create world and add ground plane
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Add the UR10e robot to the stage
add_reference_to_stage(usd_path=ur10e_usd_path, prim_path="/World/UR10e")
ur10e_robot = my_world.scene.add(Robot(prim_path="/World/UR10e", name="ur10e_robot"))

print("UR10e robot loaded successfully!")

initial_joint_positions = np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0,
                                    0,0,0,0,0,0,0,0])


# Simple simulation loop
for i in range(5):
    print(f"Reset {i+1}...")
    my_world.reset()
    
    # Position the robot
    ur10e_robot.set_world_pose(position=np.array([0.0, 0.0, 0.0]) / get_stage_units())

    ur10e_robot.set_joint_positions(initial_joint_positions)
    
    # Run simulation for a few steps
    for j in range(200):
        my_world.step(render=True)
        
        # Print robot information at step 100
        if j == 100:
            print("UR10e joint positions:", ur10e_robot.get_joint_positions())
            print("UR10e world pose:", ur10e_robot.get_world_pose())
    
    if args.test is True:
        break

print("Simulation completed.")
simulation_app.close()
