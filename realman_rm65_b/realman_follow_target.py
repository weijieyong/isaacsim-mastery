"""
Isaac Sim Robot Simulation - Realman RM65-B Manipulator
This script sets up a robot arm simulation with motion control and target following.
"""

# ================================
# SIMULATION APP INITIALIZATION
# ================================
from isaacsim import SimulationApp

# Configure simulation parameters
app_config = {
    "headless": True,          # Run without GUI for better performance
    "hide_ui": False,          # UI visibility (no effect in headless mode)
    "width": 1280,            # Render resolution width
    "height": 720,            # Render resolution height
    "renderer": "RaytracedLighting"  # Rendering engine (alternative: "PathTracing")
}

# Initialize the simulation application
simulation_app = SimulationApp(launch_config=app_config)

# Enable WebRTC streaming for remote access (useful in headless mode)
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.livestream.webrtc")

# ================================
# IMPORTS AND ARGUMENT PARSING
# ================================
import argparse
import numpy as np

# Isaac Sim core imports
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.api.objects import VisualCuboid

# USD/Pixar imports for scene manipulation
from pxr import UsdGeom, Gf

# Motion generation imports
import isaacsim.robot_motion.motion_generation as mg

# Parse command line arguments
parser = argparse.ArgumentParser(description="Robot simulation with motion control")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

# ================================
# WORLD SETUP AND SCENE CREATION
# ================================
def setup_world():
    """Initialize the simulation world and add ground plane."""
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    return world

def load_robot_asset(world, asset_path, prim_path):
    """Load robot USD asset into the scene."""
    add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
    return world.scene.stage

def configure_robot_transform(stage, prim_path, position, rotation_degrees):
    """Configure robot position and orientation in the scene."""
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    
    # Get existing transform operations
    translate_ops = xform.GetOrderedXformOps()
    
    # Update or set position
    translate_op = _find_transform_op(translate_ops, UsdGeom.XformOp.TypeTranslate)
    if translate_op:
        translate_op.Set(Gf.Vec3d(*position))
    
    # Update or set rotation
    rotate_op = _find_transform_op(translate_ops, UsdGeom.XformOp.TypeRotateXYZ)
    if rotate_op:
        rotate_op.Set(Gf.Vec3f(*rotation_degrees))
    else:
        # Create new rotation operation if none exists
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_degrees))

def _find_transform_op(ops_list, op_type):
    """Helper function to find specific transform operation type."""
    for op in ops_list:
        if op.GetOpType() == op_type:
            return op
    return None

# ================================
# ROBOT MANIPULATOR SETUP
# ================================
def setup_manipulator(world, prim_path, name, default_joint_positions):
    """Create and configure the robot manipulator."""
    manipulator = SingleArticulation(
        prim_path=prim_path,
        name=name,
    )
    
    # Set default joint positions
    manipulator.set_joints_default_state(positions=default_joint_positions)
    
    # Add to world scene
    world.scene.add(manipulator)
    return manipulator

def create_target_cube(world, name, prim_path, position, orientation, size=0.05):
    """Create a visual target cube for the robot to follow."""
    target_cube = VisualCuboid(
        name=name,
        prim_path=prim_path,
        position=position,
        orientation=orientation,
        size=size,
        color=np.array([1, 0, 0]) #RGB
    )
    world.scene.add(target_cube)
    return target_cube

# ================================
# MOTION CONTROL SETUP
# ================================
def setup_motion_controller(robot, config_paths):
    """Initialize RmpFlow motion planning and controller."""
    robot_desc_path, rmpflow_config_path, urdf_path = config_paths
    
    # Create RmpFlow motion policy
    rmpflow = mg.lula.motion_policies.RmpFlow(
        robot_description_path=robot_desc_path,
        rmpflow_config_path=rmpflow_config_path,
        urdf_path=urdf_path,
        end_effector_frame_name="Link6",
        maximum_substep_size=0.00334,  # Control frequency parameter
    )
    
    # Create articulation motion policy
    articulation_rmp = mg.ArticulationMotionPolicy(robot, rmpflow, 1.0 / 60.0)
    
    # Create motion controller
    controller = mg.MotionPolicyController(
        name="realman_controller",
        articulation_motion_policy=articulation_rmp,
    )
    
    return rmpflow, controller

def configure_motion_constraints(robot, rmpflow, controller, world):
    """Configure robot base pose and collision constraints."""
    # Set robot base pose for motion planning
    default_position, default_orientation = robot.get_world_pose()
    rmpflow.set_robot_base_pose(
        robot_position=default_position, 
        robot_orientation=default_orientation
    )
    
    # Add ground plane as obstacle for collision avoidance
    ground_plane = world.scene.get_object(name="default_ground_plane")
    controller.add_obstacle(ground_plane)

# ================================
# MAIN SIMULATION EXECUTION
# ================================
def main():
    """Main simulation execution function."""
    
    # World and scene setup
    my_world = setup_world()
    
    # Robot asset configuration
    asset_path = "/isaac-sim/my_examples/realman_rm65_b/assets/rm_65_b/robot.usd"
    robot_prim_path = "/World/rm_65"
    
    # Load robot into scene
    stage = load_robot_asset(my_world, asset_path, robot_prim_path)
    
    # Configure robot position and orientation
    robot_position = (0, 0.05, 0.4)
    robot_rotation = (0.0, -45.0, 0.0)
    configure_robot_transform(stage, robot_prim_path, robot_position, robot_rotation)
    
    # Setup manipulator with default joint positions
    joints_default_positions = np.array([0, np.pi/4, 0, 0, 0, 0])
    manipulator = setup_manipulator(
        my_world, robot_prim_path, "rm_65_robot", joints_default_positions
    )
    
    # Create target cube for robot to follow
    target_position = np.array([-0.4, 0.05, 0.66])
    target_orientation = euler_angles_to_quat(np.array([-np.pi, 0, 0]))
    target_name = "target_cube"
    target_prim_path = "/World/target_cube"
    
    target_cube = create_target_cube(
        my_world, target_name, target_prim_path, target_position, target_orientation
    )
    
    # Get robot reference and reset world
    my_realman = my_world.scene.get_object("rm_65_robot")
    print(f"Robot initialized: {my_realman}")
    my_world.reset()
    
    # Motion controller setup
    config_paths = (
        "/isaac-sim/my_examples/realman_rm65_b/rmpflow/rm65_robot_descriptor.yaml",
        "/isaac-sim/my_examples/realman_rm65_b/rmpflow/rm65_rmpflow_common.yaml",
        "/isaac-sim/my_examples/realman_rm65_b/assets/rm_65_b/robot.urdf"
    )
    
    rmpflow, my_controller = setup_motion_controller(my_realman, config_paths)
    configure_motion_constraints(my_realman, rmpflow, my_controller, my_world)
    
    # Get articulation controller for applying actions
    articulation_controller = my_realman.get_articulation_controller()
    
    # ================================
    # SIMULATION LOOP
    # ================================
    reset_needed = False
    
    while simulation_app.is_running():
        # Step the simulation forward
        my_world.step(render=True)
        
        # Handle simulation state changes
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
            
        if my_world.is_playing():
            # Reset world if needed
            if reset_needed:
                my_world.reset()
                reset_needed = False
            
            # Execute motion control
            target_obj = my_world.scene.get_object(target_name)
            if target_obj is not None:
                # Get current target pose
                target_pos, target_orient = target_obj.get_world_pose()
                
                # Generate motion actions using RmpFlow
                actions = my_controller.forward(
                    target_end_effector_position=target_pos,
                    target_end_effector_orientation=target_orient,
                )
                
                # Apply actions to robot
                articulation_controller.apply_action(actions)
            else:
                print(f"Warning: Target object '{target_name}' not found in scene.")
        
        # Exit if in test mode
        if args.test:
            break
    
    # Clean shutdown
    simulation_app.close()

# ================================
# SCRIPT ENTRY POINT
# ================================
if __name__ == "__main__":
    main()
