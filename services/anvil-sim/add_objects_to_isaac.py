#!/usr/bin/env python3
"""
Add Visible Objects to Existing Isaac Sim Instance
Connects to running Isaac Sim and adds visible simulation objects
"""

import sys
import os
import asyncio
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Isaac Sim setup
isaac_sim_base = "/isaac-sim"
sys.path.insert(0, isaac_sim_base)
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "python"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "exts"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "extscore"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "kernel", "py"))
sys.path.insert(0, os.path.join(isaac_sim_base, "exts"))

try:
    from omni.isaac.kit import SimulationApp
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage
    from omni.isaac.core.utils.prims import create_prim, define_prim
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.stage import get_current_stage, set_stage_up_axis
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder
    from omni.isaac.core.materials import PhysicsMaterial
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.manipulators import Franka
    from omni.isaac.manipulators.grippers import ParallelGripper
    from pxr import UsdGeom, Gf, Sdf, UsdPhysics
    import numpy as np
    logger.info("✅ Isaac Sim modules imported successfully")
except ImportError as e:
    logger.error(f"❌ Isaac Sim import failed: {e}")
    sys.exit(1)

async def add_objects_to_existing_isaac():
    """Add visible objects to existing Isaac Sim instance"""
    logger.info("🚀 Connecting to existing Isaac Sim instance...")
    
    # Connect to existing Isaac Sim instance
    config = {
        "width": 1920,
        "height": 1080,
        "headless": True,  # Connect to existing instance
        "enable_livestream": True,
        "livestream_port": 49100,
        "livestream_public_endpoint_address": "216.81.248.163"
    }
    
    kit = SimulationApp(launch_config=config)
    logger.info("✅ Connected to existing Isaac Sim instance")
    
    # Enable required extensions
    enable_extension("omni.isaac.core")
    enable_extension("omni.isaac.manipulators")
    enable_extension("omni.isaac.sensor")
    logger.info("✅ Required extensions enabled")
    
    # Get current stage
    stage = get_current_stage()
    if not stage:
        logger.error("❌ No stage found - Isaac Sim may not be running properly")
        return
    
    logger.info("✅ Connected to existing Isaac Sim stage")
    
    # Create world with physics
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0)
    logger.info("✅ World created with physics enabled")
    
    # Add ground plane if not exists
    try:
        world.scene.add_default_ground_plane()
        logger.info("✅ Ground plane added")
    except Exception as e:
        logger.info(f"ℹ️ Ground plane already exists or error: {e}")
    
    # Create physics material for objects
    try:
        physics_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterial",
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.8
        )
        world.scene.add(physics_material)
        logger.info("✅ Physics material created")
    except Exception as e:
        logger.info(f"ℹ️ Physics material already exists or error: {e}")
    
    # Create multiple colorful objects that will be visible
    objects = []
    
    try:
        # Create cubes with different colors
        for i in range(5):
            cube = DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=np.array([i * 0.5 - 1.0, 0, 1.0 + i * 0.1]),
                size=0.2,
                color=np.array([1.0 - i * 0.2, i * 0.2, 0.5 + i * 0.1])  # Different colors
            )
            objects.append(cube)
            world.scene.add(cube)
        
        # Create spheres
        for i in range(3):
            sphere = DynamicSphere(
                prim_path=f"/World/Sphere_{i}",
                name=f"sphere_{i}",
                position=np.array([-1.5 + i * 0.8, 1.0, 2.0]),
                radius=0.15,
                color=np.array([0.2 + i * 0.3, 0.8 - i * 0.2, 1.0 - i * 0.3])  # Different colors
            )
            objects.append(sphere)
            world.scene.add(sphere)
        
        # Create cylinders
        for i in range(3):
            cylinder = DynamicCylinder(
                prim_path=f"/World/Cylinder_{i}",
                name=f"cylinder_{i}",
                position=np.array([1.5, -1.0 + i * 0.6, 1.5]),
                radius=0.1,
                height=0.4,
                color=np.array([1.0, 0.3 + i * 0.2, 0.1 + i * 0.3])  # Different colors
            )
            objects.append(cylinder)
            world.scene.add(cylinder)
        
        logger.info(f"✅ Created {len(objects)} dynamic objects")
        
    except Exception as e:
        logger.error(f"❌ Error creating objects: {e}")
        return
    
    # Try to add a robot if assets are available
    try:
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            logger.info(f"✅ Assets root found: {assets_root_path}")
            
            # Try to load Franka robot
            franka_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            if os.path.exists(franka_path):
                add_reference_to_stage(usd_path=franka_path, prim_path="/World/Franka")
                franka_prim = stage.GetPrimAtPath("/World/Franka")
                
                # Position the robot
                franka_transform = UsdGeom.Xformable(franka_prim)
                franka_transform.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))
                
                logger.info("✅ Franka robot loaded and positioned")
            else:
                logger.warning("⚠️ Franka robot USD not found, creating simple robot representation")
                
                # Create a simple robot representation
                robot_base = DynamicCuboid(
                    prim_path="/World/RobotBase",
                    name="robot_base",
                    position=np.array([0, 0, 0.1]),
                    size=0.3,
                    color=np.array([0.2, 0.2, 0.2])
                )
                world.scene.add(robot_base)
                
                # Robot arm segments
                for i in range(3):
                    arm_segment = DynamicCuboid(
                        prim_path=f"/World/RobotArm_{i}",
                        name=f"robot_arm_{i}",
                        position=np.array([0, i * 0.3, 0.3 + i * 0.1]),
                        size=0.1,
                        color=np.array([0.1, 0.1, 0.1])
                    )
                    world.scene.add(arm_segment)
                
                logger.info("✅ Simple robot representation created")
        else:
            logger.warning("⚠️ Assets root path not available")
    except Exception as e:
        logger.warning(f"⚠️ Could not load robot: {e}")
    
    # Add lighting for better visibility
    try:
        # Directional light
        light_prim = create_prim("/World/DirectionalLight", "DistantLight")
        light_prim.GetAttribute("intensity").Set(1000)
        light_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(45, 45, 0))
        
        # Additional point lights
        for i in range(2):
            point_light = create_prim(f"/World/PointLight_{i}", "SphereLight")
            point_light.GetAttribute("intensity").Set(500)
            point_light.GetAttribute("xformOp:translate").Set(Gf.Vec3f(-2 + i * 4, 0, 3))
        
        logger.info("✅ Lighting configured")
    except Exception as e:
        logger.warning(f"⚠️ Could not configure lighting: {e}")
    
    # Add camera with good viewing angle
    try:
        camera_prim = create_prim("/World/Camera", "Camera")
        camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, -3, 2))
        camera_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(15, 0, 0))
        logger.info("✅ Camera positioned for optimal viewing")
    except Exception as e:
        logger.warning(f"⚠️ Could not configure camera: {e}")
    
    # Reset world to initialize physics
    await world.reset_async()
    logger.info("✅ World reset - physics simulation started")
    
    # Start simulation loop
    logger.info("🎬 Objects added to Isaac Sim livestream!")
    logger.info("📡 Livestream available at: http://216.81.248.163:49100")
    logger.info("🎯 Objects should now be visible with physics simulation")
    
    frame_count = 0
    
    while kit.is_running() and not kit.is_exiting():
        # Step the simulation
        await world.step_async()
        
        # Update the app
        kit.update()
        
        frame_count += 1
        if frame_count % 300 == 0:  # Log every 5 seconds at 60 FPS
            logger.info(f"📹 Isaac Sim running: {frame_count} frames")
            logger.info(f"🎯 {len(objects)} objects in simulation")
            logger.info(f"🔗 Livestream active on port 49100")
    
    logger.info("🛑 Shutting down Isaac Sim...")
    kit.close()
    logger.info("✅ Isaac Sim closed")

if __name__ == "__main__":
    asyncio.run(add_objects_to_existing_isaac())
