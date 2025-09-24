#!/usr/bin/env python3
"""
Isaac Sim Livestream Configuration Script
Properly configures Isaac Sim for livestreaming with scene setup
"""

import sys
import os
import logging
import asyncio

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Isaac Sim paths
isaac_sim_base = "/isaac-sim"

# Add Isaac Sim paths
sys.path.insert(0, isaac_sim_base)
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "exts"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "extscore"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "kernel"))
sys.path.insert(0, os.path.join(isaac_sim_base, "exts"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "python"))

try:
    from omni.isaac.kit import SimulationApp
    
    # Configure Isaac Sim for livestreaming
    config = {
        "headless": True,
        "width": 1920,
        "height": 1080,
        "window_width": 1920,
        "window_height": 1080,
        "renderer": "RayTracedLighting",
        "sync_loads": True,
        "enable_cameras": True,
        "enable_livestream": True,
        "livestream_port": 49100,
        "livestream_public_endpoint": "216.81.248.163"
    }
    
    logger.info("🚀 Starting Isaac Sim with livestreaming configuration...")
    kit = SimulationApp(config)
    
    # Import Isaac Sim modules
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder
    from omni.isaac.core.materials import PhysicsMaterial
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
    from omni.isaac.core.utils.extensions import enable_extension
    
    # Enable livestreaming extension
    enable_extension("omni.kit.livestream.core")
    enable_extension("omni.kit.livestream.webrtc")
    enable_extension("omni.services.livestream.nvcf")
    
    logger.info("✅ Isaac Sim modules imported successfully")
    
    # Create world with physics
    world = World(stage_units_in_meters=1.0)
    logger.info("🌍 World created with physics enabled")
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    logger.info("🏔️ Ground plane added")
    
    # Create physics material
    physics_material = PhysicsMaterial(
        prim_path="/World/PhysicsMaterial",
        static_friction=0.5,
        dynamic_friction=0.5,
        restitution=0.1
    )
    logger.info("⚗️ Physics material created")
    
    # Add visible objects to the scene
    objects = []
    
    # Add a red cube
    red_cube = DynamicCuboid(
        prim_path="/World/RedCube",
        name="red_cube",
        position=(0, 0, 1.0),
        size=(0.2, 0.2, 0.2),
        color=(1.0, 0.0, 0.0)
    )
    world.scene.add(red_cube)
    objects.append(red_cube)
    logger.info("🔴 Red cube added")
    
    # Add a blue sphere
    blue_sphere = DynamicSphere(
        prim_path="/World/BlueSphere",
        name="blue_sphere",
        position=(0.5, 0, 1.0),
        radius=0.1,
        color=(0.0, 0.0, 1.0)
    )
    world.scene.add(blue_sphere)
    objects.append(blue_sphere)
    logger.info("🔵 Blue sphere added")
    
    # Add a green cylinder
    green_cylinder = DynamicCylinder(
        prim_path="/World/GreenCylinder",
        name="green_cylinder",
        position=(-0.5, 0, 1.0),
        radius=0.1,
        height=0.2,
        color=(0.0, 1.0, 0.0)
    )
    world.scene.add(green_cylinder)
    objects.append(green_cylinder)
    logger.info("🟢 Green cylinder added")
    
    # Add more objects for better visualization
    for i in range(3):
        for j in range(3):
            if i == 1 and j == 1:  # Skip center position
                continue
            cube = DynamicCuboid(
                prim_path=f"/World/Cube_{i}_{j}",
                name=f"cube_{i}_{j}",
                position=(i * 0.3 - 0.3, j * 0.3 - 0.3, 1.5),
                size=(0.15, 0.15, 0.15),
                color=(0.5 + i * 0.2, 0.5 + j * 0.2, 0.8)
            )
            world.scene.add(cube)
            objects.append(cube)
    
    logger.info(f"🎯 Created {len(objects)} objects in simulation")
    
    # Set up camera for better viewing
    from omni.isaac.core.utils.render_product import create_hydra_texture
    from omni.isaac.core.utils.camera import update_camera
    
    # Create a camera
    camera_prim_path = "/World/Camera"
    camera_prim = create_prim(
        prim_path=camera_prim_path,
        prim_type="Camera",
        position=(0, -3, 2),
        orientation=(0.8, 0, 0, 0.6)
    )
    
    # Create render product for the camera
    render_product_path = create_hydra_texture(
        prim_path=camera_prim_path,
        texture_width=1920,
        texture_height=1080
    )
    
    logger.info("📷 Camera configured for livestreaming")
    
    # Configure livestreaming
    from omni.kit.livestream.core import acquire_livestream_interface
    
    livestream_interface = acquire_livestream_interface()
    if livestream_interface:
        logger.info("📡 Acquired livestream interface")
        
        # Start livestreaming
        livestream_interface.start_streaming(
            viewport_name="Viewport",
            resolution=(1920, 1080),
            framerate=30
        )
        
        logger.info("🎬 Livestreaming started")
        
        # Get livestream URL
        livestream_url = livestream_interface.get_url()
        logger.info(f"🌐 Livestream URL: {livestream_url}")
    else:
        logger.error("❌ Could not acquire livestream interface")
    
    # Run simulation
    logger.info("🎮 Starting simulation loop...")
    
    # Reset the world
    world.reset()
    
    # Run simulation for a while
    for i in range(1000):
        world.step(render=True)
        
        if i % 100 == 0:
            logger.info(f"🎬 Simulation step {i}")
            
            # Move objects slightly for animation
            for idx, obj in enumerate(objects):
                if hasattr(obj, 'set_world_pose'):
                    current_pos = obj.get_world_pose()[0]
                    new_pos = [
                        current_pos[0],
                        current_pos[1],
                        current_pos[2] + 0.01 * (1 if i % 200 < 100 else -1)
                    ]
                    obj.set_world_pose(position=new_pos)
    
    logger.info("✅ Simulation completed successfully")
    
    # Keep the simulation running
    logger.info("🔄 Keeping simulation running...")
    
    # Keep running indefinitely
    while True:
        world.step(render=True)
        await asyncio.sleep(0.033)  # ~30 FPS
    
except ImportError as e:
    logger.error(f"❌ Isaac Sim import failed: {e}")
    logger.error("This script must be run inside the Isaac Sim environment")
    sys.exit(1)
except Exception as e:
    logger.error(f"❌ Error running Isaac Sim: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
finally:
    if 'kit' in locals():
        kit.close()
        logger.info("🔚 Isaac Sim closed")
