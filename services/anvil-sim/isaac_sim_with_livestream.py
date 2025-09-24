#!/usr/bin/env python3
"""
Isaac Sim with Livestream
Loads a simulation scene and enables livestream for remote viewing
"""

import sys
import os
import asyncio
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Isaac Sim setup - Add Isaac Sim paths to Python path
isaac_sim_base = "/isaac-sim"
sys.path.insert(0, isaac_sim_base)
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "python"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "exts"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "extscore"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "kernel", "py"))
sys.path.insert(0, os.path.join(isaac_sim_base, "exts"))

try:
    from isaacsim import SimulationApp
    from isaacsim.core import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.assets import IsaacSimAssets
    from isaacsim.manipulators import Franka
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.prims import create_prim
    from isaacsim.core.utils.nucleus import get_assets_root_path
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import UsdGeom, Gf
    logger.info("✅ Isaac Sim modules imported successfully")
except ImportError as e:
    logger.error(f"❌ Isaac Sim import failed: {e}")
    sys.exit(1)

async def main():
    """Main function to run Isaac Sim with livestream"""
    logger.info("🚀 Starting Isaac Sim with livestream...")
    
    # Start Isaac Sim in headless mode with livestream
    config = {
        "width": 1920,
        "height": 1080,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "hide_ui": False,  # Show the GUI for livestream
        "renderer": "RaytracedLighting",
        "display_options": 3286,  # Set display options to show default grid
    }
    
    kit = SimulationApp(launch_config=config)
    
    # Enable livestream extension
    enable_extension("omni.services.livestream.nvcf")
    logger.info("✅ Livestream extension enabled")
    
    # Set livestream settings
    kit.set_setting("/app/window/drawMouse", True)
    kit.set_setting("/app/livestream/port", 49100)
    kit.set_setting("/app/livestream/publicEndpointAddress", "216.81.248.163")
    logger.info("✅ Livestream settings configured")
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    logger.info("✅ World created")
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    logger.info("✅ Ground plane added")
    
    # Add a robot (Franka Panda)
    try:
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            logger.warning("⚠️ Assets root path not found, creating simple scene")
            # Create a simple cube as fallback
            stage = get_current_stage()
            cube_prim = create_prim("/World/Cube", "Cube")
            cube_prim.GetAttribute("size").Set(0.5)
            cube_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, 0.25))
        else:
            logger.info("✅ Assets root path found, loading Franka robot")
            # Load Franka robot
            franka_prim_path = "/World/Franka"
            add_reference_to_stage(usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd", prim_path=franka_prim_path)
            
            # Position the robot
            franka_prim = get_current_stage().GetPrimAtPath(franka_prim_path)
            franka_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, 0))
            
    except Exception as e:
        logger.warning(f"⚠️ Could not load robot: {e}")
        # Create a simple scene with a cube
        stage = get_current_stage()
        cube_prim = create_prim("/World/Cube", "Cube")
        cube_prim.GetAttribute("size").Set(0.5)
        cube_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, 0.25))
    
    # Add some lighting
    stage = get_current_stage()
    light_prim = create_prim("/World/Light", "DistantLight")
    light_prim.GetAttribute("intensity").Set(1000)
    light_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(45, 45, 0))
    
    logger.info("✅ Scene setup complete")
    
    # Reset world
    await world.reset_async()
    logger.info("✅ World reset")
    
    # Start simulation loop
    logger.info("🎬 Starting simulation loop...")
    frame_count = 0
    
    while kit._app.is_running() and not kit.is_exiting():
        # Step the simulation
        await world.step_async()
        
        # Update the app
        kit.update()
        
        frame_count += 1
        if frame_count % 300 == 0:  # Log every 5 seconds at 60 FPS
            logger.info(f"📹 Simulation running: {frame_count} frames")
    
    logger.info("🛑 Shutting down Isaac Sim...")
    kit.close()
    logger.info("✅ Isaac Sim closed")

if __name__ == "__main__":
    asyncio.run(main())
