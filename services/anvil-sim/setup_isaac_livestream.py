#!/usr/bin/env python3
"""
Isaac Sim Livestream Setup Script
Properly configures Isaac Sim with livestream and a simulation scene
"""

import sys
import os
import asyncio
import logging

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
    from isaacsim import SimulationApp
    from isaacsim.core import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.assets import IsaacSimAssets
    from isaacsim.manipulators import Franka
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.prims import create_prim
    from isaacsim.core.utils.nucleus import get_assets_root_path
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import UsdGeom, Gf, Sdf
    logger.info("✅ Isaac Sim modules imported successfully")
except ImportError as e:
    logger.error(f"❌ Isaac Sim import failed: {e}")
    sys.exit(1)

async def setup_isaac_sim_livestream():
    """Setup Isaac Sim with proper livestream configuration"""
    logger.info("🚀 Setting up Isaac Sim with livestream...")
    
    # Start Isaac Sim with livestream configuration
    config = {
        "width": 1920,
        "height": 1080,
        "window_width": 1920,
        "window_height": 1080,
        "headless": False,  # Enable GUI for livestream
        "hide_ui": False,   # Show UI
        "renderer": "RaytracedLighting",
        "display_options": 3286,  # Show default grid
    }
    
    kit = SimulationApp(launch_config=config)
    
    # Enable livestream extensions
    enable_extension("omni.services.livestream.nvcf")
    enable_extension("omni.kit.livestream.webrtc")
    logger.info("✅ Livestream extensions enabled")
    
    # Configure livestream settings
    kit.set_setting("/app/livestream/enabled", True)
    kit.set_setting("/app/livestream/port", 49100)
    kit.set_setting("/app/livestream/publicEndpointAddress", "216.81.248.163")
    kit.set_setting("/app/livestream/allowResize", True)
    kit.set_setting("/app/window/drawMouse", True)
    kit.set_setting("/app/livestream/logLevel", "info")
    
    # Configure WebRTC settings
    kit.set_setting("/app/livestream/proto", "websocket")
    kit.set_setting("/app/livestream/ipversion", "ipv4")
    
    logger.info("✅ Livestream configuration applied")
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    logger.info("✅ World created")
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    logger.info("✅ Ground plane added")
    
    # Create a more interesting scene
    stage = get_current_stage()
    
    # Add multiple objects for visual interest
    objects = [
        {"name": "/World/Cube1", "pos": (0, 0, 0.25), "size": 0.5},
        {"name": "/World/Cube2", "pos": (1, 0, 0.25), "size": 0.3},
        {"name": "/World/Cube3", "pos": (-1, 0, 0.25), "size": 0.4},
        {"name": "/World/Cylinder", "pos": (0, 1, 0.5), "size": 0.3}
    ]
    
    for obj in objects:
        if "Cylinder" in obj["name"]:
            prim = create_prim(obj["name"], "Cylinder")
            prim.GetAttribute("radius").Set(obj["size"])
            prim.GetAttribute("height").Set(obj["size"] * 2)
        else:
            prim = create_prim(obj["name"], "Cube")
            prim.GetAttribute("size").Set(obj["size"])
        
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(*obj["pos"]))
    
    logger.info("✅ Scene objects created")
    
    # Add lighting
    light_prim = create_prim("/World/DirectionalLight", "DistantLight")
    light_prim.GetAttribute("intensity").Set(1000)
    light_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(45, 45, 0))
    
    # Add camera with better positioning
    camera_prim = create_prim("/World/Camera", "Camera")
    camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, -3, 2))
    camera_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(15, 0, 0))
    
    logger.info("✅ Lighting and camera configured")
    
    # Try to load a robot if assets are available
    try:
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            logger.info(f"✅ Assets root found: {assets_root_path}")
            
            # Try to load Franka robot
            franka_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            if os.path.exists(franka_path):
                add_reference_to_stage(usd_path=franka_path, prim_path="/World/Franka")
                franka_prim = get_current_stage().GetPrimAtPath("/World/Franka")
                franka_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, 0))
                logger.info("✅ Franka robot loaded")
            else:
                logger.warning("⚠️ Franka robot USD not found")
        else:
            logger.warning("⚠️ Assets root path not available")
    except Exception as e:
        logger.warning(f"⚠️ Could not load robot assets: {e}")
    
    # Reset world
    await world.reset_async()
    logger.info("✅ World reset complete")
    
    # Start simulation loop
    logger.info("🎬 Starting Isaac Sim livestream simulation...")
    logger.info("📡 Livestream should be available at: http://216.81.248.163:49100")
    
    frame_count = 0
    
    while kit._app.is_running() and not kit.is_exiting():
        # Step the simulation
        await world.step_async()
        
        # Update the app
        kit.update()
        
        frame_count += 1
        if frame_count % 300 == 0:  # Log every 5 seconds at 60 FPS
            logger.info(f"📹 Isaac Sim running: {frame_count} frames")
            logger.info(f"🔗 Livestream status: Enabled on port 49100")
    
    logger.info("🛑 Shutting down Isaac Sim...")
    kit.close()
    logger.info("✅ Isaac Sim closed")

if __name__ == "__main__":
    asyncio.run(setup_isaac_sim_livestream())
