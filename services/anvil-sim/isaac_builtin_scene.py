#!/usr/bin/env python3
"""
Isaac Sim Built-in Scene Creation
Uses Isaac Sim's built-in scene creation capabilities
"""

import sys
import os
import logging

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
    
    logger.info("🚀 Starting Isaac Sim with built-in scene creation...")
    
    # Simple configuration
    config = {
        "headless": True,
        "width": 1920,
        "height": 1080
    }
    
    kit = SimulationApp(config)
    
    # Import Isaac Sim modules
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
    
    logger.info("✅ Isaac Sim modules imported")
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    logger.info("🌍 World created")
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    logger.info("🏔️ Ground plane added")
    
    # Add a simple cube
    cube = DynamicCuboid(
        prim_path="/World/TestCube",
        name="test_cube",
        position=(0, 0, 1.0),
        size=(0.2, 0.2, 0.2),
        color=(1.0, 0.0, 0.0)
    )
    world.scene.add(cube)
    logger.info("🔴 Test cube added")
    
    # Add a simple sphere
    sphere = DynamicSphere(
        prim_path="/World/TestSphere",
        name="test_sphere",
        position=(0.5, 0, 1.0),
        radius=0.1,
        color=(0.0, 0.0, 1.0)
    )
    world.scene.add(sphere)
    logger.info("🔵 Test sphere added")
    
    logger.info("🎯 Scene setup complete")
    
    # Reset world
    world.reset()
    logger.info("🔄 World reset")
    
    # Run simulation
    logger.info("🎮 Starting simulation...")
    
    for i in range(100):
        world.step(render=True)
        if i % 10 == 0:
            logger.info(f"🎬 Step {i}")
    
    logger.info("✅ Simulation completed")
    
    # Keep running
    logger.info("🔄 Keeping simulation running...")
    while True:
        world.step(render=True)
    
except ImportError as e:
    logger.error(f"❌ Import failed: {e}")
    sys.exit(1)
except Exception as e:
    logger.error(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
