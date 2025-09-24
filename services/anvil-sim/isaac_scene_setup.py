#!/usr/bin/env python3
"""
Isaac Sim Scene Setup Script
Creates a simple scene with visible objects for testing
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
    
    logger.info("🚀 Starting Isaac Sim scene setup...")
    
    # Simple configuration
    config = {
        "headless": True,
        "width": 1920,
        "height": 1080
    }
    
    kit = SimulationApp(config)
    
    # Import Isaac Sim modules
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder
    
    logger.info("✅ Isaac Sim modules imported")
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    logger.info("🌍 World created")
    
    # Add ground plane
    world.scene.add_default_ground_plane()
    logger.info("🏔️ Ground plane added")
    
    # Add visible objects
    objects = []
    
    # Red cube
    cube = DynamicCuboid(
        prim_path="/World/RedCube",
        name="red_cube",
        position=(0, 0, 1.0),
        size=(0.2, 0.2, 0.2),
        color=(1.0, 0.0, 0.0)
    )
    world.scene.add(cube)
    objects.append(cube)
    logger.info("🔴 Red cube added")
    
    # Blue sphere
    sphere = DynamicSphere(
        prim_path="/World/BlueSphere",
        name="blue_sphere",
        position=(0.5, 0, 1.0),
        radius=0.1,
        color=(0.0, 0.0, 1.0)
    )
    world.scene.add(sphere)
    objects.append(sphere)
    logger.info("🔵 Blue sphere added")
    
    # Green cylinder
    cylinder = DynamicCylinder(
        prim_path="/World/GreenCylinder",
        name="green_cylinder",
        position=(-0.5, 0, 1.0),
        radius=0.1,
        height=0.2,
        color=(0.0, 1.0, 0.0)
    )
    world.scene.add(cylinder)
    objects.append(cylinder)
    logger.info("🟢 Green cylinder added")
    
    # Add more objects for better visualization
    for i in range(2):
        for j in range(2):
            if i == 0 and j == 0:  # Skip center position
                continue
            cube = DynamicCuboid(
                prim_path=f"/World/Cube_{i}_{j}",
                name=f"cube_{i}_{j}",
                position=(i * 0.4 - 0.2, j * 0.4 - 0.2, 1.5),
                size=(0.15, 0.15, 0.15),
                color=(0.5 + i * 0.3, 0.5 + j * 0.3, 0.8)
            )
            world.scene.add(cube)
            objects.append(cube)
    
    logger.info(f"🎯 Created {len(objects)} objects in simulation")
    
    # Reset world
    world.reset()
    logger.info("🔄 World reset")
    
    # Run simulation with animation
    logger.info("🎮 Starting animated simulation...")
    
    for i in range(1000):
        world.step(render=True)
        
        if i % 100 == 0:
            logger.info(f"🎬 Simulation step {i}")
            
            # Animate objects
            for idx, obj in enumerate(objects):
                if hasattr(obj, 'set_world_pose'):
                    current_pos = obj.get_world_pose()[0]
                    # Simple bouncing animation
                    bounce = 0.1 * (1 if (i // 50) % 2 == 0 else -1)
                    new_pos = [
                        current_pos[0],
                        current_pos[1],
                        current_pos[2] + bounce
                    ]
                    obj.set_world_pose(position=new_pos)
    
    logger.info("✅ Scene setup and animation completed")
    
    # Keep running indefinitely
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
