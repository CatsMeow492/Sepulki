#!/usr/bin/env python3
"""
Real Isaac Sim Renderer - Uses actual Isaac Sim
Provides photorealistic 3D robot simulation with advanced lighting and materials.
"""

import asyncio
import logging
import time
from typing import Dict, Any, Optional, Tuple
import numpy as np
import sys
import os

# Add Isaac Sim to Python path - use the correct host installation path
isaac_sim_base = "/home/shadeform/isaac-sim/isaac-sim-2023.1.1"
sys.path.insert(0, isaac_sim_base)

# Also add the Isaac Sim extensions and modules from the correct location
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "exts"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "extscore"))
sys.path.insert(0, os.path.join(isaac_sim_base, "kit", "kernel"))
sys.path.insert(0, os.path.join(isaac_sim_base, "exts"))

# Isaac Sim imports
try:
    from omni.isaac.kit import SimulationApp
    from omni.isaac.core import World, SimulationContext
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.core.utils.stage import create_new_stage
    from omni.isaac.sensor import Camera
    try:
        from omni.isaac.core.utils.prims import create_prim
        CREATE_PRIM_AVAILABLE = True
    except ImportError:
        CREATE_PRIM_AVAILABLE = False
        print("⚠️  create_prim not available, scene setup will be limited")
    from omni.isaac.core.materials import PhysicsMaterial
    ISAAC_SIM_AVAILABLE = True
    print("✅ Isaac Sim modules loaded successfully")
except ImportError as e:
    ISAAC_SIM_AVAILABLE = False
    CREATE_PRIM_AVAILABLE = False
    print(f"⚠️  Isaac Sim modules not available: {e}")
    print("   This is expected if Isaac Sim is not installed or not in PATH")

import structlog

logger = structlog.get_logger(__name__)

class IsaacSimRealRenderer:
    """Real Isaac Sim renderer with photorealistic 3D simulation."""
    
    def __init__(self, width: int = 1920, height: int = 1080):
        self.width = width
        self.height = height
        self.frame_count = 0
        
        # Isaac Sim components
        self.app = None
        self.world = None
        self.camera = None
        self.robot = None
        self.scene_initialized = False
        
        # Current state
        self.camera_state = {
            'position': [4.0, 4.0, 4.0],
            'target': [0.0, 0.0, 0.0],
            'fov': 50.0
        }
        
        self.joint_states = {
            'joint1': 0.0,
            'joint2': 0.0
        }
        
        self.robot_config = {
            'name': 'Default Robot',
            'isaac_sim_path': None,
            'specifications': {}
        }
        
        # Try to initialize Isaac Sim
        try:
            self._initialize_isaac_sim()
        except Exception as e:
            logger.warning(f"Isaac Sim initialization failed, falling back to mock rendering: {e}")
            self.scene_initialized = False
    
    def _initialize_isaac_sim(self):
        """Initialize Isaac Sim application and world."""
        try:
            # Configure Isaac Sim for headless rendering
            config = {
                "headless": True,
                "width": self.width,
                "height": self.height,
                "renderer": "RayTracedLighting",  # Photorealistic rendering
            }
            
            self.app = SimulationApp(config)
            logger.info("🎬 Real Isaac Sim application initialized", config=config)
            
            # Enable required extensions
            enable_extension("omni.isaac.core")
            enable_extension("omni.isaac.sensor")
            
            # Create new stage
            create_new_stage()
            
            # Initialize world
            self.world = World()
            logger.info("🌍 Real Isaac Sim world created")
            
            # Setup scene
            self._setup_scene()
            
            self.scene_initialized = True
            logger.info("✅ Real Isaac Sim scene initialized successfully")
            
        except Exception as e:
            logger.error("❌ Failed to initialize real Isaac Sim", error=str(e))
            self.scene_initialized = False
    
    def _setup_scene(self):
        """Setup the Isaac Sim scene with lighting, environment, and materials."""
        try:
            # Setup environment lighting
            self._setup_lighting()
            
            # Setup ground plane
            self._setup_ground()
            
            logger.info("🏗️ Real Isaac Sim scene setup completed")
            
        except Exception as e:
            logger.error("❌ Failed to setup Isaac Sim scene", error=str(e))
    
    def _setup_lighting(self):
        """Setup advanced lighting for photorealistic rendering."""
        if not CREATE_PRIM_AVAILABLE:
            logger.warning("⚠️ Skipping lighting setup - create_prim not available")
            return

        try:
            # Create dome light for environment lighting
            dome_light = create_prim(
                "/World/DomeLight",
                "DomeLight",
                position=[0, 0, 0],
                orientation=[0, 0, 0, 1]
            )

            # Configure dome light
            dome_light.GetAttribute("intensity").Set(1.0)
            dome_light.GetAttribute("color").Set((1.0, 1.0, 1.0))

            logger.info("💡 Real Isaac Sim lighting setup completed")

        except Exception as e:
            logger.error("❌ Failed to setup lighting", error=str(e))
    
    def _setup_ground(self):
        """Setup ground plane with realistic materials."""
        if not CREATE_PRIM_AVAILABLE:
            logger.warning("⚠️ Skipping ground setup - create_prim not available")
            return

        try:
            # Create ground plane
            ground = create_prim(
                "/World/Ground",
                "Xform",
                position=[0, 0, 0],
                scale=[10, 10, 1]
            )

            # Add ground mesh
            ground_mesh = create_prim(
                "/World/Ground/Mesh",
                "Plane",
                position=[0, 0, 0],
                scale=[10, 10, 1]
            )

            logger.info("🏞️ Real Isaac Sim ground plane setup completed")

        except Exception as e:
            logger.error("❌ Failed to setup ground", error=str(e))
    
    async def load_robot(self, robot_config: Dict[str, Any]):
        """Load actual Isaac Sim robot model."""
        if not self.scene_initialized:
            logger.warning("Isaac Sim scene not initialized - cannot load robot")
            return False
        
        try:
            robot_name = robot_config.get('name', 'Default Robot')
            robot_path = robot_config.get('isaac_sim_path')
            
            if not robot_path:
                logger.warning("No Isaac Sim path provided for robot", robot_name=robot_name)
                return False
            
            # Remove existing robot if any
            if self.robot:
                self.world.scene.remove_object(self.robot)
            
            # Load robot from Isaac Sim assets
            self.robot = self.world.scene.add(
                Robot(
                    prim_path=f"/World/{robot_name}",
                    name=robot_name,
                    usd_path=robot_path
                )
            )
            
            self.robot_config.update(robot_config)
            logger.info("🤖 Real robot loaded successfully", 
                       robot_name=robot_name, 
                       robot_path=robot_path)
            
            return True
            
        except Exception as e:
            logger.error("❌ Failed to load real robot", 
                        robot_name=robot_config.get('name'),
                        error=str(e))
            return False
    
    async def setup_camera(self, position: list, target: list, fov: float):
        """Setup Isaac Sim camera with advanced rendering."""
        if not self.scene_initialized:
            logger.warning("Isaac Sim scene not initialized - cannot setup camera")
            return False
        
        try:
            # Remove existing camera if any
            if self.camera:
                self.world.scene.remove_object(self.camera)
            
            # Create camera
            self.camera = Camera(
                prim_path="/World/Camera",
                position=position,
                look_at=target,
                fov=fov,
                resolution=(self.width, self.height)
            )
            
            # Configure camera settings
            self.camera.set_resolution((self.width, self.height))
            self.camera.set_fov(fov)
            
            self.camera_state.update({
                'position': position,
                'target': target,
                'fov': fov
            })
            
            logger.info("📹 Real Isaac Sim camera setup completed", 
                       position=position, 
                       target=target, 
                       fov=fov)
            
            return True
            
        except Exception as e:
            logger.error("❌ Failed to setup real camera", error=str(e))
            return False
    
    async def update_joints(self, joint_states: Dict[str, float]):
        """Update robot joint states."""
        if not self.robot:
            logger.warning("No robot loaded - cannot update joints")
            return
        
        try:
            # Update joint states
            self.joint_states.update(joint_states)
            
            # Apply to robot articulation
            articulation = self.robot.get_articulation()
            joint_names = articulation.get_joint_names()
            
            for joint_name, angle in joint_states.items():
                if joint_name in joint_names:
                    articulation.set_joint_positions({joint_name: angle})
            
            logger.debug("🔧 Real joint states updated", joint_states=joint_states)
            
        except Exception as e:
            logger.error("❌ Failed to update real joints", error=str(e))
    
    async def render_frame(self) -> np.ndarray:
        """Render a photorealistic frame from Isaac Sim."""
        if not self.scene_initialized or not self.camera:
            logger.warning("Isaac Sim not ready - returning black frame")
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        try:
            # Step simulation
            self.world.step_async()
            
            # Capture frame from camera
            frame_data = self.camera.get_rgba()
            
            if frame_data is not None:
                # Convert to BGR format for OpenCV compatibility
                frame_bgr = frame_data[:, :, :3][:, :, ::-1]  # RGBA to BGR
                frame_bgr = (frame_bgr * 255).astype(np.uint8)
                
                self.frame_count += 1
                
                # Log every 60 frames (once per second)
                if self.frame_count % 60 == 0:
                    logger.info("🎬 Real Isaac Sim frame rendered", 
                               frame_count=self.frame_count,
                               robot_name=self.robot_config.get('name'))
                
                return frame_bgr
            else:
                logger.warning("No frame data from real Isaac Sim camera")
                return np.zeros((self.height, self.width, 3), dtype=np.uint8)
                
        except Exception as e:
            logger.error("❌ Failed to render real Isaac Sim frame", error=str(e))
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
    
    def update_camera(self, position: list, target: list, fov: float):
        """Update camera parameters."""
        self.camera_state.update({
            'position': position,
            'target': target,
            'fov': fov
        })
        
        # Update camera if it exists
        if self.camera:
            self.camera.set_position(position)
            self.camera.set_look_at(target)
            self.camera.set_fov(fov)
        
        logger.debug("📹 Real camera updated", position=position, target=target, fov=fov)
    
    def update_robot_config(self, robot_config: Dict[str, Any]):
        """Update robot configuration."""
        self.robot_config.update(robot_config)
        logger.info("🤖 Real robot configuration updated", 
                   robot_name=robot_config.get('name', 'Unknown'),
                   isaac_sim_path=robot_config.get('isaac_sim_path'))
    
    def cleanup(self):
        """Cleanup Isaac Sim resources."""
        try:
            if self.app:
                self.app.close()
                logger.info("🧹 Real Isaac Sim application closed")
        except Exception as e:
            logger.error("❌ Failed to cleanup real Isaac Sim", error=str(e))

# Global renderer instance
isaac_sim_real_renderer = IsaacSimRealRenderer()

def get_isaac_sim_real_renderer() -> IsaacSimRealRenderer:
    """Get the global real Isaac Sim renderer instance."""
    return isaac_sim_real_renderer