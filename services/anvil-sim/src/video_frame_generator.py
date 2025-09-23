#!/usr/bin/env python3
"""
Video Frame Generator - Creates realistic Isaac Sim-style video frames
Generates synthetic 3D robot simulation frames that respond to camera controls.
"""

import asyncio
import math
import time
from typing import Dict, Any, Tuple
import numpy as np

try:
    import cv2
    from PIL import Image, ImageDraw, ImageFont
    VIDEO_AVAILABLE = True
except ImportError:
    VIDEO_AVAILABLE = False
    print("Warning: Video dependencies not available. Install opencv-python and pillow.")

import structlog

logger = structlog.get_logger(__name__)

class IsaacSimVideoGenerator:
    """Generates Isaac Sim-style video frames for WebRTC streaming."""
    
    def __init__(self, width: int = 1920, height: int = 1080):
        self.width = width
        self.height = height
        self.frame_count = 0
        
        # Current camera state
        self.camera = {
            'position': [4.0, 4.0, 4.0],
            'target': [0.0, 0.0, 0.0],
            'fov': 50.0
        }
        
        # Current joint states
        self.joint_states = {
            'joint1': 0.0,
            'joint2': 0.0
        }
        
        # Current robot configuration
        self.robot_config = {
            'name': 'Default Robot',
            'isaac_sim_path': None,
            'specifications': {
                'dof': 2,
                'payload_kg': 5,
                'reach_m': 1.0
            }
        }
        
        # Simulation time
        self.sim_time = 0.0
        
    def update_camera(self, position: list, target: list, fov: float):
        """Update camera parameters."""
        self.camera['position'] = position
        self.camera['target'] = target  
        self.camera['fov'] = fov
        logger.debug("Camera updated", position=position, target=target, fov=fov)
        
    def update_joints(self, joint_states: Dict[str, float]):
        """Update robot joint states."""
        self.joint_states.update(joint_states)
        logger.debug("Joints updated", joint_states=joint_states)
        
    def update_robot_config(self, robot_config: Dict[str, Any]):
        """Update robot configuration for rendering."""
        self.robot_config.update(robot_config)
        logger.info("Robot configuration updated", 
                   robot_name=robot_config.get('name', 'Unknown'),
                   isaac_sim_path=robot_config.get('isaac_sim_path'))
        
    def generate_frame(self) -> np.ndarray:
        """Generate a single Isaac Sim-style video frame."""
        logger.debug("Generating video frame", frame_count=self.frame_count, 
                    robot_name=self.robot_config.get('name', 'Unknown'),
                    video_available=VIDEO_AVAILABLE)
                    
        if not VIDEO_AVAILABLE:
            # Return black frame if video libs unavailable
            logger.warning("Video libraries not available, returning black frame")
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
            
        # Create base image
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Professional Isaac Sim background (changes with camera position)
        self._render_background(frame)
        self._render_grid(frame)
        self._render_warehouse_environment(frame)
        self._render_robot(frame)
        self._render_hud(frame)
        
        self.frame_count += 1
        self.sim_time += 1.0 / 60.0  # 60 FPS simulation time
        
        # Debug: Check if frame has any non-black pixels
        non_black_pixels = np.count_nonzero(frame)
        total_pixels = frame.size
        
        # Log every 60 frames (once per second)
        if self.frame_count % 60 == 0:
            logger.info("Video frame generated", frame_count=self.frame_count, 
                       robot_name=self.robot_config.get('name', 'Unknown'),
                       non_black_pixels=non_black_pixels,
                       total_pixels=total_pixels,
                       frame_shape=frame.shape,
                       frame_min=frame.min(),
                       frame_max=frame.max())
        
        return frame
        
    def _render_background(self, frame: np.ndarray):
        """Render camera-dependent background."""
        # Different backgrounds based on camera position
        cam_x, cam_y, cam_z = self.camera['position']

        logger.debug("Rendering background", cam_x=cam_x, cam_y=cam_y, cam_z=cam_z, frame_shape=frame.shape)

        if cam_z > 4:  # Front view
            # Blue gradient
            for y in range(self.height):
                color_factor = y / self.height
                color = [int(20 + color_factor * 30), int(30 + color_factor * 40), int(60 + color_factor * 80)]
                frame[y, :] = color
            logger.debug("Applied front view blue gradient")
        elif cam_y > 4:  # Top view
            # Gray gradient
            for y in range(self.height):
                color_factor = y / self.height
                color = [int(40 + color_factor * 20), int(40 + color_factor * 20), int(40 + color_factor * 20)]
                frame[y, :] = color
            logger.debug("Applied top view gray gradient")
        elif abs(cam_x) > 4:  # Side view
            # Warm gradient
            for y in range(self.height):
                color_factor = y / self.height
                color = [int(60 + color_factor * 40), int(30 + color_factor * 20), int(20 + color_factor * 10)]
                frame[y, :] = color
            logger.debug("Applied side view warm gradient")
        else:  # Isometric
            # Default Isaac Sim gradient
            for y in range(self.height):
                color_factor = y / self.height
                color = [int(15 + color_factor * 20), int(25 + color_factor * 30), int(35 + color_factor * 40)]
                frame[y, :] = color
            logger.debug("Applied isometric default gradient")
                
    def _render_grid(self, frame: np.ndarray):
        """Render perspective grid (changes with camera distance)."""
        cam_distance = math.sqrt(sum(x*x for x in self.camera['position']))
        grid_size = max(50, int(100 - cam_distance * 5))
        
        # Horizontal lines
        for y in range(0, self.height, grid_size):
            cv2.line(frame, (0, y), (self.width, y), (60, 60, 60), 1)
            
        # Vertical lines  
        for x in range(0, self.width, grid_size):
            cv2.line(frame, (x, 0), (x, self.height), (60, 60, 60), 1)
            
    def _render_warehouse_environment(self, frame: np.ndarray):
        """Render warehouse shelving and floor."""
        # Floor (bottom 15%)
        floor_y = int(self.height * 0.85)
        cv2.rectangle(frame, (0, floor_y), (self.width, self.height), (50, 50, 50), -1)
        
        # Warehouse shelving (perspective based on camera)
        cam_x = self.camera['position'][0]
        shelf_offset = int(cam_x * 20)  # Parallax effect
        
        # Left shelf
        cv2.rectangle(frame, (100 + shelf_offset, int(self.height * 0.3)), 
                     (120 + shelf_offset, int(self.height * 0.85)), (80, 80, 80), -1)
        
        # Right shelf  
        cv2.rectangle(frame, (200 + shelf_offset, int(self.height * 0.3)),
                     (220 + shelf_offset, int(self.height * 0.85)), (80, 80, 80), -1)
        
        # Shelf levels
        for level in [0.4, 0.5, 0.6, 0.7]:
            y = int(self.height * level)
            cv2.rectangle(frame, (100 + shelf_offset, y), (220 + shelf_offset, y + 8), (100, 100, 100), -1)
            
    def _render_robot(self, frame: np.ndarray):
        """Render robot with current joint positions (specific to selected robot model)."""
        # Calculate robot position based on camera
        cam_x, cam_y, cam_z = self.camera['position']
        
        # Perspective scaling
        cam_distance = math.sqrt(cam_x*cam_x + cam_y*cam_y + cam_z*cam_z)
        scale = min(2.0, 8.0 / max(cam_distance, 2.0))
        
        # Robot center (with parallax)
        center_x = int(self.width / 2 - cam_x * 15)
        center_y = int(self.height / 2 - cam_y * 10)
        
        robot_name = self.robot_config.get('name', 'Default Robot')

        logger.debug("Rendering robot", robot_name=robot_name, center_x=center_x, center_y=center_y, scale=scale, cam_distance=cam_distance)

        # Render different robot types based on configuration
        if 'Franka' in robot_name or 'Panda' in robot_name:
            logger.debug("Rendering Franka Panda robot")
            self._render_franka_panda(frame, center_x, center_y, scale)
        elif 'UR5' in robot_name or 'UR10' in robot_name:
            logger.debug("Rendering Universal Robot")
            self._render_universal_robot(frame, center_x, center_y, scale)
        elif 'KUKA' in robot_name or 'KR210' in robot_name:
            logger.debug("Rendering KUKA robot")
            self._render_kuka_robot(frame, center_x, center_y, scale)
        elif 'Carter' in robot_name or 'mobile' in robot_name.lower():
            logger.debug("Rendering mobile robot")
            self._render_mobile_robot(frame, center_x, center_y, scale)
        elif 'TurtleBot' in robot_name:
            logger.debug("Rendering TurtleBot")
            self._render_turtlebot(frame, center_x, center_y, scale)
        else:
            logger.debug("Rendering generic robot arm")
            # Default generic robot arm
            self._render_generic_arm(frame, center_x, center_y, scale)
            
    def _render_franka_panda(self, frame: np.ndarray, center_x: int, center_y: int, scale: float):
        """Render Franka Emika Panda robot."""
        # Franka Panda has 7 DOF and distinctive white/gray appearance
        joint1_angle = self.joint_states.get('joint1', 0.0)
        joint2_angle = self.joint_states.get('joint2', 0.0)
        
        # Distinctive white/gray Franka base
        base_size = int(40 * scale)
        cv2.rectangle(frame, 
                     (center_x - base_size//2, center_y + int(50 * scale)),
                     (center_x + base_size//2, center_y + int(80 * scale)),
                     (240, 240, 240), -1)  # Light gray/white
        
        # Franka arm segments (7-DOF, but we'll show main 2 for demo)
        arm1_length = int(90 * scale)
        arm1_end_x = center_x + int(math.cos(joint1_angle - math.pi/2) * arm1_length)
        arm1_end_y = center_y + int(math.sin(joint1_angle - math.pi/2) * arm1_length)
        
        cv2.line(frame, (center_x, center_y), (arm1_end_x, arm1_end_y), (200, 200, 200), int(10 * scale))
        
        # Franka joint indicators (orange/black theme)
        cv2.circle(frame, (center_x, center_y), int(8 * scale), (255, 140, 0), -1)  # Orange
        
        # Second segment
        arm2_length = int(70 * scale)
        arm2_end_x = arm1_end_x + int(math.cos(joint1_angle + joint2_angle - math.pi/2) * arm2_length)
        arm2_end_y = arm1_end_y + int(math.sin(joint1_angle + joint2_angle - math.pi/2) * arm2_length)
        
        cv2.line(frame, (arm1_end_x, arm1_end_y), (arm2_end_x, arm2_end_y), (180, 180, 180), int(8 * scale))
        cv2.circle(frame, (arm1_end_x, arm1_end_y), int(6 * scale), (255, 140, 0), -1)
        
        # Franka gripper (distinctive design)
        cv2.circle(frame, (arm2_end_x, arm2_end_y), int(12 * scale), (100, 100, 100), -1)
        
    def _render_universal_robot(self, frame: np.ndarray, center_x: int, center_y: int, scale: float):
        """Render Universal Robots UR10e - MASSIVE BLUE INDUSTRIAL ROBOT (UNMISTAKABLY DIFFERENT)."""
        # UR robots have 6 DOF and MASSIVE distinctive blue industrial appearance
        joint1_angle = self.joint_states.get('joint1', 0.0)
        joint2_angle = self.joint_states.get('joint2', 0.0)
        
        # 🏭 MASSIVE BLUE INDUSTRIAL BASE (10x larger than Franka)
        base_width = int(200 * scale)  # HUGE width  
        base_height = int(80 * scale)  # THICK height
        cv2.rectangle(frame, 
                     (center_x - base_width//2, center_y + int(20 * scale)),
                     (center_x + base_width//2, center_y + int(100 * scale)),
                     (0, 100, 255), -1)  # BRIGHT ELECTRIC BLUE
        
        # Giant "UR10e INDUSTRIAL" text across the entire base
        cv2.putText(frame, "UR10e INDUSTRIAL", (center_x - 120, center_y + 65), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        
        # 🦾 MASSIVE BLUE ARM SEGMENTS (much thicker and longer than Franka)
        arm1_length = int(200 * scale)  # MUCH longer than Franka
        arm1_end_x = center_x + int(math.cos(joint1_angle - math.pi/2) * arm1_length)
        arm1_end_y = center_y + int(math.sin(joint1_angle - math.pi/2) * arm1_length)
        
        cv2.line(frame, (center_x, center_y), (arm1_end_x, arm1_end_y), (0, 120, 255), int(35 * scale))  # THICK BLUE
        
        # Second MASSIVE segment
        arm2_length = int(160 * scale)  # MUCH longer
        arm2_angle = joint1_angle + joint2_angle
        arm2_end_x = arm1_end_x + int(math.cos(arm2_angle - math.pi/2) * arm2_length)
        arm2_end_y = arm1_end_y + int(math.sin(arm2_angle - math.pi/2) * arm2_length)
        
        cv2.line(frame, (arm1_end_x, arm1_end_y), (arm2_end_x, arm2_end_y), (0, 80, 200), int(30 * scale))  # THICK DARK BLUE
        
        # 🔵 MASSIVE UR joint indicators (HUGE blue circles)
        cv2.circle(frame, (center_x, center_y), int(25 * scale), (100, 200, 255), -1)  # HUGE Light blue
        cv2.circle(frame, (arm1_end_x, arm1_end_y), int(20 * scale), (0, 100, 200), -1)  # HUGE Dark blue
        
        # 🟠 MASSIVE industrial end effector (orange to contrast blue)
        cv2.rectangle(frame, 
                     (arm2_end_x - int(40 * scale), arm2_end_y - int(25 * scale)),
                     (arm2_end_x + int(40 * scale), arm2_end_y + int(25 * scale)),
                     (255, 140, 0), -1)  # MASSIVE orange gripper
        
        # UR end effector
        cv2.circle(frame, (arm2_end_x, arm2_end_y), int(10 * scale), (30, 80, 160), -1)
        
    def _render_kuka_robot(self, frame: np.ndarray, center_x: int, center_y: int, scale: float):
        """Render KUKA KR210 industrial robot."""
        # KUKA robots have orange/gray industrial design
        joint1_angle = self.joint_states.get('joint1', 0.0)
        joint2_angle = self.joint_states.get('joint2', 0.0)
        
        # Large industrial base (KUKA orange)
        base_size = int(50 * scale)
        cv2.rectangle(frame, 
                     (center_x - base_size//2, center_y + int(60 * scale)),
                     (center_x + base_size//2, center_y + int(100 * scale)),
                     (100, 130, 255), -1)  # KUKA orange
        
        # Heavy-duty arm segments (larger for KR210)
        arm1_length = int(120 * scale)  # Longer reach for KR210
        arm1_end_x = center_x + int(math.cos(joint1_angle - math.pi/2) * arm1_length)
        arm1_end_y = center_y + int(math.sin(joint1_angle - math.pi/2) * arm1_length)
        
        cv2.line(frame, (center_x, center_y), (arm1_end_x, arm1_end_y), (120, 150, 255), int(15 * scale))
        
        # KUKA joint indicators (orange theme)
        cv2.circle(frame, (center_x, center_y), int(10 * scale), (255, 165, 0), -1)  # KUKA orange
        
        # Second segment (heavy duty)
        arm2_length = int(100 * scale)
        arm2_end_x = arm1_end_x + int(math.cos(joint1_angle + joint2_angle - math.pi/2) * arm2_length)
        arm2_end_y = arm1_end_y + int(math.sin(joint1_angle + joint2_angle - math.pi/2) * arm2_length)
        
        cv2.line(frame, (arm1_end_x, arm1_end_y), (arm2_end_x, arm2_end_y), (140, 170, 255), int(12 * scale))
        cv2.circle(frame, (arm1_end_x, arm1_end_y), int(8 * scale), (255, 165, 0), -1)
        
        # Heavy-duty end effector
        cv2.circle(frame, (arm2_end_x, arm2_end_y), int(15 * scale), (200, 130, 0), -1)
        
    def _render_mobile_robot(self, frame: np.ndarray, center_x: int, center_y: int, scale: float):
        """Render mobile robot (Nova Carter, TurtleBot).""" 
        # Mobile robots are rendered as platforms with wheels
        platform_width = int(80 * scale)
        platform_height = int(40 * scale)
        
        # Robot platform
        cv2.rectangle(frame,
                     (center_x - platform_width//2, center_y - platform_height//2),
                     (center_x + platform_width//2, center_y + platform_height//2),
                     (50, 150, 50), -1)  # Green platform
        
        # Wheels
        wheel_radius = int(15 * scale)
        cv2.circle(frame, (center_x - platform_width//2, center_y + platform_height//2), wheel_radius, (40, 40, 40), -1)
        cv2.circle(frame, (center_x + platform_width//2, center_y + platform_height//2), wheel_radius, (40, 40, 40), -1)
        cv2.circle(frame, (center_x - platform_width//2, center_y - platform_height//2), wheel_radius, (40, 40, 40), -1)
        cv2.circle(frame, (center_x + platform_width//2, center_y - platform_height//2), wheel_radius, (40, 40, 40), -1)
        
        # Sensors/cameras on top
        cv2.circle(frame, (center_x, center_y - platform_height//2 - int(10 * scale)), int(8 * scale), (100, 200, 255), -1)
        
    def _render_turtlebot(self, frame: np.ndarray, center_x: int, center_y: int, scale: float):
        """Render TurtleBot3."""
        # TurtleBot has distinctive circular design
        platform_radius = int(30 * scale)
        
        # Circular platform
        cv2.circle(frame, (center_x, center_y), platform_radius, (100, 100, 200), -1)
        
        # LiDAR on top
        cv2.circle(frame, (center_x, center_y - int(20 * scale)), int(8 * scale), (200, 100, 100), -1)
        
        # Wheels
        wheel_radius = int(8 * scale)
        cv2.circle(frame, (center_x - platform_radius, center_y), wheel_radius, (40, 40, 40), -1)
        cv2.circle(frame, (center_x + platform_radius, center_y), wheel_radius, (40, 40, 40), -1)
        
    def _render_generic_arm(self, frame: np.ndarray, center_x: int, center_y: int, scale: float):
        """Render generic robot arm (fallback)."""
        joint1_angle = self.joint_states.get('joint1', 0.0)
        joint2_angle = self.joint_states.get('joint2', 0.0)
        
        # Generic robot base
        base_size = int(30 * scale)
        cv2.rectangle(frame, 
                     (center_x - base_size//2, center_y + int(40 * scale)),
                     (center_x + base_size//2, center_y + int(60 * scale)),
                     (70, 130, 180), -1)
        
        # Generic arm rendering with joint angles
        arm1_length = int(80 * scale)
        arm1_end_x = center_x + int(math.cos(joint1_angle - math.pi/2) * arm1_length)
        arm1_end_y = center_y + int(math.sin(joint1_angle - math.pi/2) * arm1_length)
        
        cv2.line(frame, (center_x, center_y), (arm1_end_x, arm1_end_y), (60, 150, 220), int(8 * scale))
        
        # Joint indicators
        cv2.circle(frame, (center_x, center_y), int(6 * scale), (220, 80, 80), -1)
        
        # Second link  
        arm2_length = int(60 * scale)
        arm2_end_x = arm1_end_x + int(math.cos(joint1_angle + joint2_angle - math.pi/2) * arm2_length)
        arm2_end_y = arm1_end_y + int(math.sin(joint1_angle + joint2_angle - math.pi/2) * arm2_length)
        
        cv2.line(frame, (arm1_end_x, arm1_end_y), (arm2_end_x, arm2_end_y), (150, 100, 220), int(6 * scale))
        cv2.circle(frame, (arm1_end_x, arm1_end_y), int(4 * scale), (220, 150, 80), -1)
        
        # End effector
        cv2.circle(frame, (arm2_end_x, arm2_end_y), int(8 * scale), (220, 200, 80), -1)
        
    def _render_hud(self, frame: np.ndarray):
        """Render Isaac Sim HUD and information overlay.""" 
        # NVIDIA Isaac Sim branding
        cv2.putText(frame, 'NVIDIA Isaac Sim', (30, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        
        cv2.putText(frame, 'PHYSICS SIMULATION', (30, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 200, 255), 2)
        
        # Selected robot name
        robot_name = self.robot_config.get('name', 'Default Robot')
        cv2.putText(frame, f'Robot: {robot_name}', (30, 115),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 255, 150), 2)
        
        # Camera info (updates with camera changes)
        pos_text = f"Camera: ({self.camera['position'][0]:.1f}, {self.camera['position'][1]:.1f}, {self.camera['position'][2]:.1f})"
        cv2.putText(frame, pos_text, (30, 145), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 200), 1)
        
        fov_text = f"FOV: {self.camera['fov']:.0f}°"
        cv2.putText(frame, fov_text, (30, 170),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 200), 1)
        
        # Joint states
        y_offset = 205
        for joint_name, angle in self.joint_states.items():
            joint_text = f"{joint_name}: {angle:.3f} rad"
            cv2.putText(frame, joint_text, (30, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
            y_offset += 25
            
        # Frame counter and timing
        cv2.putText(frame, f"Frame: {self.frame_count}", (self.width - 200, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.putText(frame, f"Time: {self.sim_time:.1f}s", (self.width - 200, 75),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

# Global video generator instance  
video_generator = IsaacSimVideoGenerator()

def get_video_generator() -> IsaacSimVideoGenerator:
    """Get the global video generator instance."""
    return video_generator
