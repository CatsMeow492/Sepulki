#!/usr/bin/env python3
"""
Test Video Generator - Creates a test video file that aiortc can definitely stream
"""

import cv2
import numpy as np
import os
import tempfile
import structlog

logger = structlog.get_logger(__name__)

def create_test_video_file(client_id: str, duration_seconds: int = 10) -> str:
    """Create a test video file that shows animated Isaac Sim content."""
    
    # Create temporary video file
    temp_dir = tempfile.gettempdir()
    video_path = os.path.join(temp_dir, f"isaac_sim_test_{client_id}.mp4")
    
    try:
        # Video settings
        fps = 30
        width = 640
        height = 480
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        # Create video writer
        out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
        
        total_frames = fps * duration_seconds
        
        for frame_num in range(total_frames):
            # Create animated frame
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Animated background
            t = frame_num / fps
            color_r = int(128 + 127 * np.sin(t * 2))
            color_g = int(128 + 127 * np.sin(t * 2 + 2))
            color_b = int(128 + 127 * np.sin(t * 2 + 4))
            frame[:] = [color_b, color_g, color_r]  # BGR format
            
            # Isaac Sim text
            cv2.putText(frame, 'ISAAC SIM TEST STREAM', (50, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            cv2.putText(frame, f'Frame: {frame_num}', (50, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.putText(frame, f'Time: {t:.2f}s', (50, 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Animated robot representation
            center_x = width // 2
            center_y = height // 2 + 50
            
            # Robot arm movement
            arm_angle = t * 2  # 2 rad/s rotation
            arm_length = 80
            arm_end_x = int(center_x + arm_length * np.cos(arm_angle))
            arm_end_y = int(center_y + arm_length * np.sin(arm_angle))
            
            # Draw robot base
            cv2.circle(frame, (center_x, center_y), 20, (100, 150, 200), -1)
            
            # Draw robot arm
            cv2.line(frame, (center_x, center_y), (arm_end_x, arm_end_y), (0, 255, 0), 8)
            
            # Draw end effector
            cv2.circle(frame, (arm_end_x, arm_end_y), 10, (0, 0, 255), -1)
            
            # Frame counter
            cv2.putText(frame, f'FPS: {fps}', (width - 120, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            out.write(frame)
        
        out.release()
        
        logger.info("✅ Test video file created", 
                   client_id=client_id, path=video_path, 
                   frames=total_frames, duration=duration_seconds)
        
        return video_path
        
    except Exception as e:
        logger.error("❌ Failed to create test video file", 
                    client_id=client_id, error=str(e))
        return None

def cleanup_test_video(video_path: str):
    """Clean up temporary video file."""
    try:
        if video_path and os.path.exists(video_path):
            os.remove(video_path)
            logger.info("🗑️ Test video file cleaned up", path=video_path)
    except Exception as e:
        logger.warning("⚠️ Failed to cleanup test video", path=video_path, error=str(e))
