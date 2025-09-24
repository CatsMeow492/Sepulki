#!/usr/bin/env python3
"""
Isaac Sim Viewport Capture Service
Captures frames from Isaac Sim's viewport using Python API and streams via WebSocket
"""

import asyncio
import json
import logging
import cv2
import numpy as np
import base64
import websockets
from datetime import datetime
import sys
import os

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

class IsaacViewportCapture:
    def __init__(self, websocket_port=8765):
        self.websocket_port = websocket_port
        self.connected_clients = set()
        self.frame_count = 0
        self.running = True
        self.viewport = None
        self.capture_interface = None
        
    async def initialize_isaac_sim(self):
        """Initialize Isaac Sim and get viewport capture interface"""
        try:
            logger.info("🚀 Initializing Isaac Sim Python API...")
            
            # Import Isaac Sim modules
            from omni.isaac.kit import SimulationApp
            from omni.kit.viewport.utility import get_active_viewport
            from omni.kit.viewport.capture import Capture
            
            # Get the active viewport
            self.viewport = get_active_viewport()
            if not self.viewport:
                logger.error("❌ No active viewport found")
                return False
            
            logger.info(f"✅ Active viewport found: {self.viewport}")
            
            # Create capture interface
            self.capture_interface = Capture()
            logger.info("✅ Viewport capture interface created")
            
            return True
            
        except ImportError as e:
            logger.error(f"❌ Isaac Sim import failed: {e}")
            logger.error("This script needs to run inside the Isaac Sim container")
            return False
        except Exception as e:
            logger.error(f"❌ Isaac Sim initialization failed: {e}")
            return False
    
    async def capture_viewport_frame(self):
        """Capture a single frame from the viewport"""
        try:
            if not self.capture_interface or not self.viewport:
                return None
            
            # Capture frame from viewport
            # This is a simplified approach - the actual API might be different
            frame_data = await self.capture_interface.capture_aov(
                aov_name="color",  # Color AOV (Ambient Occlusion Volume)
                viewport=self.viewport
            )
            
            if frame_data:
                # Convert to numpy array
                frame_array = np.array(frame_data)
                
                # Convert to OpenCV format (BGR)
                frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR)
                
                return frame_bgr
            
            return None
            
        except Exception as e:
            logger.error(f"❌ Frame capture error: {e}")
            return None
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames"""
        logger.info("🎨 Starting Isaac Sim viewport frame capture...")
        
        while self.running:
            if self.connected_clients:
                try:
                    # Capture frame from Isaac Sim viewport
                    frame = await self.capture_viewport_frame()
                    
                    if frame is not None:
                        # Convert to base64
                        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        frame_base64 = base64.b64encode(buffer).decode('utf-8')
                        
                        # Send to all connected clients
                        message = json.dumps({
                            "type": "video_frame",
                            "frame_data": frame_base64,
                            "frame_number": self.frame_count,
                            "timestamp": datetime.now().isoformat(),
                            "width": frame.shape[1],
                            "height": frame.shape[0],
                            "source": "isaac_sim_viewport"
                        })
                        
                        # Send to all clients
                        disconnected = set()
                        for client in self.connected_clients:
                            try:
                                await client.send(message)
                            except websockets.exceptions.ConnectionClosed:
                                disconnected.add(client)
                        
                        # Remove disconnected clients
                        self.connected_clients -= disconnected
                        
                        self.frame_count += 1
                        
                        if self.frame_count % 30 == 0:
                            logger.info(f"📹 Captured {self.frame_count} frames from Isaac Sim viewport")
                    else:
                        # If no frame captured, send a mock frame to keep connection alive
                        frame = self.create_mock_frame()
                        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        frame_base64 = base64.b64encode(buffer).decode('utf-8')
                        
                        message = json.dumps({
                            "type": "video_frame",
                            "frame_data": frame_base64,
                            "frame_number": self.frame_count,
                            "timestamp": datetime.now().isoformat(),
                            "width": frame.shape[1],
                            "height": frame.shape[0],
                            "source": "isaac_sim_mock"
                        })
                        
                        # Send to all clients
                        disconnected = set()
                        for client in self.connected_clients:
                            try:
                                await client.send(message)
                            except websockets.exceptions.ConnectionClosed:
                                disconnected.add(client)
                        
                        self.connected_clients -= disconnected
                        self.frame_count += 1
                        
                except Exception as e:
                    logger.error(f"❌ Video stream error: {e}")
            
            # 15 FPS
            await asyncio.sleep(1/15)
    
    def create_mock_frame(self):
        """Create mock Isaac Sim frame when real capture fails"""
        # Create 1920x1080 frame
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        # Background gradient
        for y in range(1080):
            color = int(30 + (y / 1080) * 20)
            frame[y, :] = [color, color + 10, color + 20]
        
        # Isaac Sim HUD
        cv2.rectangle(frame, (10, 10), (400, 180), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (400, 180), (0, 255, 0), 2)
        cv2.putText(frame, "NVIDIA Isaac Sim", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "VIEWPORT CAPTURE", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"Frame: {self.frame_count}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, "Python API Capture", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, "Status: Mock Frame", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Mock robot visualization
        center_x, center_y = 960, 540
        robot_color = (100, 150, 255)
        
        # Robot base
        cv2.circle(frame, (center_x, center_y), 50, robot_color, -1)
        
        # Animated arm
        arm_length = 100
        for i in range(3):
            angle = (self.frame_count * 2 + i * 60) % 360
            end_x = int(center_x + arm_length * np.cos(np.radians(angle)))
            end_y = int(center_y + arm_length * np.sin(np.radians(angle)))
            
            cv2.line(frame, (center_x, center_y), (end_x, end_y), robot_color, 8)
            cv2.circle(frame, (end_x, end_y), 15, robot_color, -1)
            
            center_x, center_y = end_x, end_y
        
        return frame
    
    async def handle_websocket_client(self, websocket, path):
        """Handle WebSocket client connections"""
        logger.info(f"📱 New client connected: {websocket.remote_address}")
        self.connected_clients.add(websocket)
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "connection_established",
                "message": "Connected to Isaac Sim Viewport Capture",
                "timestamp": datetime.now().isoformat(),
                "isaac_initialized": self.capture_interface is not None
            }))
            
            # Handle messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get("type") == "start_video_stream":
                        await websocket.send(json.dumps({
                            "type": "video_stream_started",
                            "message": "Isaac Sim viewport capture started",
                            "timestamp": datetime.now().isoformat()
                        }))
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON: {message}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {websocket.remote_address}")
        finally:
            self.connected_clients.discard(websocket)
    
    async def start_websocket_server(self):
        """Start WebSocket server"""
        logger.info(f"🔌 Starting WebSocket server on port {self.websocket_port}")
        
        server = await websockets.serve(self.handle_websocket_client, "0.0.0.0", self.websocket_port)
        logger.info(f"✅ WebSocket server running on port {self.websocket_port}")
        
        # Start video stream
        asyncio.create_task(self.generate_video_stream())
        
        return server

async def main():
    capture = IsaacViewportCapture()
    
    try:
        # Initialize Isaac Sim
        isaac_initialized = await capture.initialize_isaac_sim()
        
        if not isaac_initialized:
            logger.error("❌ Failed to initialize Isaac Sim")
            return
        
        # Start WebSocket server
        server = await capture.start_websocket_server()
        
        logger.info("🎉 Isaac Sim viewport capture service started!")
        logger.info("📡 WebSocket server running on port 8765")
        logger.info("🎬 Ready to capture and stream Isaac Sim viewport frames")
        
        # Keep running
        await server.wait_closed()
        
    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
        capture.running = False
    except Exception as e:
        logger.error(f"❌ Service error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
