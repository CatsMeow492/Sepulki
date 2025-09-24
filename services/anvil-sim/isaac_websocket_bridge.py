#!/usr/bin/env python3
"""
Isaac Sim WebSocket Bridge Service
Connects to real Isaac Sim livestream and forwards to WebSocket clients
"""

import asyncio
import websockets
import json
import cv2
import numpy as np
import aiohttp
import base64
from datetime import datetime
import logging
import signal
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class IsaacVideoStreamBridge:
    def __init__(self, port=8765, isaac_livestream_url="http://localhost:49100"):
        self.port = port
        self.isaac_livestream_url = isaac_livestream_url
        self.connected_clients = set()
        self.frame_count = 0
        self.running = True
        self.isaac_available = False
        
    async def register_client(self, websocket):
        """Register a new client"""
        logger.info(f"📱 New client connected: {websocket.remote_address}")
        self.connected_clients.add(websocket)
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "connection_established",
                "message": "Connected to Isaac Sim WebSocket bridge",
                "isaac_available": self.isaac_available,
                "timestamp": datetime.now().isoformat()
            }))
            
            # Handle messages from this client
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_message(data, websocket)
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON: {message}")
                except Exception as e:
                    logger.error(f"Error handling message: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {websocket.remote_address}")
        finally:
            self.connected_clients.discard(websocket)
    
    async def handle_message(self, data, websocket):
        """Handle incoming messages"""
        message_type = data.get("type")
        
        if message_type == "start_video_stream":
            logger.info("🎬 Starting video stream for client")
            await websocket.send(json.dumps({
                "type": "video_stream_started",
                "message": "Video stream started",
                "isaac_available": self.isaac_available,
                "timestamp": datetime.now().isoformat()
            }))
        elif message_type == "stop_video_stream":
            logger.info("⏹️ Stopping video stream")
        else:
            logger.warning(f"Unknown message type: {message_type}")
    
    async def check_isaac_availability(self):
        """Check if Isaac Sim livestream is available"""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(self.isaac_livestream_url, timeout=5) as response:
                    if response.status == 200:
                        self.isaac_available = True
                        logger.info("✅ Isaac Sim livestream is available")
                        return True
                    else:
                        logger.warning(f"⚠️ Isaac Sim livestream returned status {response.status}")
                        return False
        except Exception as e:
            logger.warning(f"⚠️ Isaac Sim livestream not available: {e}")
            return False
    
    async def fetch_isaac_frame(self):
        """Fetch a frame from Isaac Sim livestream"""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(self.isaac_livestream_url, timeout=2) as response:
                    if response.status == 200:
                        # Read the image data
                        image_data = await response.read()
                        
                        # Convert to numpy array
                        nparr = np.frombuffer(image_data, np.uint8)
                        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            return frame
                        else:
                            logger.warning("Failed to decode Isaac Sim frame")
                            return None
                    else:
                        logger.warning(f"Isaac Sim returned status {response.status}")
                        return None
        except Exception as e:
            logger.warning(f"Error fetching Isaac Sim frame: {e}")
            return None
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames"""
        logger.info("🎨 Starting video frame generation...")
        
        # Check Isaac Sim availability periodically
        last_isaac_check = 0
        
        while self.running:
            if self.connected_clients:
                try:
                    current_time = asyncio.get_event_loop().time()
                    
                    # Check Isaac Sim availability every 10 seconds
                    if current_time - last_isaac_check > 10:
                        await self.check_isaac_availability()
                        last_isaac_check = current_time
                    
                    # Try to get frame from Isaac Sim first
                    frame = None
                    if self.isaac_available:
                        frame = await self.fetch_isaac_frame()
                    
                    # Fallback to mock frame if Isaac Sim is not available or frame fetch failed
                    if frame is None:
                        frame = self.create_mock_frame()
                        if self.frame_count % 60 == 0:  # Log every 4 seconds at 15 FPS
                            logger.info("📹 Using mock frames (Isaac Sim not available)")
                    
                    # Convert to base64
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    frame_base64 = buffer.tobytes().decode('latin-1')
                    
                    # Send to all connected clients
                    message = json.dumps({
                        "type": "video_frame",
                        "frame_data": frame_base64,
                        "frame_number": self.frame_count,
                        "timestamp": datetime.now().isoformat(),
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                        "source": "isaac_sim" if frame is not None and self.isaac_available else "mock"
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
                        source_type = "Isaac Sim" if self.isaac_available else "Mock"
                        logger.info(f"📹 Sent {self.frame_count} frames to {len(self.connected_clients)} clients ({source_type})")
                    
                except Exception as e:
                    logger.error(f"Error generating frame: {e}")
            
            # 15 FPS
            await asyncio.sleep(1/15)
    
    def create_mock_frame(self):
        """Create mock Isaac Sim frame"""
        # Create 1920x1080 frame
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        # Background gradient
        for y in range(1080):
            color = int(30 + (y / 1080) * 20)
            frame[y, :] = [color, color + 10, color + 20]
        
        # Isaac Sim HUD
        cv2.rectangle(frame, (10, 10), (300, 150), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (300, 150), (0, 255, 0), 2)
        cv2.putText(frame, "NVIDIA Isaac Sim", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "PHYSICS", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"Frame: {self.frame_count}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, "Robot: Franka Emika Panda", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Status indicator
        status_color = (0, 255, 0) if self.isaac_available else (0, 165, 255)  # Green or Orange
        status_text = "LIVE" if self.isaac_available else "MOCK"
        cv2.putText(frame, f"Status: {status_text}", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
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

async def main():
    """Main entry point"""
    bridge = IsaacVideoStreamBridge()
    
    logger.info(f"🔌 Starting Isaac Sim WebSocket bridge on port {bridge.port}")
    logger.info(f"🎬 Isaac Sim livestream URL: {bridge.isaac_livestream_url}")
    
    # Check initial Isaac Sim availability
    await bridge.check_isaac_availability()
    
    # Start video stream generation
    asyncio.create_task(bridge.generate_video_stream())
    
    # Start WebSocket server
    async with websockets.serve(bridge.register_client, "0.0.0.0", bridge.port):
        logger.info(f"✅ Isaac Sim WebSocket bridge running on port {bridge.port}")
        await asyncio.Future()  # Keep running

def signal_handler(signum, frame):
    logger.info("🛑 Received interrupt signal")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
