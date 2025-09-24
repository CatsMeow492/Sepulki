#!/usr/bin/env python3
"""
Fixed Base64 WebSocket Bridge Service for Anvil Sim
Properly encodes binary image data as base64 for transmission
"""

import asyncio
import websockets
import json
import cv2
import numpy as np
import base64
from datetime import datetime
import logging
import signal
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FixedVideoStreamBridge:
    def __init__(self, port=8765):
        self.port = port
        self.connected_clients = set()
        self.frame_count = 0
        self.running = True
        
    async def register_client(self, websocket):
        """Register a new client"""
        logger.info(f"📱 New client connected: {websocket.remote_address}")
        self.connected_clients.add(websocket)
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "connection_established",
                "message": "Connected to Anvil Sim WebSocket bridge",
                "timestamp": datetime.now().isoformat()
            }))
            
            # Handle messages
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
    
    async def start_server(self):
        """Start WebSocket server"""
        logger.info(f"🔌 Starting Fixed Base64 WebSocket bridge on port {self.port}")
        
        # Start server
        server = await websockets.serve(self.register_client, "0.0.0.0", self.port)
        logger.info(f"✅ Fixed Base64 WebSocket bridge running on port {self.port}")
        
        # Start video stream
        asyncio.create_task(self.generate_video_stream())
        
        # Keep server running
        try:
            await server.wait_closed()
        except KeyboardInterrupt:
            logger.info("🛑 Shutting down...")
            self.running = False
    
    async def handle_message(self, data, websocket):
        """Handle incoming messages"""
        message_type = data.get("type")
        
        if message_type == "start_video_stream":
            logger.info("🎬 Starting video stream for client")
            await websocket.send(json.dumps({
                "type": "video_stream_started",
                "message": "Video stream started",
                "timestamp": datetime.now().isoformat()
            }))
        elif message_type == "stop_video_stream":
            logger.info("⏹️ Stopping video stream for client")
        else:
            logger.warning(f"Unknown message type: {message_type}")
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames with proper base64 encoding"""
        logger.info("🎨 Starting video frame generation with fixed base64 encoding...")
        
        while self.running:
            if self.connected_clients:
                try:
                    # Create mock Isaac Sim frame
                    frame = self.create_mock_frame()
                    
                    # Convert to JPEG with proper encoding
                    success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    
                    if not success:
                        logger.error("Failed to encode frame as JPEG")
                        continue
                    
                    # Convert to base64 properly - this is the key fix
                    frame_base64 = base64.b64encode(buffer).decode('ascii')
                    
                    # Send to all connected clients
                    message = json.dumps({
                        "type": "video_frame",
                        "frame_data": frame_base64,
                        "frame_number": self.frame_count,
                        "timestamp": datetime.now().isoformat(),
                        "width": frame.shape[1],
                        "height": frame.shape[0]
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
                        logger.info(f"📹 Sent {self.frame_count} frames to {len(self.connected_clients)} clients (Fixed Base64)")
                    
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

def signal_handler(signum, frame):
    logger.info("🛑 Received interrupt signal")
    sys.exit(0)

async def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    bridge = FixedVideoStreamBridge()
    await bridge.start_server()

if __name__ == "__main__":
    asyncio.run(main())
