#!/usr/bin/env python3
"""
Isaac Sim WebRTC Client
Properly connects to Isaac Sim's WebRTC livestream using aiortc
"""

import asyncio
import json
import logging
import cv2
import numpy as np
import base64
import websockets
from datetime import datetime
import aiohttp
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaPlayer

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class IsaacWebRTCClient:
    def __init__(self, websocket_port=8765, isaac_host="localhost", isaac_port=49100):
        self.websocket_port = websocket_port
        self.isaac_host = isaac_host
        self.isaac_port = isaac_port
        self.connected_clients = set()
        self.frame_count = 0
        self.running = True
        self.current_frame = None
        self.isaac_connected = False
        self.pc = None
        
    async def connect_to_isaac_webrtc(self):
        """Connect to Isaac Sim's WebRTC livestream"""
        try:
            logger.info(f"🔌 Connecting to Isaac Sim WebRTC at {self.isaac_host}:{self.isaac_port}")
            
            # Create WebRTC peer connection
            self.pc = RTCPeerConnection()
            
            # Set up video track handler
            @self.pc.on("track")
            async def on_track(track):
                logger.info(f"📹 Received track: {track.kind}")
                if track.kind == "video":
                    self.isaac_connected = True
                    logger.info("✅ Isaac Sim video track received")
                    
                    # Process video frames
                    while True:
                        try:
                            frame = await track.recv()
                            # Convert frame to OpenCV format
                            img = frame.to_ndarray(format="bgr24")
                            self.current_frame = img
                            self.frame_count += 1
                            
                            if self.frame_count % 30 == 0:
                                logger.info(f"📹 Received {self.frame_count} frames from Isaac Sim")
                                
                        except Exception as e:
                            logger.error(f"❌ Error receiving frame: {e}")
                            break
            
            # Create offer
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            
            # Send offer to Isaac Sim via HTTP POST
            offer_data = {
                "type": offer.type,
                "sdp": offer.sdp
            }
            
            async with aiohttp.ClientSession() as session:
                try:
                    async with session.post(f"http://{self.isaac_host}:{self.isaac_port}/webrtc/offer", 
                                           json=offer_data, timeout=10) as response:
                        if response.status == 200:
                            answer_data = await response.json()
                            
                            # Set remote description
                            answer = RTCSessionDescription(
                                sdp=answer_data["sdp"],
                                type=answer_data["type"]
                            )
                            await self.pc.setRemoteDescription(answer)
                            
                            logger.info("✅ WebRTC connection established with Isaac Sim")
                            return True
                        else:
                            logger.error(f"❌ WebRTC offer failed: {response.status}")
                            return False
                            
                except Exception as e:
                    logger.error(f"❌ WebRTC connection error: {e}")
                    return False
                    
        except Exception as e:
            logger.error(f"❌ Isaac Sim WebRTC connection failed: {e}")
            return False
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames"""
        logger.info("🎨 Starting video stream generation...")
        
        while self.running:
            if self.connected_clients:
                try:
                    # Use real Isaac Sim frame if available
                    if self.current_frame is not None and self.isaac_connected:
                        frame = self.current_frame
                        frame_source = "isaac_sim_webrtc"
                    else:
                        # Create fallback frame
                        frame = self.create_fallback_frame()
                        frame_source = "fallback"
                    
                    # Convert to base64
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    frame_base64 = base64.b64encode(buffer).decode('utf-8')
                    
                    # Send to all connected clients
                    message = json.dumps({
                        "type": "video_frame",
                        "frame_data": frame_base64,
                        "frame_number": self.frame_count,
                        "timestamp": datetime.now().isoformat(),
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                        "source": frame_source,
                        "isaac_connected": self.isaac_connected
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
                        logger.info(f"📹 Streamed {self.frame_count} frames to {len(self.connected_clients)} clients (Source: {frame_source})")
                        
                except Exception as e:
                    logger.error(f"❌ Video stream error: {e}")
            
            # 15 FPS
            await asyncio.sleep(1/15)
    
    def create_fallback_frame(self):
        """Create fallback frame when Isaac Sim is not connected"""
        # Create 1920x1080 frame
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        # Background gradient
        for y in range(1080):
            color = int(20 + (y / 1080) * 10)
            frame[y, :] = [color, color + 5, color + 10]
        
        # Status message
        cv2.putText(frame, "Isaac Sim WebRTC Connection Failed", (600, 500), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        cv2.putText(frame, "Using Fallback Content", (700, 550), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        
        return frame
    
    async def handle_websocket_client(self, websocket):
        """Handle WebSocket client connections"""
        logger.info(f"📱 New client connected: {websocket.remote_address}")
        self.connected_clients.add(websocket)
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "connection_established",
                "message": "Connected to Isaac Sim WebRTC Client",
                "timestamp": datetime.now().isoformat(),
                "isaac_connected": self.isaac_connected,
                "stream_type": "isaac_webrtc"
            }))
            
            # Handle messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get("type") == "start_video_stream":
                        await websocket.send(json.dumps({
                            "type": "video_stream_started",
                            "message": "Isaac Sim WebRTC stream started",
                            "timestamp": datetime.now().isoformat(),
                            "isaac_connected": self.isaac_connected
                        }))
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON: {message}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {websocket.remote_address}")
        finally:
            self.connected_clients.discard(websocket)
    
    async def start_websocket_server(self):
        """Start WebSocket server"""
        logger.info(f"🔌 Starting Isaac Sim WebRTC WebSocket server on port {self.websocket_port}")
        
        server = await websockets.serve(self.handle_websocket_client, "0.0.0.0", self.websocket_port)
        logger.info(f"✅ Isaac Sim WebRTC WebSocket server running on port {self.websocket_port}")
        
        # Start video stream
        asyncio.create_task(self.generate_video_stream())
        
        return server

async def main():
    client = IsaacWebRTCClient()
    
    try:
        # Start WebSocket server
        server = await client.start_websocket_server()
        
        # Attempt to connect to Isaac Sim WebRTC
        asyncio.create_task(client.connect_to_isaac_webrtc())
        
        logger.info("🎉 Isaac Sim WebRTC Client started!")
        logger.info("📡 WebSocket server running on port 8765")
        logger.info("🎬 Attempting to connect to Isaac Sim WebRTC livestream")
        
        # Keep running
        await server.wait_closed()
        
    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
        client.running = False
        if client.pc:
            await client.pc.close()
    except Exception as e:
        logger.error(f"❌ Service error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
