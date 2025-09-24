#!/usr/bin/env python3
"""
Isaac Sim WebRTC Client
Connects to Isaac Sim's WebRTC livestream and captures video frames
"""

import asyncio
import json
import logging
import cv2
import numpy as np
import base64
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import BYE, add_signaling_arguments, create_signaling
import websockets
from datetime import datetime

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class IsaacWebRTCClient:
    def __init__(self, isaac_host="localhost", isaac_port=49100):
        self.isaac_host = isaac_host
        self.isaac_port = isaac_port
        self.pc = None
        self.current_frame = None
        self.frame_count = 0
        self.websocket_clients = set()
        
    async def connect_to_isaac(self):
        """Connect to Isaac Sim WebRTC livestream"""
        logger.info(f"🔌 Connecting to Isaac Sim WebRTC at {self.isaac_host}:{self.isaac_port}")
        
        try:
            # Create peer connection
            self.pc = RTCPeerConnection()
            
            # Set up event handlers
            @self.pc.on("track")
            def on_track(track):
                logger.info(f"📹 Received track: {track.kind}")
                if track.kind == "video":
                    asyncio.create_task(self.process_video_track(track))
            
            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"🔗 Connection state: {self.pc.connectionState}")
            
            # Create offer
            await self.pc.setLocalDescription(await self.pc.createOffer())
            
            # Connect to Isaac Sim's WebRTC signaling
            # This is a simplified approach - Isaac Sim might use a different signaling method
            signaling_url = f"ws://{self.isaac_host}:{self.isaac_port}/webrtc"
            
            try:
                async with websockets.connect(signaling_url) as ws:
                    # Send offer to Isaac Sim
                    offer_message = {
                        "type": "offer",
                        "sdp": self.pc.localDescription.sdp
                    }
                    await ws.send(json.dumps(offer_message))
                    
                    # Wait for answer
                    response = await ws.recv()
                    answer_data = json.loads(response)
                    
                    if answer_data.get("type") == "answer":
                        answer = RTCSessionDescription(
                            sdp=answer_data["sdp"],
                            type="answer"
                        )
                        await self.pc.setRemoteDescription(answer)
                        logger.info("✅ WebRTC connection established with Isaac Sim")
                        return True
                    
            except Exception as e:
                logger.error(f"❌ WebRTC signaling failed: {e}")
                return False
                
        except Exception as e:
            logger.error(f"❌ WebRTC connection failed: {e}")
            return False
    
    async def process_video_track(self, track):
        """Process video frames from Isaac Sim"""
        logger.info("🎬 Starting video frame processing...")
        
        try:
            while True:
                frame = await track.recv()
                
                # Convert frame to numpy array
                img = frame.to_ndarray(format="bgr24")
                
                # Store current frame
                self.current_frame = img
                self.frame_count += 1
                
                # Send to WebSocket clients
                await self.broadcast_frame(img)
                
                if self.frame_count % 30 == 0:
                    logger.info(f"📹 Processed {self.frame_count} frames from Isaac Sim")
                    
        except Exception as e:
            logger.error(f"❌ Video processing error: {e}")
    
    async def broadcast_frame(self, frame):
        """Broadcast frame to WebSocket clients"""
        if not self.websocket_clients:
            return
            
        try:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            frame_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Create message
            message = json.dumps({
                "type": "video_frame",
                "frame_data": frame_base64,
                "frame_number": self.frame_count,
                "timestamp": datetime.now().isoformat(),
                "width": frame.shape[1],
                "height": frame.shape[0],
                "source": "isaac_sim_webrtc"
            })
            
            # Send to all clients
            disconnected = set()
            for client in self.websocket_clients:
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            
            # Remove disconnected clients
            self.websocket_clients -= disconnected
            
        except Exception as e:
            logger.error(f"❌ Frame broadcast error: {e}")
    
    async def start_websocket_server(self, port=8765):
        """Start WebSocket server for clients"""
        logger.info(f"🔌 Starting WebSocket server on port {port}")
        
        async def handle_client(websocket, path):
            logger.info(f"📱 New WebSocket client connected: {websocket.remote_address}")
            self.websocket_clients.add(websocket)
            
            try:
                # Send welcome message
                await websocket.send(json.dumps({
                    "type": "connection_established",
                    "message": "Connected to Isaac Sim WebRTC Bridge",
                    "timestamp": datetime.now().isoformat(),
                    "isaac_connected": self.pc is not None and self.pc.connectionState == "connected"
                }))
                
                # Handle messages
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        if data.get("type") == "start_video_stream":
                            await websocket.send(json.dumps({
                                "type": "video_stream_started",
                                "message": "Video stream started",
                                "timestamp": datetime.now().isoformat()
                            }))
                    except json.JSONDecodeError:
                        logger.error(f"Invalid JSON: {message}")
                        
            except websockets.exceptions.ConnectionClosed:
                logger.info(f"WebSocket client disconnected: {websocket.remote_address}")
            finally:
                self.websocket_clients.discard(websocket)
        
        server = await websockets.serve(handle_client, "0.0.0.0", port)
        logger.info(f"✅ WebSocket server running on port {port}")
        return server
    
    async def cleanup(self):
        """Cleanup resources"""
        if self.pc:
            await self.pc.close()

async def main():
    client = IsaacWebRTCClient()
    
    try:
        # Start WebSocket server
        server = await client.start_websocket_server()
        
        # Connect to Isaac Sim
        connected = await client.connect_to_isaac()
        
        if connected:
            logger.info("🎉 Isaac Sim WebRTC connection successful!")
            # Keep running
            await server.wait_closed()
        else:
            logger.error("❌ Failed to connect to Isaac Sim WebRTC")
            
    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
    finally:
        await client.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
