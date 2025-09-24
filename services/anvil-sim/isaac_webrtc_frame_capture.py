#!/usr/bin/env python3
"""
Isaac Sim WebRTC Frame Capture Service
Connects to Isaac Sim's WebRTC livestream and captures real frames
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
import ssl
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import BYE, add_signaling_arguments, create_signaling

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class IsaacWebRTCFrameCapture:
    def __init__(self, websocket_port=8765, isaac_host="localhost", isaac_port=49100):
        self.websocket_port = websocket_port
        self.isaac_host = isaac_host
        self.isaac_port = isaac_port
        self.connected_clients = set()
        self.frame_count = 0
        self.running = True
        self.webrtc_pc = None
        self.current_frame = None
        self.isaac_connected = False
        
    async def connect_to_isaac_webrtc(self):
        """Attempt to connect to Isaac Sim's WebRTC livestream"""
        try:
            logger.info(f"🔌 Attempting to connect to Isaac Sim WebRTC at {self.isaac_host}:{self.isaac_port}")
            
            # Create peer connection
            self.webrtc_pc = RTCPeerConnection()
            
            # Set up event handlers
            @self.webrtc_pc.on("track")
            def on_track(track):
                logger.info(f"📹 Received WebRTC track: {track.kind}")
                if track.kind == "video":
                    asyncio.create_task(self.process_webrtc_video_track(track))
            
            @self.webrtc_pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"🔗 WebRTC connection state: {self.webrtc_pc.connectionState}")
                if self.webrtc_pc.connectionState == "connected":
                    self.isaac_connected = True
                else:
                    self.isaac_connected = False
            
            # Try different WebRTC signaling approaches
            signaling_methods = [
                f"ws://{self.isaac_host}:{self.isaac_port}/webrtc",
                f"ws://{self.isaac_host}:{self.isaac_port}/signaling",
                f"ws://{self.isaac_host}:{self.isaac_port}/ws"
            ]
            
            for signaling_url in signaling_methods:
                try:
                    logger.info(f"🔄 Trying signaling URL: {signaling_url}")
                    
                    # Create offer
                    await self.webrtc_pc.setLocalDescription(await self.webrtc_pc.createOffer())
                    
                    async with websockets.connect(signaling_url) as ws:
                        # Send offer to Isaac Sim
                        offer_message = {
                            "type": "offer",
                            "sdp": self.webrtc_pc.localDescription.sdp
                        }
                        await ws.send(json.dumps(offer_message))
                        
                        # Wait for answer with timeout
                        try:
                            response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                            answer_data = json.loads(response)
                            
                            if answer_data.get("type") == "answer":
                                answer = RTCSessionDescription(
                                    sdp=answer_data["sdp"],
                                    type="answer"
                                )
                                await self.webrtc_pc.setRemoteDescription(answer)
                                logger.info("✅ WebRTC connection established with Isaac Sim")
                                return True
                        except asyncio.TimeoutError:
                            logger.warning(f"⏰ Timeout waiting for answer from {signaling_url}")
                            continue
                            
                except Exception as e:
                    logger.warning(f"❌ WebRTC signaling failed for {signaling_url}: {e}")
                    continue
            
            logger.error("❌ All WebRTC signaling methods failed")
            return False
                
        except Exception as e:
            logger.error(f"❌ WebRTC connection failed: {e}")
            return False
    
    async def process_webrtc_video_track(self, track):
        """Process video frames from Isaac Sim WebRTC stream"""
        logger.info("🎬 Starting WebRTC video frame processing...")
        
        try:
            while True:
                frame = await track.recv()
                
                # Convert frame to numpy array
                img = frame.to_ndarray(format="bgr24")
                
                # Store current frame
                self.current_frame = img
                self.frame_count += 1
                
                if self.frame_count % 30 == 0:
                    logger.info(f"📹 Processed {self.frame_count} real Isaac Sim frames")
                    
        except Exception as e:
            logger.error(f"❌ WebRTC video processing error: {e}")
    
    def create_enhanced_isaac_frame(self):
        """Create enhanced Isaac Sim frame (fallback when WebRTC not available)"""
        # Create 1920x1080 frame with realistic lighting
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        # Dynamic background with gradient and lighting effects
        for y in range(1080):
            for x in range(1920):
                # Base gradient
                base_color = int(25 + (y / 1080) * 15)
                
                # Add subtle noise for realism
                noise = np.random.randint(-5, 5)
                
                # Lighting effect from top-left
                distance_from_light = np.sqrt((x - 200)**2 + (y - 200)**2)
                light_factor = max(0, 1 - (distance_from_light / 800))
                
                final_color = int(base_color + (light_factor * 20) + noise)
                final_color = max(0, min(255, final_color))
                
                frame[y, x] = [final_color, final_color + 5, final_color + 10]
        
        # Isaac Sim HUD with realistic styling
        self.draw_isaac_hud(frame)
        
        # Enhanced robot visualization
        self.draw_enhanced_robot(frame)
        
        # Add particle effects
        self.draw_particle_effects(frame)
        
        return frame
    
    def draw_isaac_hud(self, frame):
        """Draw Isaac Sim HUD with realistic styling"""
        # Main HUD panel
        hud_rect = (10, 10, 450, 200)
        cv2.rectangle(frame, (hud_rect[0], hud_rect[1]), (hud_rect[2], hud_rect[3]), (0, 0, 0), -1)
        cv2.rectangle(frame, (hud_rect[0], hud_rect[1]), (hud_rect[2], hud_rect[3]), (0, 255, 0), 2)
        
        # HUD content
        cv2.putText(frame, "NVIDIA Isaac Sim", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "PHYSICS SIMULATION", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"Frame: {self.frame_count}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, "Robot: Franka Emika Panda", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Connection status
        if self.isaac_connected:
            cv2.putText(frame, "Status: REAL ISAAC SIM", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, "Source: WebRTC Stream", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(frame, "Status: ENHANCED MOCK", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, "Source: Simulated", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        cv2.putText(frame, "Physics: 240 Hz", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    def draw_enhanced_robot(self, frame):
        """Draw enhanced robot visualization"""
        center_x, center_y = 960, 540
        
        # Robot base with metallic appearance
        base_color = (120, 140, 160)
        cv2.circle(frame, (center_x, center_y), 60, base_color, -1)
        cv2.circle(frame, (center_x, center_y), 60, (200, 200, 200), 2)
        
        # Robot arm with realistic joint movement
        arm_length = 120
        current_angle = (self.frame_count * 0.5) % 360
        
        # Base joint
        joint1_x = center_x
        joint1_y = center_y
        
        # First arm segment
        angle1 = np.radians(current_angle)
        end1_x = int(joint1_x + arm_length * np.cos(angle1))
        end1_y = int(joint1_y + arm_length * np.sin(angle1))
        
        cv2.line(frame, (joint1_x, joint1_y), (end1_x, end1_y), (100, 150, 200), 12)
        cv2.circle(frame, (end1_x, end1_y), 20, (150, 170, 190), -1)
        cv2.circle(frame, (end1_x, end1_y), 20, (200, 200, 200), 2)
        
        # Second arm segment
        angle2 = np.radians(current_angle + 45)
        end2_x = int(end1_x + arm_length * 0.8 * np.cos(angle2))
        end2_y = int(end1_y + arm_length * 0.8 * np.sin(angle2))
        
        cv2.line(frame, (end1_x, end1_y), (end2_x, end2_y), (120, 140, 180), 10)
        cv2.circle(frame, (end2_x, end2_y), 15, (160, 180, 200), -1)
        cv2.circle(frame, (end2_x, end2_y), 15, (200, 200, 200), 2)
        
        # Gripper
        gripper_angle = np.radians(current_angle + 90)
        gripper_length = 30
        
        # Left gripper finger
        gripper_left_x = int(end2_x + gripper_length * np.cos(gripper_angle))
        gripper_left_y = int(end2_y + gripper_length * np.sin(gripper_angle))
        cv2.line(frame, (end2_x, end2_y), (gripper_left_x, gripper_left_y), (80, 100, 120), 6)
        
        # Right gripper finger
        gripper_right_x = int(end2_x + gripper_length * np.cos(gripper_angle + np.pi))
        gripper_right_y = int(end2_y + gripper_length * np.sin(gripper_angle + np.pi))
        cv2.line(frame, (end2_x, end2_y), (gripper_right_x, gripper_right_y), (80, 100, 120), 6)
    
    def draw_particle_effects(self, frame):
        """Add particle effects for realism"""
        # Dust particles
        for _ in range(20):
            x = np.random.randint(0, 1920)
            y = np.random.randint(0, 1080)
            size = np.random.randint(1, 3)
            color = np.random.randint(100, 150)
            cv2.circle(frame, (x, y), size, (color, color, color), -1)
        
        # Light reflections
        for _ in range(5):
            x = np.random.randint(100, 1820)
            y = np.random.randint(100, 980)
            size = np.random.randint(2, 5)
            cv2.circle(frame, (x, y), size, (255, 255, 255), -1)
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames"""
        logger.info("🎨 Starting Isaac Sim video stream...")
        
        while self.running:
            if self.connected_clients:
                try:
                    # Use real Isaac Sim frame if available, otherwise use enhanced mock
                    if self.current_frame is not None and self.isaac_connected:
                        frame = self.current_frame
                        frame_source = "isaac_sim_webrtc"
                    else:
                        frame = self.create_enhanced_isaac_frame()
                        frame_source = "enhanced_mock"
                    
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
    
    async def handle_websocket_client(self, websocket):
        """Handle WebSocket client connections"""
        logger.info(f"📱 New client connected: {websocket.remote_address}")
        self.connected_clients.add(websocket)
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "connection_established",
                "message": "Connected to Isaac Sim Frame Capture Service",
                "timestamp": datetime.now().isoformat(),
                "isaac_connected": self.isaac_connected,
                "stream_type": "webrtc_with_fallback"
            }))
            
            # Handle messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get("type") == "start_video_stream":
                        await websocket.send(json.dumps({
                            "type": "video_stream_started",
                            "message": "Isaac Sim frame capture started",
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
        logger.info(f"🔌 Starting Isaac Sim Frame Capture WebSocket server on port {self.websocket_port}")
        
        server = await websockets.serve(self.handle_websocket_client, "0.0.0.0", self.websocket_port)
        logger.info(f"✅ Isaac Sim Frame Capture WebSocket server running on port {self.websocket_port}")
        
        # Start video stream
        asyncio.create_task(self.generate_video_stream())
        
        return server

async def main():
    capture = IsaacWebRTCFrameCapture()
    
    try:
        # Start WebSocket server
        server = await capture.start_websocket_server()
        
        # Attempt to connect to Isaac Sim WebRTC
        asyncio.create_task(capture.connect_to_isaac_webrtc())
        
        logger.info("🎉 Isaac Sim Frame Capture service started!")
        logger.info("📡 WebSocket server running on port 8765")
        logger.info("🎬 Attempting to capture real Isaac Sim frames via WebRTC")
        logger.info("🔄 Fallback to enhanced mock content if WebRTC unavailable")
        
        # Keep running
        await server.wait_closed()
        
    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
        capture.running = False
        if capture.webrtc_pc:
            await capture.webrtc_pc.close()
    except Exception as e:
        logger.error(f"❌ Service error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
