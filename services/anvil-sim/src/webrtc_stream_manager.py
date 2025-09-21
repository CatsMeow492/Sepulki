#!/usr/bin/env python3
"""
WebRTC Stream Manager - Handles video streaming from Isaac Sim to browser clients
Provides WebRTC-based streaming with adaptive quality and multi-user support.
"""

import asyncio
import json
import logging
import time
import uuid
from typing import Dict, Optional, List, Any
from dataclasses import dataclass, asdict
from datetime import datetime

import structlog
import websockets
from websockets.server import WebSocketServerProtocol

# Import video frame generator
from video_frame_generator import get_video_generator

# Real aiortc for video streaming
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel, VideoStreamTrack, RTCConfiguration, RTCIceServer
    from aiortc.contrib.media import MediaPlayer, MediaRelay
    import av
    import numpy as np
    WEBRTC_AVAILABLE = True
except ImportError:
    WEBRTC_AVAILABLE = False
    # Mock classes for development
    class RTCPeerConnection:
        def __init__(self, config=None): pass
        async def createOffer(self): return {"sdp": "mock_offer", "type": "offer"}
        async def createAnswer(self): return {"sdp": "mock_answer", "type": "answer"}
        async def setLocalDescription(self, desc): pass
        async def setRemoteDescription(self, desc): pass
        def close(self): pass
        def addTrack(self, track): pass
    
    class RTCSessionDescription:
        def __init__(self, sdp, type): 
            self.sdp = sdp
            self.type = type
            
    class VideoStreamTrack:
        def __init__(self): pass

logger = structlog.get_logger(__name__)

class IsaacSimVideoTrack(VideoStreamTrack):
    """Custom video track that generates Isaac Sim frames."""
    
    def __init__(self, client_id: str):
        super().__init__()
        self.client_id = client_id
        self.video_generator = get_video_generator()
        
    async def recv(self):
        """Generate next video frame."""
        if not WEBRTC_AVAILABLE:
            await asyncio.sleep(1/30)  # 30 FPS
            return None
            
        try:
            # Generate frame from Isaac Sim video generator  
            frame_data = self.video_generator.generate_frame()
            
            # Convert numpy array to av.VideoFrame
            frame = av.VideoFrame.from_ndarray(frame_data, format="bgr24")
            frame.pts = int(time.time() * 90000)  # 90kHz timebase
            frame.time_base = av.Rational(1, 90000)
            
            return frame
            
        except Exception as e:
            logger.error("Video frame generation failed", 
                        client_id=self.client_id, error=str(e))
            await asyncio.sleep(1/30)
            return None

@dataclass
class StreamClient:
    """Represents a connected streaming client."""
    id: str
    user_id: str
    session_id: str
    websocket: WebSocketServerProtocol
    peer_connection: RTCPeerConnection
    quality_profile: str = "engineering"
    connected_at: datetime = None
    last_activity: datetime = None
    
    def __post_init__(self):
        if self.connected_at is None:
            self.connected_at = datetime.utcnow()
        if self.last_activity is None:
            self.last_activity = datetime.utcnow()

@dataclass 
class StreamMetrics:
    """Streaming performance metrics."""
    session_id: str
    client_count: int
    average_fps: float
    average_bitrate: float
    frame_drops: int
    latency_ms: float
    bandwidth_usage: float
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.utcnow()

class WebRTCStreamManager:
    """
    Manages WebRTC streaming connections between Isaac Sim and browser clients.
    Handles multiple simultaneous streams with adaptive quality.
    """
    
    def __init__(self, isaac_sim_manager=None):
        self.isaac_sim_manager = isaac_sim_manager
        self.clients: Dict[str, StreamClient] = {}
        self.session_streams: Dict[str, List[str]] = {}  # session_id -> client_ids
        self.websocket_server = None
        self.running = False
        
        # STUN/TURN servers for WebRTC connectivity  
        if WEBRTC_AVAILABLE:
            self.rtc_config = RTCConfiguration(
                iceServers=[
                    RTCIceServer(urls="stun:stun.l.google.com:19302"),
                    RTCIceServer(urls="stun:stun1.l.google.com:19302"),
                    # Add TURN servers for production
                ]
            )
        else:
            self.rtc_config = None
        
        # Quality profiles for adaptive streaming
        self.quality_profiles = {
            "demo": {
                "width": 1280,
                "height": 720,
                "fps": 30,
                "bitrate": 2000000,  # 2 Mbps
                "codec": "H264"
            },
            "engineering": {
                "width": 1920,
                "height": 1080, 
                "fps": 60,
                "bitrate": 5000000,  # 5 Mbps
                "codec": "H264"
            },
            "certification": {
                "width": 3840,
                "height": 2160,
                "fps": 60, 
                "bitrate": 15000000,  # 15 Mbps
                "codec": "H265"
            }
        }
    
    async def start_server(self, host: str = "localhost", port: int = 8001):
        """Start the WebSocket server for WebRTC signaling."""
        try:
            self.websocket_server = await websockets.serve(
                self._handle_websocket_connection,
                host,
                port,
                ping_interval=20,
                ping_timeout=10
            )
            
            self.running = True
            logger.info("WebRTC streaming server started", 
                       host=host, port=port,
                       webrtc_available=WEBRTC_AVAILABLE)
                       
        except Exception as e:
            logger.error("Failed to start WebRTC streaming server", error=str(e))
            raise
    
    async def stop_server(self):
        """Stop the WebSocket server and cleanup connections."""
        self.running = False
        
        # Close all client connections
        client_ids = list(self.clients.keys())
        for client_id in client_ids:
            await self._disconnect_client(client_id)
        
        # Stop WebSocket server
        if self.websocket_server:
            self.websocket_server.close()
            await self.websocket_server.wait_closed()
            
        logger.info("WebRTC streaming server stopped")
    
    async def _handle_websocket_connection(self, websocket: WebSocketServerProtocol, path: str):
        """Handle new WebSocket connections for WebRTC signaling."""
        client_id = str(uuid.uuid4())
        logger.info("New WebSocket connection", client_id=client_id, path=path)
        
        try:
            # Wait for initial client info
            message = await websocket.recv()
            data = json.loads(message)
            
            if data.get("type") != "join_session":
                await websocket.close(code=1008, reason="Invalid initial message")
                return
            
            # Create stream client
            client = StreamClient(
                id=client_id,
                user_id=data.get("user_id", "anonymous"),
                session_id=data.get("session_id", ""),
                websocket=websocket,
                peer_connection=RTCPeerConnection(self.rtc_config),
                quality_profile=data.get("quality_profile", "engineering")
            )
            
            self.clients[client_id] = client
            
            # Add to session streams
            session_id = client.session_id
            if session_id not in self.session_streams:
                self.session_streams[session_id] = []
            self.session_streams[session_id].append(client_id)
            
            # Send connection confirmation
            await self._send_to_client(client_id, {
                "type": "connection_established",
                "client_id": client_id,
                "session_id": session_id,
                "quality_profile": client.quality_profile
            })
            
            logger.info("Stream client connected", 
                       client_id=client_id,
                       session_id=session_id,
                       quality_profile=client.quality_profile)
            
            # Handle client messages
            async for message in websocket:
                try:
                    await self._handle_client_message(client_id, json.loads(message))
                except json.JSONDecodeError:
                    logger.warning("Invalid JSON from client", client_id=client_id)
                except Exception as e:
                    logger.error("Error handling client message", 
                               client_id=client_id, error=str(e))
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("Client disconnected", client_id=client_id)
        except Exception as e:
            logger.error("WebSocket connection error", 
                        client_id=client_id, error=str(e))
        finally:
            if client_id in self.clients:
                await self._disconnect_client(client_id)
    
    async def _handle_client_message(self, client_id: str, data: Dict[str, Any]):
        """Handle messages from streaming clients."""
        client = self.clients.get(client_id)
        if not client:
            return
        
        message_type = data.get("type")
        client.last_activity = datetime.utcnow()
        
        logger.debug("Received client message", 
                    client_id=client_id, message_type=message_type)
        
        if message_type == "offer":
            await self._handle_webrtc_offer(client_id, data)
            
        elif message_type == "answer":
            await self._handle_webrtc_answer(client_id, data)
            
        elif message_type == "ice_candidate":
            await self._handle_ice_candidate(client_id, data)
            
        elif message_type == "update_quality":
            await self._update_quality_profile(client_id, data.get("quality_profile"))
            
        elif message_type == "request_keyframe":
            await self._request_keyframe(client_id)
            
        elif message_type == "camera_control":
            await self._handle_camera_control(client_id, data)
            
        elif message_type == "joint_control":
            await self._handle_joint_control(client_id, data)
            
        else:
            logger.warning("Unknown message type", 
                          client_id=client_id, message_type=message_type)
    
    async def _handle_webrtc_offer(self, client_id: str, data: Dict[str, Any]):
        """Handle WebRTC offer from client."""
        client = self.clients.get(client_id)
        if not client:
            return
        
        try:
            # Set remote description (client's offer)
            sdp = data.get("sdp")
            if not sdp:
                logger.warning("No SDP in offer", client_id=client_id)
                return
                
            if WEBRTC_AVAILABLE:
                remote_desc = RTCSessionDescription(sdp, "offer")
                await client.peer_connection.setRemoteDescription(remote_desc)
                
                # Add Isaac Sim video track
                video_track = IsaacSimVideoTrack(client_id)
                client.peer_connection.addTrack(video_track)
                
                logger.info("Added Isaac Sim video track", client_id=client_id)
                
                # Create answer
                answer = await client.peer_connection.createAnswer()
                await client.peer_connection.setLocalDescription(answer)
                
                # Send answer back to client
                await self._send_to_client(client_id, {
                    "type": "answer",
                    "sdp": answer.sdp
                })
                
                logger.info("Real WebRTC video streaming started", client_id=client_id)
            else:
                # Mock answer for development
                await self._send_to_client(client_id, {
                    "type": "answer",
                    "sdp": "mock_answer_sdp"
                })
            
            logger.info("WebRTC offer handled", client_id=client_id)
            
        except Exception as e:
            logger.error("Failed to handle WebRTC offer", 
                        client_id=client_id, error=str(e))
    
    async def _handle_webrtc_answer(self, client_id: str, data: Dict[str, Any]):
        """Handle WebRTC answer from client."""
        client = self.clients.get(client_id)
        if not client:
            return
        
        try:
            sdp = data.get("sdp")
            if not sdp:
                return
                
            if WEBRTC_AVAILABLE:
                remote_desc = RTCSessionDescription(sdp, "answer")
                await client.peer_connection.setRemoteDescription(remote_desc)
            
            logger.info("WebRTC answer handled", client_id=client_id)
            
        except Exception as e:
            logger.error("Failed to handle WebRTC answer", 
                        client_id=client_id, error=str(e))
    
    async def _handle_ice_candidate(self, client_id: str, data: Dict[str, Any]):
        """Handle ICE candidate from client."""
        # In real implementation, would add ICE candidate to peer connection
        logger.debug("ICE candidate received", client_id=client_id)
    
    async def _update_quality_profile(self, client_id: str, quality_profile: str):
        """Update streaming quality profile for client."""
        client = self.clients.get(client_id)
        if not client or quality_profile not in self.quality_profiles:
            return
        
        client.quality_profile = quality_profile
        
        # Notify about quality change
        await self._send_to_client(client_id, {
            "type": "quality_updated",
            "quality_profile": quality_profile,
            "settings": self.quality_profiles[quality_profile]
        })
        
        logger.info("Quality profile updated", 
                   client_id=client_id, quality_profile=quality_profile)
    
    async def _request_keyframe(self, client_id: str):
        """Request keyframe for improved streaming quality."""
        # In real implementation, would trigger keyframe in Isaac Sim
        logger.debug("Keyframe requested", client_id=client_id)
    
    async def _handle_camera_control(self, client_id: str, data: Dict[str, Any]):
        """Handle camera control commands."""
        client = self.clients.get(client_id)
        if not client:
            return
        
        camera_data = {
            "position": data.get("position", [4, 4, 4]),
            "target": data.get("target", [0, 0, 0]),
            "fov": data.get("fov", 50)
        }
        
        # Update video generator with new camera position
        get_video_generator().update_camera(
            camera_data["position"],
            camera_data["target"], 
            camera_data["fov"]
        )
        
        # Forward to Isaac Sim Manager
        if self.isaac_sim_manager:
            # Would call isaac_sim_manager.update_camera(client.session_id, camera_data)
            pass
            
        # Send acknowledgment
        await self._send_to_client(client_id, {
            "type": "camera_update_response",
            "camera": camera_data,
            "timestamp": datetime.utcnow().isoformat(),
            "status": "updated"
        })
        
        logger.debug("Camera control handled", 
                    client_id=client_id, camera_data=camera_data)
    
    async def _handle_joint_control(self, client_id: str, data: Dict[str, Any]):
        """Handle robot joint control commands."""
        client = self.clients.get(client_id)
        if not client:
            return
        
        joint_states = data.get("joint_states", {})
        
        # Update video generator with new joint states
        get_video_generator().update_joints(joint_states)
        
        # Forward to Isaac Sim Manager
        if self.isaac_sim_manager:
            try:
                await self.isaac_sim_manager.update_joint_states(
                    client.session_id, joint_states
                )
            except Exception as e:
                logger.error("Failed to update joint states", 
                           client_id=client_id, error=str(e))
                           
        # Send acknowledgment
        await self._send_to_client(client_id, {
            "type": "joint_update_response",
            "joint_states": joint_states,
            "timestamp": datetime.utcnow().isoformat(),
            "status": "updated"
        })
        
        logger.debug("Joint control handled", 
                    client_id=client_id, joint_count=len(joint_states))
    
    async def _send_to_client(self, client_id: str, message: Dict[str, Any]):
        """Send message to specific client."""
        client = self.clients.get(client_id)
        if not client or client.websocket.closed:
            return
        
        try:
            await client.websocket.send(json.dumps(message))
        except websockets.exceptions.ConnectionClosed:
            logger.debug("Client connection closed", client_id=client_id)
            await self._disconnect_client(client_id)
        except Exception as e:
            logger.error("Failed to send message to client", 
                        client_id=client_id, error=str(e))
    
    async def broadcast_to_session(self, session_id: str, message: Dict[str, Any]):
        """Broadcast message to all clients in a session."""
        client_ids = self.session_streams.get(session_id, [])
        
        for client_id in client_ids:
            await self._send_to_client(client_id, message)
    
    async def _disconnect_client(self, client_id: str):
        """Disconnect and cleanup client."""
        client = self.clients.get(client_id)
        if not client:
            return
        
        # Remove from session streams
        session_id = client.session_id
        if session_id in self.session_streams:
            try:
                self.session_streams[session_id].remove(client_id)
                if not self.session_streams[session_id]:
                    del self.session_streams[session_id]
            except ValueError:
                pass
        
        # Close peer connection
        try:
            client.peer_connection.close()
        except:
            pass
        
        # Close websocket
        try:
            await client.websocket.close()
        except:
            pass
        
        # Remove from clients
        del self.clients[client_id]
        
        logger.info("Client disconnected", client_id=client_id)
    
    def get_streaming_metrics(self) -> List[StreamMetrics]:
        """Get current streaming performance metrics."""
        metrics = []
        
        # Group clients by session
        session_clients: Dict[str, List[StreamClient]] = {}
        for client in self.clients.values():
            session_id = client.session_id
            if session_id not in session_clients:
                session_clients[session_id] = []
            session_clients[session_id].append(client)
        
        # Generate metrics per session
        for session_id, clients in session_clients.items():
            metrics.append(StreamMetrics(
                session_id=session_id,
                client_count=len(clients),
                average_fps=60.0,  # Would get from actual streaming
                average_bitrate=5000000.0,
                frame_drops=0,
                latency_ms=150.0,
                bandwidth_usage=len(clients) * 5.0  # MB/s estimate
            ))
        
        return metrics
    
    def get_session_info(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get streaming info for a specific session."""
        client_ids = self.session_streams.get(session_id, [])
        if not client_ids:
            return None
        
        clients = [self.clients[cid] for cid in client_ids if cid in self.clients]
        
        return {
            "session_id": session_id,
            "client_count": len(clients),
            "clients": [
                {
                    "id": c.id,
                    "user_id": c.user_id,
                    "quality_profile": c.quality_profile,
                    "connected_at": c.connected_at.isoformat(),
                    "last_activity": c.last_activity.isoformat()
                }
                for c in clients
            ],
            "streaming_active": len(clients) > 0
        }

# Global instance
webrtc_stream_manager = WebRTCStreamManager()
