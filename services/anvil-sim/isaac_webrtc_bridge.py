#!/usr/bin/env python3
"""
Isaac Sim WebRTC Bridge Service
Provides HTTP API endpoints that bridge to Isaac Sim's WebRTC streaming
"""

import asyncio
import json
import logging
import websockets
import aiohttp
from aiohttp import web, WSMsgType
import uuid
from datetime import datetime
import ssl

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class IsaacWebRTCBridge:
    def __init__(self, isaac_host="localhost", isaac_port=49100, bridge_port=8002):
        self.isaac_host = isaac_host
        self.isaac_port = isaac_port
        self.bridge_port = bridge_port
        self.sessions = {}
        self.websocket_connections = {}
        
    async def health_check(self, request):
        """Health check endpoint"""
        return web.json_response({
            "status": "healthy",
            "service": "isaac-webrtc-bridge",
            "timestamp": datetime.now().isoformat(),
            "isaac_sim": f"{self.isaac_host}:{self.isaac_port}"
        })
    
    async def create_scene(self, request):
        """Create a new Isaac Sim scene session"""
        try:
            data = await request.json()
            session_id = str(uuid.uuid4())
            
            # Store session data
            self.sessions[session_id] = {
                "session_id": session_id,
                "user_id": data.get("user_id", "anonymous"),
                "environment": data.get("environment", "warehouse"),
                "quality_profile": data.get("quality_profile", "engineering"),
                "robot_config": data.get("isaac_sim_robot"),
                "created_at": datetime.now().isoformat(),
                "status": "created"
            }
            
            logger.info(f"✅ Created Isaac Sim session: {session_id}")
            
            return web.json_response({
                "session_id": session_id,
                "status": "created",
                "webrtc_endpoint": f"ws://{self.isaac_host}:{self.isaac_port}",
                "message": "Session created successfully"
            })
            
        except Exception as e:
            logger.error(f"❌ Error creating scene: {e}")
            return web.json_response({
                "error": str(e),
                "status": "error"
            }, status=500)
    
    async def websocket_handler(self, request):
        """WebSocket handler for WebRTC signaling"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        session_id = None
        isaac_ws = None
        
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        logger.info(f"📨 Received WebSocket message: {data.get('type', 'unknown')}")
                        
                        if data.get("type") == "join_session":
                            session_id = data.get("session_id")
                            if session_id in self.sessions:
                                # Connect to Isaac Sim WebRTC
                                isaac_ws = await websockets.connect(
                                    f"ws://{self.isaac_host}:{self.isaac_port}",
                                    ping_interval=20,
                                    ping_timeout=10
                                )
                                self.websocket_connections[session_id] = isaac_ws
                                
                                # Forward messages to Isaac Sim
                                asyncio.create_task(self.forward_to_isaac(isaac_ws, ws, session_id))
                                
                                await ws.send_str(json.dumps({
                                    "type": "session_joined",
                                    "session_id": session_id,
                                    "status": "connected"
                                }))
                            else:
                                await ws.send_str(json.dumps({
                                    "type": "error",
                                    "message": "Invalid session ID"
                                }))
                        
                        elif data.get("type") in ["webrtc_offer", "ice_candidate"]:
                            # Forward WebRTC signaling to Isaac Sim
                            if isaac_ws:
                                await isaac_ws.send(json.dumps(data))
                            else:
                                await ws.send_str(json.dumps({
                                    "type": "error",
                                    "message": "Not connected to Isaac Sim"
                                }))
                        
                        else:
                            # Forward other messages to Isaac Sim
                            if isaac_ws:
                                await isaac_ws.send(json.dumps(data))
                                
                    except json.JSONDecodeError:
                        await ws.send_str(json.dumps({
                            "type": "error",
                            "message": "Invalid JSON"
                        }))
                    except Exception as e:
                        logger.error(f"❌ WebSocket error: {e}")
                        await ws.send_str(json.dumps({
                            "type": "error",
                            "message": str(e)
                        }))
                        
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"❌ WebSocket error: {ws.exception()}")
                    
        except Exception as e:
            logger.error(f"❌ WebSocket handler error: {e}")
        finally:
            if session_id and session_id in self.websocket_connections:
                del self.websocket_connections[session_id]
            if isaac_ws:
                await isaac_ws.close()
            logger.info(f"🔌 WebSocket connection closed for session: {session_id}")
        
        return ws
    
    async def forward_to_isaac(self, isaac_ws, client_ws, session_id):
        """Forward messages from Isaac Sim to client"""
        try:
            async for message in isaac_ws:
                try:
                    data = json.loads(message)
                    logger.info(f"📤 Forwarding from Isaac Sim: {data.get('type', 'unknown')}")
                    await client_ws.send_str(json.dumps(data))
                except json.JSONDecodeError:
                    # Forward raw message if not JSON
                    await client_ws.send_str(message)
                except Exception as e:
                    logger.error(f"❌ Error forwarding message: {e}")
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"🔌 Isaac Sim WebSocket closed for session: {session_id}")
        except Exception as e:
            logger.error(f"❌ Error in forward_to_isaac: {e}")
    
    async def get_session_status(self, request):
        """Get session status"""
        session_id = request.match_info.get('session_id')
        if session_id in self.sessions:
            return web.json_response(self.sessions[session_id])
        else:
            return web.json_response({
                "error": "Session not found"
            }, status=404)
    
    async def start_server(self):
        """Start the bridge server"""
        app = web.Application()
        
        # Add CORS middleware
        async def cors_handler(request, handler):
            response = await handler(request)
            response.headers['Access-Control-Allow-Origin'] = '*'
            response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
            response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
            return response
        
        app.middlewares.append(cors_handler)
        
        # Routes
        app.router.add_get('/health', self.health_check)
        app.router.add_post('/create_scene', self.create_scene)
        app.router.add_get('/session/{session_id}', self.get_session_status)
        app.router.add_get('/ws', self.websocket_handler)
        
        logger.info(f"🚀 Starting Isaac Sim WebRTC Bridge on port {self.bridge_port}")
        logger.info(f"🔗 Isaac Sim endpoint: {self.isaac_host}:{self.isaac_port}")
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self.bridge_port)
        await site.start()
        
        logger.info(f"✅ Isaac Sim WebRTC Bridge started successfully")
        
        # Keep the server running
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("🛑 Shutting down Isaac Sim WebRTC Bridge")
        finally:
            await runner.cleanup()

async def main():
    bridge = IsaacWebRTCBridge(
        isaac_host="localhost",
        isaac_port=49100,
        bridge_port=8002
    )
    await bridge.start_server()

if __name__ == "__main__":
    asyncio.run(main())
