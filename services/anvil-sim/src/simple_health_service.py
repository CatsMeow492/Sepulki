#!/usr/bin/env python3
"""
Simple Health Service - Minimal Isaac Sim integration for MVP
Provides basic health checks and WebSocket connections without complex dependencies.
"""

import asyncio
import json
import logging
from datetime import datetime
from typing import Dict, Any

import aiohttp
from aiohttp import web
import websockets
from websockets.server import serve
import structlog

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = structlog.get_logger(__name__)

class SimpleAnvilService:
    """Minimal Anvil Sim service for MVP demonstration."""
    
    def __init__(self):
        self.active_sessions: Dict[str, Dict[str, Any]] = {}
        self.websocket_clients: Dict[str, Any] = {}
        
    async def health_check(self, request):
        """Health check endpoint."""
        return web.json_response({
            'status': 'healthy',
            'service': 'anvil-sim',
            'version': '1.0.0-mvp',
            'isaac_sim_available': False,  # Will be True when real Isaac Sim is integrated
            'timestamp': datetime.utcnow().isoformat(),
            'active_sessions': len(self.active_sessions),
            'websocket_connections': len(self.websocket_clients)
        })
    
    async def create_session(self, request):
        """Create a new simulation session."""
        try:
            data = await request.json()
            
            session_id = f"session_{int(datetime.utcnow().timestamp())}_{len(self.active_sessions)}"
            
            session = {
                'id': session_id,
                'user_id': data.get('user_id', 'anonymous'),
                'sepulka_id': data.get('sepulka_id', 'demo-robot'),
                'environment': data.get('environment', 'warehouse'),
                'quality_profile': data.get('quality_profile', 'engineering'),
                'status': 'ready',
                'created_at': datetime.utcnow().isoformat(),
                'isaac_sim_mode': False,  # Simulation mode for MVP
                'urdf_content': data.get('urdf_content', '')
            }
            
            self.active_sessions[session_id] = session
            
            logger.info("Session created", session_id=session_id, 
                       user_id=session['user_id'])
            
            return web.json_response({
                'success': True,
                'session_id': session_id,
                'status': session['status'],
                'message': 'Session created successfully (simulation mode)',
                'isaac_sim_available': False
            })
            
        except Exception as e:
            logger.error("Failed to create session", error=str(e))
            return web.json_response({
                'success': False,
                'error': str(e),
                'message': 'Failed to create session'
            }, status=400)
    
    async def get_session_info(self, request):
        """Get session information."""
        session_id = request.match_info.get('session_id')
        
        if session_id not in self.active_sessions:
            return web.json_response({
                'success': False,
                'error': 'Session not found'
            }, status=404)
        
        session = self.active_sessions[session_id]
        return web.json_response({
            'success': True,
            'session': session
        })
    
    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections for real-time communication."""
        client_id = f"client_{len(self.websocket_clients)}"
        self.websocket_clients[client_id] = {
            'websocket': websocket,
            'connected_at': datetime.utcnow(),
            'session_id': None
        }
        
        logger.info("WebSocket client connected", client_id=client_id)
        
        try:
            # Wait for messages from client
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_websocket_message(client_id, data)
                except json.JSONDecodeError as e:
                    logger.warning("Invalid JSON from client", client_id=client_id, error=str(e))
                    try:
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'message': 'Invalid JSON format'
                        }))
                    except:
                        break  # Connection lost
                except Exception as e:
                    logger.error("WebSocket message error", client_id=client_id, error=str(e))
                    try:
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'message': str(e)
                        }))
                    except:
                        break  # Connection lost
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("WebSocket client disconnected", client_id=client_id)
        except Exception as e:
            logger.error("WebSocket connection error", client_id=client_id, error=str(e))
        finally:
            if client_id in self.websocket_clients:
                del self.websocket_clients[client_id]
                logger.info("WebSocket client cleaned up", client_id=client_id)
    
    async def handle_websocket_message(self, client_id: str, data: Dict[str, Any]):
        """Handle incoming WebSocket messages."""
        message_type = data.get('type')
        client = self.websocket_clients.get(client_id)
        
        if not client:
            logger.warning("Message from unknown client", client_id=client_id)
            return
        
        websocket = client['websocket']
        
        logger.info("Handling WebSocket message", 
                   client_id=client_id, message_type=message_type)
        
        try:
            if message_type == 'join_session':
                session_id = data.get('session_id', '')
                user_id = data.get('user_id', 'anonymous') 
                quality_profile = data.get('quality_profile', 'engineering')
                
                client['session_id'] = session_id
                client['user_id'] = user_id
                client['quality_profile'] = quality_profile
                
                # Send connection confirmation
                response = {
                    'type': 'connection_established',
                    'client_id': client_id,
                    'session_id': session_id,
                    'user_id': user_id,
                    'quality_profile': quality_profile,
                    'isaac_sim_available': False,  # Simulation mode for MVP
                    'message': 'Connected to Isaac Sim simulation mode'
                }
                
                await websocket.send(json.dumps(response))
                logger.info("Client joined session", 
                           client_id=client_id, session_id=session_id)
                
            elif message_type == 'offer':
                # Handle WebRTC offer (for future Isaac Sim video streaming)
                await websocket.send(json.dumps({
                    'type': 'answer',
                    'sdp': 'mock_answer_sdp_for_simulation_mode',
                    'message': 'WebRTC answer (simulation mode)'
                }))
                logger.info("WebRTC offer handled", client_id=client_id)
                
            elif message_type == 'ice_candidate':
                # Handle ICE candidate (acknowledge but no action needed in simulation mode)
                logger.debug("ICE candidate received", client_id=client_id)
                
            elif message_type == 'joint_control':
                # Mock joint control response
                joint_states = data.get('joint_states', {})
                response = {
                    'type': 'joint_update_response',
                    'joint_states': joint_states,
                    'timestamp': datetime.utcnow().isoformat(),
                    'status': 'updated'
                }
                await websocket.send(json.dumps(response))
                logger.debug("Joint control handled", 
                           client_id=client_id, joint_count=len(joint_states))
                
            elif message_type == 'camera_control':
                # Mock camera control response
                camera_data = {
                    'position': data.get('position', [4, 4, 4]),
                    'target': data.get('target', [0, 0, 0]), 
                    'fov': data.get('fov', 50)
                }
                response = {
                    'type': 'camera_update_response',
                    'camera': camera_data,
                    'timestamp': datetime.utcnow().isoformat(),
                    'status': 'updated'
                }
                await websocket.send(json.dumps(response))
                logger.debug("Camera control handled", client_id=client_id)
                
            elif message_type == 'request_keyframe':
                # Mock keyframe response
                response = {
                    'type': 'keyframe_response',
                    'status': 'completed',
                    'timestamp': datetime.utcnow().isoformat(),
                    'message': 'Keyframe generated (simulation mode)'
                }
                await websocket.send(json.dumps(response))
                logger.debug("Keyframe request handled", client_id=client_id)
                
            else:
                logger.warning("Unknown message type", 
                             client_id=client_id, message_type=message_type)
                await websocket.send(json.dumps({
                    'type': 'error',
                    'message': f'Unknown message type: {message_type}'
                }))
        
        except Exception as e:
            logger.error("Error handling WebSocket message", 
                        client_id=client_id, message_type=message_type, error=str(e))
            try:
                await websocket.send(json.dumps({
                    'type': 'error',
                    'message': f'Message handling failed: {str(e)}'
                }))
            except:
                # Connection probably lost
                pass

async def create_app():
    """Create the web application."""
    service = SimpleAnvilService()
    
    app = web.Application()
    
    # Health check endpoint
    app.router.add_get('/health', service.health_check)
    
    # Session management endpoints
    app.router.add_post('/create_scene', service.create_session)
    app.router.add_get('/session/{session_id}', service.get_session_info)
    
    # CORS middleware for development
    @web.middleware
    async def cors_handler(request, handler):
        if request.method == 'OPTIONS':
            response = web.Response()
        else:
            response = await handler(request)
        
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
        return response
    
    app.middlewares.append(cors_handler)
    
    return app, service

async def main():
    """Main service entry point."""
    logger.info("Starting Simple Anvil Sim Service (MVP)")
    
    # Create web app
    app, service = await create_app()
    
    # Start HTTP server
    runner = aiohttp.web.AppRunner(app)
    await runner.setup()
    site = aiohttp.web.TCPSite(runner, '0.0.0.0', 8002)  # Health check port
    await site.start()
    
    logger.info("HTTP server started on port 8002")
    
    # Start WebSocket server
    websocket_server = await serve(service.websocket_handler, "0.0.0.0", 8001)
    logger.info("WebSocket server started on port 8001")
    
    logger.info("Simple Anvil Sim Service ready!")
    logger.info("Health check: http://localhost:8002/health")
    logger.info("WebSocket: ws://localhost:8001")
    
    # Keep running
    try:
        await asyncio.Future()  # Run forever
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        websocket_server.close()
        await websocket_server.wait_closed()
        await runner.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
