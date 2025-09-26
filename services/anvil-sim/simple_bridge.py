#!/usr/bin/env python3
"""
Simple Isaac Sim Bridge Service
Provides basic HTTP endpoints for Isaac Sim WebRTC streaming
"""

import asyncio
import json
import logging
from aiohttp import web
from datetime import datetime
import uuid

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SimpleBridge:
    def __init__(self, port=8002):
        self.port = port
        self.sessions = {}
        
    async def health_check(self, request):
        """Health check endpoint"""
        logger.info("Health check requested")
        return web.json_response({
            "status": "healthy",
            "service": "simple-isaac-bridge",
            "timestamp": datetime.now().isoformat(),
            "isaac_sim": "localhost:49100"
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
                "created_at": datetime.now().isoformat(),
                "status": "created"
            }
            
            logger.info(f"Created session: {session_id}")
            
            return web.json_response({
                "session_id": session_id,
                "status": "created",
                "webrtc_endpoint": "ws://localhost:49100",
                "message": "Session created successfully"
            })
            
        except Exception as e:
            logger.error(f"Error creating scene: {e}")
            return web.json_response({
                "error": str(e),
                "status": "error"
            }, status=500)
    
    async def websocket_handler(self, request):
        """Simple WebSocket handler"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        logger.info("WebSocket connection established")
        
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    logger.info(f"Received: {data}")
                    
                    # Echo back the message
                    await ws.send_str(json.dumps({
                        "type": "echo",
                        "data": data,
                        "timestamp": datetime.now().isoformat()
                    }))
                    
                elif msg.type == web.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
                    
        except Exception as e:
            logger.error(f"WebSocket handler error: {e}")
        finally:
            logger.info("WebSocket connection closed")
        
        return ws
    
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
        app.router.add_get('/ws', self.websocket_handler)
        
        logger.info(f"Starting Simple Isaac Bridge on port {self.port}")
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        await site.start()
        
        logger.info(f"Simple Isaac Bridge started successfully")
        
        # Keep the server running
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down Simple Isaac Bridge")
        finally:
            await runner.cleanup()

async def main():
    bridge = SimpleBridge(port=8002)
    await bridge.start_server()

if __name__ == "__main__":
    asyncio.run(main())
