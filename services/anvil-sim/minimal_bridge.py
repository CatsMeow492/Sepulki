#!/usr/bin/env python3
"""
Minimal Isaac Sim Bridge Service
"""

import asyncio
import json
import logging
import websockets
from aiohttp import web
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

async def health_check(request):
    """Health check endpoint"""
    logger.info("Health check requested")
    return web.json_response({
        "status": "healthy",
        "service": "minimal-isaac-bridge",
        "timestamp": datetime.now().isoformat()
    })

async def create_scene(request):
    """Create scene endpoint"""
    try:
        data = await request.json()
        session_id = data.get("session_id", "test-session-123")
        user_id = data.get("user_id", "anonymous")
        environment = data.get("environment", "warehouse")
        quality_profile = data.get("quality_profile", "engineering")
        robot_config = data.get("isaac_sim_robot", {})
        
        logger.info(f"Create scene requested: {data}")
        logger.info(f"Session: {session_id}, User: {user_id}, Environment: {environment}")
        
        return web.json_response({
            "session_id": session_id,
            "status": "created",
            "message": "Scene created successfully",
            "environment": environment,
            "quality_profile": quality_profile,
            "robot_config": robot_config,
            "timestamp": datetime.now().isoformat()
        })
    except Exception as e:
        logger.error(f"Error: {e}")
        return web.json_response({"error": str(e)}, status=500)

async def check_webrtc_client(request):
    """Check if Isaac Sim WebRTC Browser Client is available"""
    try:
        import aiohttp
        async with aiohttp.ClientSession() as session:
            try:
                async with session.get("http://localhost:8211/streaming/webrtc-client", timeout=2) as response:
                    if response.status == 200:
                        return web.json_response({
                            "available": True,
                            "url": "http://216.81.248.164:8211/streaming/webrtc-client?server=216.81.248.164",
                            "status": "ready"
                        })
                    else:
                        return web.json_response({
                            "available": False,
                            "status": f"HTTP {response.status}"
                        })
            except Exception as e:
                return web.json_response({
                    "available": False,
                    "status": "not_running",
                    "error": str(e)
                })
    except Exception as e:
        return web.json_response({
            "available": False,
            "status": "error",
            "error": str(e)
        })

async def websocket_handler(request):
    """WebSocket handler that connects to Isaac Sim WebRTC"""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    logger.info("WebSocket connection established")
    
    # Connect to Isaac Sim WebRTC
    isaac_ws = None
    try:
        isaac_ws = await websockets.connect(
            "ws://localhost:49100",
            ping_interval=20,
            ping_timeout=10
        )
        logger.info("Connected to Isaac Sim WebRTC")
        
        # Forward messages between client and Isaac Sim
        async def forward_to_isaac():
            try:
                async for msg in ws:
                    if msg.type == web.WSMsgType.TEXT:
                        data = json.loads(msg.data)
                        logger.info(f"Forwarding to Isaac Sim: {data.get('type', 'unknown')}")
                        await isaac_ws.send(json.dumps(data))
                    elif msg.type == web.WSMsgType.ERROR:
                        logger.error(f"WebSocket error: {ws.exception()}")
            except Exception as e:
                logger.error(f"Error forwarding to Isaac Sim: {e}")
        
        async def forward_from_isaac():
            try:
                async for message in isaac_ws:
                    data = json.loads(message)
                    logger.info(f"Forwarding from Isaac Sim: {data.get('type', 'unknown')}")
                    await ws.send_str(json.dumps(data))
            except Exception as e:
                logger.error(f"Error forwarding from Isaac Sim: {e}")
        
        # Run both forwarding tasks concurrently
        await asyncio.gather(
            forward_to_isaac(),
            forward_from_isaac()
        )
        
    except Exception as e:
        logger.error(f"WebSocket handler error: {e}")
        # Fallback: echo messages if Isaac Sim is not available
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    logger.info(f"Echoing (Isaac Sim unavailable): {data}")
                    await ws.send_str(json.dumps({
                        "type": "echo",
                        "data": data,
                        "timestamp": datetime.now().isoformat(),
                        "note": "Isaac Sim WebRTC not available"
                    }))
        except Exception as echo_error:
            logger.error(f"Echo error: {echo_error}")
    finally:
        if isaac_ws:
            await isaac_ws.close()
        logger.info("WebSocket connection closed")
    
    return ws

async def init_app():
    """Initialize the app"""
    app = web.Application()
    
    # Add CORS middleware
    @web.middleware
    async def cors_handler(request, handler):
        # Handle preflight requests
        if request.method == 'OPTIONS':
            response = web.Response()
        else:
            response = await handler(request)
        
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
        response.headers['Access-Control-Max-Age'] = '86400'
        return response
    
    app.middlewares.append(cors_handler)
    
    # Routes
    app.router.add_get('/health', health_check)
    app.router.add_post('/create_scene', create_scene)
    app.router.add_get('/check_webrtc_client', check_webrtc_client)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/', websocket_handler)  # Root WebSocket endpoint
    
    return app

async def main():
    app = await init_app()
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8002)
    await site.start()
    
    logger.info("Minimal bridge started on port 8002")
    
    try:
        await asyncio.Future()  # Run forever
    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        await runner.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
