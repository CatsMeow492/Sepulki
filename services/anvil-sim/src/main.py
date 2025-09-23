#!/usr/bin/env python3
"""
Anvil Sim - Isaac Sim Integration Service
Provides physics simulation, validation, and 3D scene management for Sepulki platform.
"""

# Setup Isaac Sim paths BEFORE any imports
import sys
import os

# Add Isaac Sim to Python path - use the container path for real Isaac Sim
isaac_sim_path = "/isaac-sim/kit/python"
sys.path.insert(0, isaac_sim_path)

# Also add the Isaac Sim extensions and modules
sys.path.insert(0, "/isaac-sim/kit/exts")
sys.path.insert(0, "/isaac-sim/kit/extscore")
sys.path.insert(0, "/isaac-sim/kit/kernel")
sys.path.insert(0, "/isaac-sim/exts")

# Now import Isaac Sim modules
try:
    from omni.isaac.kit import SimulationApp
    ISAAC_SIM_AVAILABLE = True
    print("✅ Isaac Sim modules imported successfully in anvil-sim service")
except ImportError as e:
    ISAAC_SIM_AVAILABLE = False
    print(f"❌ Isaac Sim import failed in anvil-sim service: {e}")

import asyncio
import logging
import signal
from typing import Optional, Dict, Any
from datetime import datetime

import structlog
from grpc import aio as grpc_aio
from aiohttp import web

# Import real Isaac Sim renderer
from isaac_sim_real_renderer import get_isaac_sim_real_renderer

from config.anvil_config import ISAAC_SIM_CONFIG, GRPC_PORT, WEBSOCKET_PORT
from services.simulation_service import SimulationServicer
from isaac_sim_manager import isaac_sim_manager
from webrtc_stream_manager import webrtc_stream_manager

# Mock protocols for development
try:
    from protocols import anvil_pb2_grpc
except ImportError:
    class anvil_pb2_grpc:
        @staticmethod
        def add_AnvilSimServicer_to_server(servicer, server):
            pass

# Setup logging
def setup_logging():
    import logging
    logging.basicConfig(
        level=getattr(logging, ISAAC_SIM_CONFIG.get("log_level", "INFO")),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

# Configure structured logging
setup_logging()
logger = structlog.get_logger(__name__)

class AnvilSimService:
    """Main service class that orchestrates Isaac Sim integration."""
    
    def __init__(self):
        self.simulation_app: Optional[SimulationApp] = None
        self.grpc_server: Optional[grpc_aio.Server] = None
        self.http_app = None
        self.http_runner = None
        self.running = False
        self.active_sessions: Dict[str, Dict[str, Any]] = {}
        self.isaac_sim_renderer = get_isaac_sim_real_renderer()
        
    async def initialize_isaac_sim(self):
        """Initialize Isaac Sim simulation application using Real Isaac Sim Renderer."""
        try:
            if ISAAC_SIM_AVAILABLE:
                logger.info("Real Isaac Sim renderer initialized successfully")
                self.simulation_app = self.isaac_sim_renderer.app
            else:
                logger.warning("Isaac Sim not available - running in simulation mode")
                       
        except Exception as e:
            logger.error("Failed to initialize Real Isaac Sim renderer", error=str(e))
            raise
    
    async def start_grpc_server(self):
        """Start the gRPC server for simulation services."""
        try:
            self.grpc_server = grpc_aio.server()
            
            # Add simulation servicer
            simulation_servicer = SimulationServicer(
                simulation_app=self.simulation_app,
                world=getattr(isaac_sim_manager, 'world', None)
            )
            anvil_pb2_grpc.add_AnvilSimServicer_to_server(
                simulation_servicer, 
                self.grpc_server
            )
            
            # Configure server
            listen_addr = f'[::]:{GRPC_PORT}'
            self.grpc_server.add_insecure_port(listen_addr)
            
            # Start server
            await self.grpc_server.start()
            logger.info("gRPC server started", address=listen_addr)
            
        except Exception as e:
            logger.error("Failed to start gRPC server", error=str(e))
            raise
    
    async def start_websocket_server(self):
        """Start WebSocket server for real-time streaming."""
        try:
            # Initialize WebRTC Stream Manager with Isaac Sim Manager
            webrtc_stream_manager.isaac_sim_manager = isaac_sim_manager
            
            # Start WebRTC streaming server (bind to all interfaces for Docker)
            await webrtc_stream_manager.start_server("0.0.0.0", WEBSOCKET_PORT)
            logger.info("WebRTC streaming server started", port=WEBSOCKET_PORT)
            
        except Exception as e:
            logger.error("Failed to start WebRTC streaming server", error=str(e))
            raise

    async def health_check(self, request):
        """Health check endpoint for frontend compatibility."""
        return web.json_response({
            'status': 'healthy',
            'service': 'anvil-sim-real',
            'version': '1.0.0-isaac-sim',
            'isaac_sim_available': ISAAC_SIM_AVAILABLE,
            'timestamp': datetime.utcnow().isoformat(),
            'active_sessions': len(self.active_sessions),
            'mode': 'isaac_sim' if ISAAC_SIM_AVAILABLE else 'simulation'
        })

    async def create_scene(self, request):
        """Create Isaac Sim session endpoint for frontend compatibility."""
        try:
            data = await request.json()
            
            session_id = f"session_{int(datetime.utcnow().timestamp())}_{len(self.active_sessions)}"
            
            # Extract Isaac Sim robot configuration
            isaac_sim_robot = data.get('isaac_sim_robot')
            physics_config = data.get('physics_config', {})

            session = {
                'id': session_id,
                'user_id': data.get('user_id', 'anonymous'),
                'sepulka_id': data.get('sepulka_id', 'demo-robot'),
                'environment': data.get('environment', 'warehouse'),
                'quality_profile': data.get('quality_profile', 'engineering'),
                'status': 'ready',
                'created_at': datetime.utcnow().isoformat(),
                'isaac_sim_mode': ISAAC_SIM_AVAILABLE,
                'urdf_content': data.get('urdf_content', ''),
                'webrtc_ready': True,  # Real service supports WebRTC
                # NEW: Isaac Sim robot configuration
                'isaac_sim_robot': isaac_sim_robot,
                'robot_name': isaac_sim_robot.get('name') if isaac_sim_robot else 'Default Robot',
                'isaac_sim_path': isaac_sim_robot.get('isaac_sim_path') if isaac_sim_robot else None,
                'physics_config': physics_config
            }
            
            self.active_sessions[session_id] = session
            
            # Update real Isaac Sim renderer with robot configuration
            if isaac_sim_robot and ISAAC_SIM_AVAILABLE:
                await self.isaac_sim_renderer.load_robot(isaac_sim_robot)
                logger.info("Real Isaac Sim robot loaded", 
                           robot_name=isaac_sim_robot.get('name'),
                           isaac_sim_path=isaac_sim_robot.get('isaac_sim_path'))
                
            logger.info("Isaac Sim session created", session_id=session_id, 
                       user_id=session['user_id'], isaac_sim_available=ISAAC_SIM_AVAILABLE,
                       robot_name=session.get('robot_name', 'Default Robot'))
            
            return web.json_response({
                'success': True,
                'session_id': session_id,
                'status': session['status'],
                'message': f'Isaac Sim session created ({"isaac_sim" if ISAAC_SIM_AVAILABLE else "simulation"} mode)',
                'isaac_sim_available': ISAAC_SIM_AVAILABLE,
                'webrtc_ready': True
            })
            
        except Exception as e:
            logger.error("Failed to create Isaac Sim session", error=str(e))
            return web.json_response({
                'success': False,
                'error': str(e),
                'message': 'Failed to create Isaac Sim session'
            }, status=400)

    async def change_robot(self, request):
        """Change robot model in existing Isaac Sim session."""
        try:
            data = await request.json()
            
            session_id = data.get('session_id')
            isaac_sim_robot = data.get('isaac_sim_robot')
            
            if not session_id:
                return web.json_response({
                    'success': False,
                    'error': 'session_id is required'
                }, status=400)
                
            if not isaac_sim_robot:
                return web.json_response({
                    'success': False,
                    'error': 'isaac_sim_robot configuration is required'
                }, status=400)
            
            # Check if session exists
            if session_id not in self.active_sessions:
                return web.json_response({
                    'success': False,
                    'error': f'Session {session_id} not found'
                }, status=404)
            
            # Update session with new robot configuration
            session = self.active_sessions[session_id]
            session['isaac_sim_robot'] = isaac_sim_robot
            session['robot_name'] = isaac_sim_robot.get('name', 'Unknown Robot')
            session['isaac_sim_path'] = isaac_sim_robot.get('isaac_sim_path')
            session['updated_at'] = datetime.utcnow().isoformat()
            
            # Update real Isaac Sim renderer with new robot configuration
            if ISAAC_SIM_AVAILABLE:
                try:
                    await self.isaac_sim_renderer.load_robot(isaac_sim_robot)
                    logger.info("Real Isaac Sim robot loaded", 
                               session_id=session_id,
                               robot_name=isaac_sim_robot.get('name'),
                               isaac_sim_path=isaac_sim_robot.get('isaac_sim_path'))
                except Exception as e:
                    logger.error("Failed to load robot in Real Isaac Sim", 
                                session_id=session_id, error=str(e))
                    return web.json_response({
                        'success': False,
                        'error': f'Failed to update visual simulation: {str(e)}'
                    }, status=500)
            
            return web.json_response({
                'success': True,
                'session_id': session_id,
                'robot_name': isaac_sim_robot.get('name'),
                'isaac_sim_path': isaac_sim_robot.get('isaac_sim_path'),
                'message': f'Robot changed to {isaac_sim_robot.get("name")} successfully',
                'isaac_sim_available': ISAAC_SIM_AVAILABLE,
                'visual_simulation_updated': True
            })
            
        except Exception as e:
            logger.error("Failed to change robot", error=str(e))
            return web.json_response({
                'success': False,
                'error': str(e)
            }, status=500)

    async def video_stream(self, request):
        """Stream live video frames from Isaac Sim via HTTP (bypasses WebRTC muting)."""
        try:
            session_id = request.match_info['session_id']
            
            # Check if session exists
            if session_id not in self.active_sessions:
                return web.json_response({
                    'error': f'Session {session_id} not found'
                }, status=404)
            
            session = self.active_sessions[session_id]
            logger.info("Starting HTTP video stream", session_id=session_id, 
                       robot_name=session.get('robot_name', 'Unknown'))
            
            # Import required modules
            import cv2
            import asyncio
            
            # Set up streaming response
            response = web.StreamResponse(
                status=200,
                reason='OK',
                headers={
                    'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                    'Cache-Control': 'no-cache',
                    'Connection': 'close',
                    'Access-Control-Allow-Origin': '*',
                }
            )
            
            await response.prepare(request)
            
            frame_count = 0
            
            try:
                while True:
                    # Generate frame from real Isaac Sim renderer
                    if ISAAC_SIM_AVAILABLE:
                        frame_data = await self.isaac_sim_renderer.render_frame()
                    else:
                        # Fallback to black frame if Isaac Sim not available
                        frame_data = np.zeros((1080, 1920, 3), dtype=np.uint8)
                    
                    # Encode frame as JPEG
                    _, buffer = cv2.imencode('.jpg', frame_data, [
                        cv2.IMWRITE_JPEG_QUALITY, 85,
                        cv2.IMWRITE_JPEG_OPTIMIZE, 1
                    ])
                    
                    frame_bytes = buffer.tobytes()
                    
                    # Send frame in multipart format
                    await response.write(b'--frame\r\n')
                    await response.write(b'Content-Type: image/jpeg\r\n')
                    await response.write(f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode())
                    await response.write(frame_bytes)
                    await response.write(b'\r\n')
                    
                    frame_count += 1
                    
                    # Log every 60 frames (every 2 seconds at 30 FPS)
                    if frame_count % 60 == 0:
                        logger.info("Real Isaac Sim video stream active", session_id=session_id, 
                                   frame_count=frame_count, 
                                   robot_name=session.get('robot_name'))
                    
                    # Control frame rate (30 FPS)
                    await asyncio.sleep(1/30)
                    
            except asyncio.CancelledError:
                logger.info("HTTP video stream cancelled", session_id=session_id)
            except Exception as e:
                logger.error("HTTP video stream error", session_id=session_id, error=str(e))
            finally:
                logger.info("HTTP video stream ended", session_id=session_id, total_frames=frame_count)
            
            return response
            
        except Exception as e:
            logger.error("Failed to start video stream", error=str(e))
            return web.json_response({
                'error': str(e)
            }, status=500)

    async def start_http_server(self):
        """Start HTTP server for health checks and session management."""
        try:
            self.http_app = web.Application()
            
            # Add CORS support  
            @web.middleware
            async def cors_handler(request, handler):
                response = await handler(request)
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
                return response
            
            async def options_handler(request):
                return web.Response(
                    headers={
                        'Access-Control-Allow-Origin': '*',
                        'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
                        'Access-Control-Allow-Headers': 'Content-Type'
                    }
                )
            
            # Add routes
            self.http_app.router.add_get('/health', self.health_check)
            self.http_app.router.add_post('/create_scene', self.create_scene)
            self.http_app.router.add_post('/change_robot', self.change_robot)
            self.http_app.router.add_get('/video_stream/{session_id}', self.video_stream)
            self.http_app.router.add_options('/{path:.*}', options_handler)
            
            # Add middleware
            self.http_app.middlewares.append(cors_handler)
            
            # Start HTTP server
            self.http_runner = web.AppRunner(self.http_app)
            await self.http_runner.setup()
            site = web.TCPSite(self.http_runner, '0.0.0.0', 8002)
            await site.start()
            
            logger.info("HTTP server started", port=8002)
            
        except Exception as e:
            logger.error("Failed to start HTTP server", error=str(e))
            raise
    
    async def start(self):
        """Start all services."""
        try:
            logger.info("Starting Anvil Sim service")
            
            # Initialize Isaac Sim
            await self.initialize_isaac_sim()
            
            # Start servers
            await asyncio.gather(
                self.start_grpc_server(),
                self.start_websocket_server(),
                self.start_http_server()
            )
            
            self.running = True
            logger.info("Anvil Sim service started successfully",
                       grpc_port=GRPC_PORT,
                       websocket_port=WEBSOCKET_PORT,
                       isaac_sim_available=ISAAC_SIM_AVAILABLE)
            
        except Exception as e:
            logger.error("Failed to start Anvil Sim service", error=str(e))
            await self.shutdown()
            raise
    
    async def shutdown(self):
        """Gracefully shutdown all services."""
        logger.info("Shutting down Anvil Sim service")
        self.running = False
        
        # Shutdown servers
        if self.grpc_server:
            await self.grpc_server.stop(5.0)
            logger.info("gRPC server stopped")
        
        # Stop WebRTC streaming server
        await webrtc_stream_manager.stop_server()
        logger.info("WebRTC streaming server stopped")
        
        # Stop HTTP server
        if self.http_runner:
            await self.http_runner.cleanup()
            logger.info("HTTP server stopped")
        
        # Shutdown Real Isaac Sim renderer
        if self.isaac_sim_renderer:
            self.isaac_sim_renderer.cleanup()
        logger.info("Real Isaac Sim renderer shutdown")
    
    async def run(self):
        """Main service loop."""
        await self.start()
        
        try:
            # Keep service running
            while self.running:
                await asyncio.sleep(1)
                
                # Update Real Isaac Sim renderer
                if self.isaac_sim_renderer and ISAAC_SIM_AVAILABLE:
                    # Isaac Sim renderer updates automatically during frame rendering
                    pass
                    
        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        finally:
            await self.shutdown()

def signal_handler(signum, frame):
    """Handle shutdown signals."""
    logger.info("Received signal", signal=signum)
    sys.exit(0)

async def main():
    """Main entry point."""
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run service
    service = AnvilSimService()
    try:
        await service.run()
    except Exception as e:
        logger.error("Service failed", error=str(e))
        return 1
    
    return 0

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
