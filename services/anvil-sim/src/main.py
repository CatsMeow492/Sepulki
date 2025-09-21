#!/usr/bin/env python3
"""
Anvil Sim - Isaac Sim Integration Service
Provides physics simulation, validation, and 3D scene management for Sepulki platform.
"""

import asyncio
import logging
import os
import signal
import sys
from typing import Optional, Dict, Any
from datetime import datetime

import structlog
from grpc import aio as grpc_aio
from aiohttp import web

# Isaac Sim imports (these would be available in Isaac Sim environment)
try:
    import omni
    from omni.isaac.kit import SimulationApp
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Warning: Isaac Sim not available. Running in simulation mode.")

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
        
    async def initialize_isaac_sim(self):
        """Initialize Isaac Sim simulation application using Isaac Sim Manager."""
        try:
            # Initialize Isaac Sim Manager
            success = await isaac_sim_manager.initialize()
            
            if success:
                self.simulation_app = isaac_sim_manager.simulation_app
                logger.info("Isaac Sim Manager initialized successfully")
            else:
                logger.warning("Isaac Sim Manager running in simulation mode")
                       
        except Exception as e:
            logger.error("Failed to initialize Isaac Sim Manager", error=str(e))
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
            
            # Update video generator with robot configuration
            if isaac_sim_robot:
                import sys
                import os
                sys.path.append(os.path.dirname(__file__))
                from video_frame_generator import get_video_generator
                get_video_generator().update_robot_config({
                    'name': isaac_sim_robot.get('name', 'Unknown Robot'),
                    'isaac_sim_path': isaac_sim_robot.get('isaac_sim_path'),
                    'specifications': isaac_sim_robot.get('specifications', {})
                })
                
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
        
        # Shutdown Isaac Sim Manager
        await isaac_sim_manager.shutdown()
        logger.info("Isaac Sim Manager shutdown")
    
    async def run(self):
        """Main service loop."""
        await self.start()
        
        try:
            # Keep service running
            while self.running:
                await asyncio.sleep(1)
                
                # Update Isaac Sim Manager
                isaac_sim_manager.update()
                    
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
