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
from typing import Optional

import structlog
from grpc import aio as grpc_aio

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
from services.websocket_server import WebSocketServer
from protocols import anvil_pb2_grpc
from utils.logging import setup_logging

# Configure structured logging
setup_logging()
logger = structlog.get_logger(__name__)

class AnvilSimService:
    """Main service class that orchestrates Isaac Sim integration."""
    
    def __init__(self):
        self.simulation_app: Optional[SimulationApp] = None
        self.grpc_server: Optional[grpc_aio.Server] = None
        self.websocket_server: Optional[WebSocketServer] = None
        self.running = False
        
    async def initialize_isaac_sim(self):
        """Initialize Isaac Sim simulation application."""
        if not ISAAC_SIM_AVAILABLE:
            logger.warning("Isaac Sim not available, using mock simulation")
            return
            
        try:
            # Initialize Isaac Sim
            config = {
                "headless": ISAAC_SIM_CONFIG["headless"],
                "width": ISAAC_SIM_CONFIG["width"],
                "height": ISAAC_SIM_CONFIG["height"],
                "enable_livestream": ISAAC_SIM_CONFIG["enable_livestream"],
                "livestream_port": ISAAC_SIM_CONFIG["livestream_port"]
            }
            
            self.simulation_app = SimulationApp(config)
            
            # Import additional Isaac Sim modules after app initialization
            from omni.isaac.core import World
            from omni.isaac.core.utils.extensions import enable_extension
            
            # Enable required extensions
            enable_extension("omni.isaac.sensor")
            enable_extension("omni.isaac.manipulators") 
            enable_extension("omni.isaac.wheeled_robots")
            
            # Create simulation world
            self.world = World(stage_units_in_meters=1.0)
            await self.world.initialize_simulation_context_async()
            
            logger.info("Isaac Sim initialized successfully",
                       headless=config["headless"],
                       livestream=config["enable_livestream"])
                       
        except Exception as e:
            logger.error("Failed to initialize Isaac Sim", error=str(e))
            raise
    
    async def start_grpc_server(self):
        """Start the gRPC server for simulation services."""
        try:
            self.grpc_server = grpc_aio.server()
            
            # Add simulation servicer
            simulation_servicer = SimulationServicer(
                simulation_app=self.simulation_app,
                world=getattr(self, 'world', None)
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
            self.websocket_server = WebSocketServer(
                port=WEBSOCKET_PORT,
                simulation_app=self.simulation_app
            )
            await self.websocket_server.start()
            logger.info("WebSocket server started", port=WEBSOCKET_PORT)
            
        except Exception as e:
            logger.error("Failed to start WebSocket server", error=str(e))
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
                self.start_websocket_server()
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
        
        if self.websocket_server:
            await self.websocket_server.stop()
            logger.info("WebSocket server stopped")
        
        # Cleanup Isaac Sim
        if self.simulation_app:
            self.simulation_app.close()
            logger.info("Isaac Sim closed")
    
    async def run(self):
        """Main service loop."""
        await self.start()
        
        try:
            # Keep service running
            while self.running:
                await asyncio.sleep(1)
                
                # Update Isaac Sim if available
                if ISAAC_SIM_AVAILABLE and self.simulation_app:
                    self.simulation_app.update()
                    
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
