#!/usr/bin/env python3
"""
Anvil Sim Isaac Sim Application
Runs the Anvil Sim service within the Isaac Sim environment.
"""

import sys
import os
import asyncio
from omni.isaac.kit import SimulationApp

# Add the anvil-sim service to the path
sys.path.insert(0, "/home/shadeform/sepulki/services/anvil-sim/src")

# Import the anvil-sim service
from main import AnvilSimService

def main():
    """Main function to run the Anvil Sim service within Isaac Sim."""

    # Configure Isaac Sim for headless operation with minimal resources
    config = {
        "headless": True,
        "width": 1,  # Minimal window size since we don't need display
        "height": 1,
        "renderer": "RayTracedLighting",
        "active_gpu": 0,
        "physics_gpu": 0,
    }

    # Create the Isaac Sim application
    app = SimulationApp(config)
    print("🎬 Isaac Sim application created for Anvil Sim service")

    try:
        # Create and start the Anvil Sim service
        service = AnvilSimService()
        print("🔧 Anvil Sim service created")

        # Start the service
        asyncio.run(service.start_service())
        print("✅ Anvil Sim service started successfully")

        # Keep the service running
        while True:
            app.update()

    except KeyboardInterrupt:
        print("🛑 Shutting down Anvil Sim service...")
    except Exception as e:
        print(f"❌ Failed to run Anvil Sim service: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if 'service' in locals():
            service.cleanup()
        app.close()
        print("🧹 Isaac Sim application closed")

if __name__ == "__main__":
    main()
