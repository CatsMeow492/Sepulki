#!/usr/bin/env python3
"""
Isaac Sim Livestream API Connection
"""

import sys
import os
import time

# Add Isaac Sim paths
isaac_sim_path = "/isaac-sim"
sys.path.insert(0, isaac_sim_path)
sys.path.insert(0, os.path.join(isaac_sim_path, "kit", "python"))

try:
    import carb
    from omni.isaac.kit import SimulationApp
    
    print("✅ Isaac Sim modules imported successfully")
    
    # Check if livestream is available
    try:
        from omni.kit.livestream.core import LivestreamCore
        print("✅ LivestreamCore available")
    except ImportError as e:
        print(f"❌ LivestreamCore not available: {e}")
    
    # Check livestream WebRTC
    try:
        from omni.kit.livestream.webrtc import WebRTCLivestream
        print("✅ WebRTCLivestream available")
    except ImportError as e:
        print(f"❌ WebRTCLivestream not available: {e}")
    
    # Check if we can access livestream through carb settings
    try:
        settings = carb.settings.get_settings()
        print(f"✅ Carb settings available")
        
        # Check livestream settings
        livestream_port = settings.get("/app/livestream/port")
        livestream_address = settings.get("/app/livestream/publicEndpointAddress")
        print(f"📡 Livestream port: {livestream_port}")
        print(f"📡 Livestream address: {livestream_address}")
        
    except Exception as e:
        print(f"❌ Error accessing settings: {e}")
        
except ImportError as e:
    print(f"❌ Isaac Sim import failed: {e}")
    print("This script needs to run inside the Isaac Sim container")
