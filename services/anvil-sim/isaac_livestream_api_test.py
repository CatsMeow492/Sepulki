#!/usr/bin/env python3
"""
Isaac Sim Livestream API Test
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
        from omni.kit.livestream.bind import acquire_livestream_interface
        livestream = acquire_livestream_interface()
        print("✅ Livestream interface acquired")
        
        # Check livestream status
        try:
            status = livestream.get_status()
            print(f"📊 Livestream status: {status}")
        except Exception as e:
            print(f"❌ Error getting status: {e}")
        
        # Check livestream settings
        try:
            settings = carb.settings.get_settings()
            port = settings.get("/app/livestream/port")
            address = settings.get("/app/livestream/publicEndpointAddress")
            enabled = settings.get("/app/livestream/enabled")
            print(f"📡 Livestream port: {port}")
            print(f"📡 Livestream address: {address}")
            print(f"📡 Livestream enabled: {enabled}")
        except Exception as e:
            print(f"❌ Error accessing settings: {e}")
            
    except ImportError as e:
        print(f"❌ Livestream interface not available: {e}")
        
except ImportError as e:
    print(f"❌ Isaac Sim import failed: {e}")
    print("This script needs to run inside the Isaac Sim container")
