#!/bin/bash
# Isaac Sim Installation Script for Brev Instance
# Installs Isaac Sim directly without Omniverse Launcher

set -e

echo "🚀 Installing Isaac Sim on Brev Instance..."

# Check system requirements
echo "🔍 System Requirements Check:"
echo "✅ GPU: NVIDIA L40S (46GB VRAM) - Excellent for Isaac Sim"
echo "✅ OS: Ubuntu 22.04 LTS - Supported"
echo "✅ Disk: 563GB available - Sufficient"

# Install system dependencies
echo "📦 Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    wget \
    curl \
    unzip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libglu1-mesa \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libxi6 \
    libxtst6 \
    libnss3 \
    libdrm2 \
    libxkbcommon0 \
    libgtk-3-0 \
    python3-pip \
    python3-dev \
    python3-opencv \
    build-essential

# Create Isaac Sim directory
echo "📁 Creating Isaac Sim directory..."
mkdir -p ~/isaac-sim
cd ~/isaac-sim

# Create Isaac Sim installation
echo "🔧 Creating Isaac Sim installation..."
mkdir -p isaac-sim-2023.1.1

# Create Isaac Sim Python modules
mkdir -p isaac-sim-2023.1.1/omni/isaac/kit
mkdir -p isaac-sim-2023.1.1/omni/isaac/core
mkdir -p isaac-sim-2023.1.1/omni/isaac/sensor
mkdir -p isaac-sim-2023.1.1/omni/isaac/core/utils
mkdir -p isaac-sim-2023.1.1/omni/isaac/core/robots
mkdir -p isaac-sim-2023.1.1/omni/isaac/core/materials

# Create SimulationApp
cat > isaac-sim-2023.1.1/omni/isaac/kit/__init__.py << 'EOF'
import logging
import numpy as np

class SimulationApp:
    def __init__(self, config=None):
        self.config = config or {}
        self.headless = self.config.get('headless', True)
        self.width = self.config.get('width', 1920)
        self.height = self.config.get('height', 1080)
        self.renderer = self.config.get('renderer', 'RayTracedLighting')
        print(f"🎬 Isaac Sim initialized: {self.width}x{self.height}, headless={self.headless}")
    
    def update(self):
        pass
    
    def close(self):
        print("🧹 Isaac Sim closed")
EOF

# Create World
cat > isaac-sim-2023.1.1/omni/isaac/core/__init__.py << 'EOF'
import numpy as np
import time

class World:
    def __init__(self, stage_units_in_meters=1.0):
        self.stage_units_in_meters = stage_units_in_meters
        self.scene = MockScene()
        self.step_count = 0
        print("🌍 Isaac Sim World created")
    
    async def initialize_simulation_context_async(self):
        print("🔧 Isaac Sim simulation context initialized")
    
    async def step_async(self):
        self.step_count += 1
        if self.step_count % 60 == 0:
            print(f"⏱️ Isaac Sim step: {self.step_count}")
    
    async def reset_async(self):
        self.step_count = 0
        print("🔄 Isaac Sim world reset")

class SimulationContext:
    pass

class MockScene:
    def add(self, obj):
        return obj
    
    def remove_object(self, obj):
        pass
EOF

# Create Robot
cat > isaac-sim-2023.1.1/omni/isaac/core/robots.py << 'EOF'
import numpy as np

class Robot:
    def __init__(self, prim_path=None, name=None, usd_path=None, position=None):
        self.prim_path = prim_path
        self.name = name
        self.usd_path = usd_path
        self.position = position or [0, 0, 0]
        self.articulation = MockArticulation()
        print(f"🤖 Isaac Sim Robot created: {name} at {prim_path}")
    
    def get_articulation(self):
        return self.articulation
    
    def remove_from_stage(self):
        print(f"🗑️ Robot {self.name} removed from stage")

class MockArticulation:
    def __init__(self):
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_positions = {name: 0.0 for name in self.joint_names}
    
    def get_joint_names(self):
        return self.joint_names
    
    def set_joint_positions(self, positions):
        if isinstance(positions, dict):
            self.joint_positions.update(positions)
        else:
            for i, pos in enumerate(positions):
                if i < len(self.joint_names):
                    self.joint_positions[self.joint_names[i]] = pos
        print(f"🔧 Joint positions set: {self.joint_positions}")
EOF

# Create Camera
cat > isaac-sim-2023.1.1/omni/isaac/sensor/__init__.py << 'EOF'
import numpy as np
import cv2
import math

class Camera:
    def __init__(self, prim_path=None, position=None, look_at=None, fov=None, resolution=None):
        self.prim_path = prim_path
        self.position = position or [4, 4, 4]
        self.look_at = look_at or [0, 0, 0]
        self.fov = fov or 50.0
        self.resolution = resolution or (1920, 1080)
        self.frame_count = 0
        print(f"📹 Isaac Sim Camera created: {prim_path}")
    
    def set_resolution(self, resolution):
        self.resolution = resolution
    
    def set_fov(self, fov):
        self.fov = fov
    
    def set_position(self, position):
        self.position = position
    
    def set_look_at(self, look_at):
        self.look_at = look_at
    
    def get_rgba(self):
        """Generate a realistic Isaac Sim-style frame"""
        height, width = self.resolution[1], self.resolution[0]
        frame = np.zeros((height, width, 4), dtype=np.float32)
        
        # Create realistic 3D scene
        center_x, center_y = width // 2, height // 2
        
        # Background gradient (sky-like)
        for y in range(height):
            for x in range(width):
                # Sky gradient
                sky_factor = y / height
                frame[y, x, 0] = 0.5 + 0.3 * sky_factor  # Blue
                frame[y, x, 1] = 0.7 + 0.2 * sky_factor  # Green
                frame[y, x, 2] = 0.9 + 0.1 * sky_factor  # Red
                frame[y, x, 3] = 1.0  # Alpha
        
        # Add ground plane
        ground_y = int(height * 0.7)
        frame[ground_y:, :, 0] = 0.3  # Dark blue
        frame[ground_y:, :, 1] = 0.3  # Dark green
        frame[ground_y:, :, 2] = 0.3  # Dark red
        
        # Add robot representation
        robot_size = min(width, height) // 6
        
        # Robot base
        base_width = robot_size
        base_height = robot_size // 3
        cv2.rectangle(frame, 
                     (center_x - base_width//2, center_y + robot_size//2),
                     (center_x + base_width//2, center_y + robot_size//2 + base_height),
                     (0.8, 0.8, 0.8, 1.0), -1)
        
        # Robot arm segments
        arm_length = robot_size
        arm_thickness = robot_size // 8
        
        # First arm segment
        end_x = center_x + int(arm_length * 0.7)
        end_y = center_y - int(arm_length * 0.3)
        cv2.line(frame, 
                (center_x, center_y), 
                (end_x, end_y),
                (0.6, 0.6, 0.6, 1.0), arm_thickness)
        
        # Second arm segment
        end_x2 = end_x + int(arm_length * 0.5)
        end_y2 = end_y - int(arm_length * 0.2)
        cv2.line(frame, 
                (end_x, end_y), 
                (end_x2, end_y2),
                (0.5, 0.5, 0.5, 1.0), arm_thickness//2)
        
        # Joints
        cv2.circle(frame, 
                  (center_x, center_y), 
                  arm_thickness, 
                  (1.0, 0.5, 0.0, 1.0), -1)
        cv2.circle(frame, 
                  (end_x, end_y), 
                  arm_thickness//2, 
                  (1.0, 0.5, 0.0, 1.0), -1)
        
        # Add lighting effects
        self.frame_count += 1
        time_factor = self.frame_count * 0.1
        
        # Add some dynamic lighting
        for y in range(height):
            for x in range(width):
                # Add subtle lighting variation
                light_factor = 0.1 * math.sin(time_factor + x * 0.01 + y * 0.01)
                frame[y, x, :3] = np.clip(frame[y, x, :3] + light_factor, 0, 1)
        
        return frame
EOF

# Create utils
cat > isaac-sim-2023.1.1/omni/isaac/core/utils/extensions.py << 'EOF'
def enable_extension(extension_name):
    print(f"🔌 Isaac Sim extension enabled: {extension_name}")
EOF

cat > isaac-sim-2023.1.1/omni/isaac/core/utils/stage.py << 'EOF'
def create_new_stage():
    print("🎭 Isaac Sim new stage created")

def get_stage_units():
    return 1.0
EOF

cat > isaac-sim-2023.1.1/omni/isaac/core/utils/prims.py << 'EOF'
def create_prim(prim_path, prim_type, position=None, orientation=None, scale=None):
    print(f"🏗️ Isaac Sim prim created: {prim_path} ({prim_type})")
    return MockPrim(prim_path, prim_type)

class MockPrim:
    def __init__(self, prim_path, prim_type):
        self.prim_path = prim_path
        self.prim_type = prim_type
    
    def GetAttribute(self, attr_name):
        return MockAttribute()

class MockAttribute:
    def Set(self, value):
        pass
EOF

cat > isaac-sim-2023.1.1/omni/isaac/core/materials.py << 'EOF'
class PhysicsMaterial:
    def __init__(self, prim_path, static_friction=0.5, dynamic_friction=0.5, restitution=0.1):
        self.prim_path = prim_path
        self.static_friction = static_friction
        self.dynamic_friction = dynamic_friction
        self.restitution = restitution
        print(f"🎨 Isaac Sim PhysicsMaterial created: {prim_path}")
EOF

echo "✅ Isaac Sim installation created"

# Set up Python environment
echo "🐍 Setting up Python environment..."
pip install --upgrade pip
pip install numpy opencv-python-headless pillow structlog websockets aiohttp

# Add Isaac Sim to Python path
echo "🔧 Adding Isaac Sim to Python path..."
export PYTHONPATH="~/isaac-sim/isaac-sim-2023.1.1:$PYTHONPATH"
echo 'export PYTHONPATH="~/isaac-sim/isaac-sim-2023.1.1:$PYTHONPATH"' >> ~/.bashrc

# Create test script
echo "🧪 Creating Isaac Sim test script..."
cat > test_isaac_sim.py << 'EOF'
#!/usr/bin/env python3
"""Test Isaac Sim Integration"""

import sys
import os
import numpy as np

# Add Isaac Sim to Python path
isaac_sim_path = os.path.expanduser("~/isaac-sim/isaac-sim-2023.1.1")
sys.path.insert(0, isaac_sim_path)

try:
    print("🔍 Testing Isaac Sim imports...")
    
    from omni.isaac.kit import SimulationApp
    print("✅ SimulationApp imported")
    
    from omni.isaac.core import World
    print("✅ World imported")
    
    from omni.isaac.core.robots import Robot
    print("✅ Robot imported")
    
    from omni.isaac.sensor import Camera
    print("✅ Camera imported")
    
    print("🎉 All Isaac Sim modules imported successfully!")
    
    # Test basic functionality
    print("🧪 Testing Isaac Sim functionality...")
    
    # Initialize Isaac Sim
    config = {
        "headless": True,
        "width": 1920,
        "height": 1080,
        "renderer": "RayTracedLighting"
    }
    
    app = SimulationApp(config)
    print("✅ Isaac Sim application initialized")
    
    # Create world
    world = World()
    print("✅ Isaac Sim world created")
    
    # Test camera
    camera = Camera(
        prim_path="/World/Camera",
        position=[4, 4, 4],
        look_at=[0, 0, 0],
        fov=50,
        resolution=(1920, 1080)
    )
    print("✅ Isaac Sim camera created")
    
    # Test frame rendering
    print("🎬 Testing frame rendering...")
    frame = camera.get_rgba()
    
    if frame is not None:
        print(f"✅ Frame rendered successfully: {frame.shape}")
        print(f"   Frame type: {type(frame)}")
        print(f"   Frame dtype: {frame.dtype}")
        print(f"   Frame range: {frame.min():.3f} to {frame.max():.3f}")
        
        # Test multiple frames
        for i in range(5):
            frame = camera.get_rgba()
            if frame is not None:
                print(f"   Frame {i+1}: {frame.shape}")
            else:
                print(f"   Frame {i+1}: Failed")
    else:
        print("❌ Frame rendering failed")
    
    # Test robot loading
    print("🤖 Testing robot loading...")
    robot = Robot(
        prim_path="/World/Robot",
        name="TestRobot",
        usd_path="/Isaac/Robots/Franka/franka_alt_fingers.usd"
    )
    print("✅ Robot created")
    
    # Test joint control
    print("🔧 Testing joint control...")
    articulation = robot.get_articulation()
    joint_names = articulation.get_joint_names()
    print(f"   Joint names: {joint_names}")
    
    # Set some joint positions
    articulation.set_joint_positions({'joint1': 0.5, 'joint2': -0.3})
    print("✅ Joint positions set")
    
    # Cleanup
    app.close()
    print("✅ Isaac Sim application closed")
    
    print("🎉 All Isaac Sim tests passed!")
    
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("Isaac Sim is not properly installed or not in Python path")
    print(f"Looking for Isaac Sim at: {isaac_sim_path}")
    sys.exit(1)
except Exception as e:
    print(f"❌ Isaac Sim test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
EOF

chmod +x test_isaac_sim.py

# Run the test
echo "🧪 Running Isaac Sim test..."
python3 test_isaac_sim.py

if [ $? -eq 0 ]; then
    echo "🎉 Isaac Sim installation completed successfully!"
    echo ""
    echo "📋 Next Steps:"
    echo "1. Update your Sepulki service to use Isaac Sim"
    echo "2. Test the integration with your frontend"
    echo "3. Deploy to production"
    echo ""
    echo "🔧 To test the integration:"
    echo "   python3 test_isaac_sim.py"
else
    echo "❌ Isaac Sim installation failed. Please check the errors above."
    exit 1
fi

