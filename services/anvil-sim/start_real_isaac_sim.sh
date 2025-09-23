#!/bin/bash

# Start Real Isaac Sim Integration on Brev Instance
# This script starts Isaac Sim container and anvil-sim service with real Isaac Sim

set -e

echo "🚀 Starting Real Isaac Sim Integration..."

# Diagnostic information
echo "🔍 Isaac Sim Diagnostic Information:"
echo "  - Hostname: $(hostname)"
echo "  - Current user: $(whoami)"
echo "  - Working directory: $(pwd)"
echo "  - Python version: $(python3 --version 2>&1)"
echo "  - Docker version: $(docker --version)"
echo "  - NVIDIA GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader,nounits || echo 'N/A')"
echo "  - Public IP: $(curl -s ifconfig.me)"
echo "  - Isaac Sim install directory: /isaac-sim"
echo "  - Isaac Sim Python path: /isaac-sim/kit/python"
echo "  - Isaac Sim extensions: /isaac-sim/kit/exts"
echo "  - Isaac Sim kernel: /isaac-sim/kit/kernel"

# Check if Isaac Sim directory exists
if [ -d "/isaac-sim" ]; then
    echo "  ✅ Isaac Sim directory exists: /isaac-sim"
    echo "  📁 Isaac Sim contents:"
    ls -la /isaac-sim/ | head -10
else
    echo "  ❌ Isaac Sim directory not found: /isaac-sim"
fi

# Check if Isaac Sim Python exists
if [ -d "/isaac-sim/kit/python" ]; then
    echo "  ✅ Isaac Sim Python directory exists: /isaac-sim/kit/python"
    echo "  🐍 Python executable: /isaac-sim/kit/python/bin/python3.11"
    if [ -f "/isaac-sim/kit/python/bin/python3.11" ]; then
        echo "  ✅ Python executable exists"
    else
        echo "  ❌ Python executable not found"
    fi
else
    echo "  ❌ Isaac Sim Python directory not found"
fi

# Test Isaac Sim import before starting container
echo "🧪 Pre-container Isaac Sim test..."
python3 -c "
import sys
sys.path.insert(0, '/isaac-sim/kit/python')
sys.path.insert(0, '/isaac-sim/kit/exts')
sys.path.insert(0, '/isaac-sim/kit/extscore')
sys.path.insert(0, '/isaac-sim/kit/kernel')
sys.path.insert(0, '/isaac-sim/exts')
try:
    from omni.isaac.kit import SimulationApp
    print('  ✅ Isaac Sim import successful before container start')
except ImportError as e:
    print(f'  ❌ Isaac Sim import failed before container start: {e}')
" 2>/dev/null || echo "  ⚠️  Python test failed (expected if Isaac Sim not installed yet)"

# Create directories if they don't exist
mkdir -p ~/docker/isaac-sim/cache/kit
mkdir -p ~/docker/isaac-sim/logs

# Start Isaac Sim container in background
echo "🎬 Starting Isaac Sim container..."
docker run --name isaac-sim -d --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0 ./runheadless.sh --/app/livestream/publicEndpointAddress=$(curl -s ifconfig.me) --/app/livestream/port=49100

# Get container ID
CONTAINER_ID=$(docker ps -q --filter name=isaac-sim)
echo "  📦 Isaac Sim container ID: $CONTAINER_ID"

# Wait for Isaac Sim to start
echo "⏳ Waiting for Isaac Sim to initialize..."
sleep 45

# Check container status
echo "🔍 Container status after initialization:"
docker ps --filter name=isaac-sim --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

# Check if Isaac Sim directories are now available
echo "🔍 Post-container Isaac Sim directory check:"
if [ -d "/isaac-sim" ]; then
    echo "  ✅ Isaac Sim directory exists after container start"
    ls -la /isaac-sim/kit/ 2>/dev/null | head -5
else
    echo "  ❌ Isaac Sim directory still not found after container start"
fi

# Test Isaac Sim access
echo "🧪 Testing Isaac Sim access after container start..."
python3 -c "
import sys
import os
print('  📍 Python path entries related to Isaac Sim:')
for path in sys.path:
    if 'isaac' in path.lower():
        print(f'    - {path}')

print('  🔍 Environment variables:')
isaac_vars=\$(env | grep -i isaac || echo 'None found')
if [ \"\$isaac_vars\" != \"None found\" ]; then
    echo \"    \$isaac_vars\"
else
    echo '    None found'
fi

print('  🧪 Testing Isaac Sim import...')
sys.path.insert(0, '/isaac-sim/kit/python')
sys.path.insert(0, '/isaac-sim/kit/exts')
sys.path.insert(0, '/isaac-sim/kit/extscore')
sys.path.insert(0, '/isaac-sim/kit/kernel')
sys.path.insert(0, '/isaac-sim/exts')
try:
    from omni.isaac.kit import SimulationApp
    print('  ✅ Isaac Sim accessible from host after container start!')
except ImportError as e:
    print(f'  ❌ Isaac Sim not accessible after container start: {e}')
    print('  🔍 Checking if container is exposing filesystem...')
    docker exec isaac-sim ls -la /isaac-sim/kit/python/ 2>/dev/null | head -3 || echo '    Container filesystem not accessible'
    exit(1)
" 2>&1

# Start anvil-sim service
echo "🔧 Starting anvil-sim service with real Isaac Sim..."
cd ~/sepulki/services/anvil-sim

# Run the service directly (it will now detect Isaac Sim)
python3 src/main.py

# Final status check
echo ""
echo "🎯 Final Status Summary:"
echo "  📦 Isaac Sim Container: $(docker ps -q --filter name=isaac-sim | wc -l) running"
echo "  🔧 Anvil-sim Service: $(ps aux | grep 'python3 src/main.py' | grep -v grep | wc -l) processes"
echo "  🌐 Services listening on:"
netstat -tlnp 2>/dev/null | grep -E ':(8000|8001|8002)' | awk '{print "    - " $4 " (" $7 ")"}' || echo "    (netstat not available)"

echo ""
echo "✅ Real Isaac Sim integration setup complete!"
echo "📋 Next steps:"
echo "  1. Check frontend at /configure page"
echo "  2. Select robot recommendations to see real Isaac Sim rendering"
echo "  3. Compare visual quality with previous OpenCV mock rendering"
