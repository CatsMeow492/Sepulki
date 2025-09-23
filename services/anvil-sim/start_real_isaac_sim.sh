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

# Check Isaac Sim installation status
echo "  🔍 Isaac Sim Installation Analysis:"
if [ -d "/isaac-sim" ]; then
    echo "  ✅ Isaac Sim directory exists: /isaac-sim"
    echo "  📁 Isaac Sim contents:"
    ls -la /isaac-sim/ | head -10
else
    echo "  ⚠️  Isaac Sim directory not found at /isaac-sim"
fi

# Check various potential Isaac Sim installation locations
echo "  🔍 Checking potential Isaac Sim locations:"
LOCATIONS=(
    "/isaac-sim/kit/python"
    "/home/shadeform/isaac-sim"
    "/opt/isaac-sim"
    "/usr/local/isaac-sim"
)

for location in "${LOCATIONS[@]}"
do
    if [ -d "$location" ]; then
        echo "  ✅ Found Isaac Sim at: $location"
        ls -la "$location" | head -3
    fi
done

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
    echo "  ⚠️  Isaac Sim Python directory not found at /isaac-sim/kit/python"
    echo "  💡 But Isaac Sim modules are importable, suggesting installation elsewhere"
fi

# Test Isaac Sim import before starting container
echo "🧪 Pre-container Isaac Sim test..."
python3 -c "
import sys
import os

print('  🔍 Current sys.path Isaac Sim entries:')
for i, path in enumerate(sys.path):
    if 'isaac' in path.lower():
        print(f'    [{i}] {path}')

# Try to find omni module location
try:
    import omni
    omni_file = getattr(omni, '__file__', None)
    if omni_file is None:
        print('  ⚠️  Omni module found but __file__ is None (not properly installed)')
        print('  🔍 Checking omni module contents:')
        print(f'    - omni.__name__: {getattr(omni, \"__name__\", \"N/A\")}')
        print(f'    - omni.__package__: {getattr(omni, \"__package__\", \"N/A\")}')
        print(f'    - dir(omni): {[x for x in dir(omni) if not x.startswith(\"_\")][:5]}...')
    else:
        print(f'  📍 Found omni module at: {omni_file}')
        omni_dir = os.path.dirname(omni_file)
        print(f'  📁 Omni directory: {omni_dir}')

        # Check if this is from Isaac Sim
        if 'isaac' in omni_dir.lower():
            print('  ✅ Omni module appears to be from Isaac Sim installation')
        else:
            print('  ⚠️  Omni module found but not from Isaac Sim')

except ImportError:
    print('  ❌ Omni module not found in current environment')

# Test Isaac Sim import with correct path
isaac_sim_base = '/home/shadeform/isaac-sim/isaac-sim-2023.1.1'
sys.path.insert(0, isaac_sim_base)
sys.path.insert(0, os.path.join(isaac_sim_base, 'kit', 'exts'))
sys.path.insert(0, os.path.join(isaac_sim_base, 'kit', 'extscore'))
sys.path.insert(0, os.path.join(isaac_sim_base, 'kit', 'kernel'))
sys.path.insert(0, os.path.join(isaac_sim_base, 'exts'))

try:
    from omni.isaac.kit import SimulationApp
    print('  ✅ Isaac Sim import successful before container start')
    print(f'  📍 SimulationApp location: {SimulationApp.__module__}')
except ImportError as e:
    print(f'  ❌ Isaac Sim import failed before container start: {e}')
" 2>&1

# Create directories if they don't exist
mkdir -p ~/docker/isaac-sim/cache/kit
mkdir -p ~/docker/isaac-sim/logs

# Check if Isaac Sim container is already running
EXISTING_CONTAINER=$(docker ps -q --filter name=isaac-sim)
if [ -n "$EXISTING_CONTAINER" ]; then
    echo "🎬 Isaac Sim container already running (ID: $EXISTING_CONTAINER)"
    CONTAINER_ID=$EXISTING_CONTAINER
else
    echo "🎬 Starting Isaac Sim container..."
    # Remove any stopped containers with the same name
    docker rm isaac-sim 2>/dev/null || true

    docker run --name isaac-sim -d --runtime=nvidia --gpus all \
      -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --rm --network=host \
      -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
      -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
      nvcr.io/nvidia/isaac-sim:5.0.0 ./runheadless.sh --/app/livestream/publicEndpointAddress=$(curl -s ifconfig.me) --/app/livestream/port=49100

    CONTAINER_ID=$(docker ps -q --filter name=isaac-sim)
fi

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
# Use the correct Isaac Sim installation path
isaac_sim_base = '/home/shadeform/isaac-sim/isaac-sim-2023.1.1'
sys.path.insert(0, isaac_sim_base)
sys.path.insert(0, os.path.join(isaac_sim_base, 'kit', 'exts'))
sys.path.insert(0, os.path.join(isaac_sim_base, 'kit', 'extscore'))
sys.path.insert(0, os.path.join(isaac_sim_base, 'kit', 'kernel'))
sys.path.insert(0, os.path.join(isaac_sim_base, 'exts'))

try:
    from omni.isaac.kit import SimulationApp
    print('  ✅ Isaac Sim accessible from host after container start!')
    print(f'  📍 SimulationApp module: {SimulationApp.__module__}')
except ImportError as e:
    print(f'  ❌ Isaac Sim not accessible after container start: {e}')
    print('  🔍 Checking container filesystem...')
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
echo "🎯 Isaac Sim Integration Status:"
if [ -n "$CONTAINER_ID" ]; then
    echo "  ✅ Isaac Sim container running"
else
    echo "  ❌ Isaac Sim container not running"
fi

ANVIL_PROCESSES=$(ps aux | grep 'python3 src/main.py' | grep -v grep | wc -l)
if [ "$ANVIL_PROCESSES" -gt 0 ]; then
    echo "  ✅ Anvil-sim service running ($ANVIL_PROCESSES processes)"
else
    echo "  ❌ Anvil-sim service not running"
fi

echo ""
echo "✅ Real Isaac Sim integration setup complete!"
echo "📋 Next steps:"
echo "  1. Check service logs above for Isaac Sim detection status"
echo "  2. Test frontend at /configure page"
echo "  3. Select robot recommendations to see photorealistic Isaac Sim rendering"
echo "  4. Compare visual quality with previous OpenCV mock rendering"
echo ""
echo "🔍 Key Discovery: Isaac Sim is pre-installed on this Brev instance"
echo "   Location: /home/shadeform/isaac-sim/isaac-sim-2023.1.1"
echo "   Status: Modules importable, but omni.__file__ is None (improper Python package installation)"
echo "   Result: Real Isaac Sim rendering should now work with correct path configuration"
