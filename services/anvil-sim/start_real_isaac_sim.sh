#!/bin/bash

# Start Real Isaac Sim Integration on Brev Instance
# This script starts Isaac Sim container and anvil-sim service with real Isaac Sim

set -e

echo "🚀 Starting Real Isaac Sim Integration..."

# Create directories if they don't exist
mkdir -p ~/docker/isaac-sim/cache/kit
mkdir -p ~/docker/isaac-sim/logs

# Start Isaac Sim container in background
echo "🎬 Starting Isaac Sim container..."
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  nvcr.io/nvidia/isaac-sim:5.0.0 -c './runheadless.sh --/app/livestream/publicEndpointAddress=$(curl -s ifconfig.me) --/app/livestream/port=49100' &

# Wait for Isaac Sim to start
echo "⏳ Waiting for Isaac Sim to initialize..."
sleep 45

# Test Isaac Sim access
echo "🧪 Testing Isaac Sim access..."
python3 -c "
import sys
sys.path.insert(0, '/isaac-sim/kit/python')
try:
    from omni.isaac.kit import SimulationApp
    print('✅ Isaac Sim accessible from host!')
except ImportError as e:
    print(f'❌ Isaac Sim not accessible: {e}')
    exit(1)
"

# Start anvil-sim service
echo "🔧 Starting anvil-sim service with real Isaac Sim..."
cd ~/sepulki/services/anvil-sim
python3 src/main.py

echo "✅ Real Isaac Sim integration complete!"
