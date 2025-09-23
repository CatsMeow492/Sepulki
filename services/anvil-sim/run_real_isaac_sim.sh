#!/bin/bash
# Run Real Isaac Sim Service on Brev Instance

set -e

echo "🚀 Starting Real Isaac Sim Service on Brev Instance..."

# Set Isaac Sim Python path
export PYTHONPATH="~/isaac-sim/isaac-sim-2023.1.1:$PYTHONPATH"

# Navigate to service directory
cd ~/sepulki/services/anvil-sim

# Test Isaac Sim integration
echo "🧪 Testing Isaac Sim integration..."
python3 test_real_isaac_sim.py

if [ $? -eq 0 ]; then
    echo "✅ Isaac Sim integration test passed"
else
    echo "❌ Isaac Sim integration test failed"
    exit 1
fi

# Start the service
echo "🎬 Starting Real Isaac Sim service..."
python3 src/main.py

echo "✅ Real Isaac Sim service started successfully!"
