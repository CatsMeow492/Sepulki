#!/bin/bash

# Run Anvil Sim service within Isaac Sim environment
# This script should be run inside the Isaac Sim container

set -e

echo "🚀 Starting Anvil Sim service within Isaac Sim..."

# Set up the Isaac Sim environment
export PYTHONPATH="/isaac-sim/kit/python:/isaac-sim/kit/exts:/isaac-sim/kit/extscore:/isaac-sim/kit/kernel:/isaac-sim/exts:$PYTHONPATH"

# Run the Anvil Sim service as an Isaac Sim application
/isaac-sim/kit/python/bin/python3.11 /home/shadeform/sepulki/services/anvil-sim/src/anvil_sim_isaac_app.py

echo "✅ Anvil Sim service completed"
