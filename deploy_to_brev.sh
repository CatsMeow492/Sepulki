#!/bin/bash
# Isaac Sim Brev Deployment Script
# Deploys the Sepulki Isaac Sim integration to NVIDIA Brev instance

set -e

echo "🚀 Deploying Isaac Sim Integration to Brev Instance..."

# Check if we're connected to Brev instance
if ! command -v brev &> /dev/null; then
    echo "❌ Brev CLI not found. Please install Brev CLI first."
    exit 1
fi

# Check if instance is ready
echo "📊 Checking Brev instance status..."
INSTANCE_STATUS=$(brev ls | grep test-instance | awk '{print $2}')
if [ "$INSTANCE_STATUS" != "RUNNING" ]; then
    echo "❌ Brev instance not ready. Status: $INSTANCE_STATUS"
    echo "Please wait for instance to be ready and run: brev hello test-instance"
    exit 1
fi

echo "✅ Brev instance is ready!"

# Create deployment directory
echo "📁 Setting up deployment directory..."
mkdir -p ~/sepulki-isaac-sim
cd ~/sepulki-isaac-sim

# Clone or update the repository
if [ -d "Sepulki" ]; then
    echo "🔄 Updating existing repository..."
    cd Sepulki
    git pull origin main
else
    echo "📥 Cloning repository..."
    git clone https://github.com/your-username/Sepulki.git
    cd Sepulki
fi

# Install Isaac Sim dependencies
echo "📦 Installing Isaac Sim dependencies..."
cd services/anvil-sim

# Install Python dependencies
pip install -r requirements-minimal.txt

# Install additional Isaac Sim dependencies
pip install omni-isaac-core omni-isaac-sensor omni-isaac-ros2-bridge

# Create Isaac Sim configuration
echo "⚙️ Creating Isaac Sim configuration..."
cat > config/isaac_sim_brev_config.py << 'EOF'
#!/usr/bin/env python3
"""
Isaac Sim Configuration for Brev Deployment
"""

ISAAC_SIM_BREV_CONFIG = {
    "headless": True,
    "width": 1920,
    "height": 1080,
    "renderer": "RayTracedLighting",
    "rtx_settings": {
        "enable_sampled_direct_lighting": True,
        "enable_denoising": True,
        "max_bounces": 8,
        "samples_per_pixel": 64,
        "enable_rtx_di": True,
        "enable_rtx_gi": False
    },
    "anti_aliasing": "TAA",
    "motion_blur": False,
    "depth_of_field": False,
    "log_level": "INFO"
}

# Brev-specific settings
BREV_CONFIG = {
    "instance_name": "test-instance",
    "gpu_type": "L40S",
    "region": "us-west-2",
    "ports": [8000, 8001, 8002],
    "auto_scaling": True,
    "cost_optimization": True
}
EOF

# Create startup script
echo "🔧 Creating startup script..."
cat > start_isaac_sim_service.sh << 'EOF'
#!/bin/bash
# Start Isaac Sim Service on Brev

echo "🎬 Starting Isaac Sim Service on Brev..."

# Set environment variables
export ISAAC_SIM_AVAILABLE=true
export BREV_DEPLOYMENT=true
export GPU_ENABLED=true

# Start the service
cd ~/sepulki-isaac-sim/Sepulki/services/anvil-sim
python3 src/main.py
EOF

chmod +x start_isaac_sim_service.sh

# Create systemd service for auto-start
echo "🔧 Creating systemd service..."
sudo tee /etc/systemd/system/isaac-sim-sepulki.service > /dev/null << EOF
[Unit]
Description=Isaac Sim Sepulki Service
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/sepulki-isaac-sim/Sepulki/services/anvil-sim
ExecStart=/home/ubuntu/sepulki-isaac-sim/Sepulki/services/anvil-sim/start_isaac_sim_service.sh
Restart=always
RestartSec=10
Environment=ISAAC_SIM_AVAILABLE=true
Environment=BREV_DEPLOYMENT=true
Environment=GPU_ENABLED=true

[Install]
WantedBy=multi-user.target
EOF

# Enable and start the service
echo "🚀 Enabling Isaac Sim service..."
sudo systemctl daemon-reload
sudo systemctl enable isaac-sim-sepulki.service

# Create health check script
echo "🏥 Creating health check script..."
cat > health_check.sh << 'EOF'
#!/bin/bash
# Health check for Isaac Sim service

echo "🏥 Checking Isaac Sim service health..."

# Check if service is running
if systemctl is-active --quiet isaac-sim-sepulki; then
    echo "✅ Isaac Sim service is running"
    
    # Check HTTP endpoints
    if curl -s http://localhost:8002/health > /dev/null; then
        echo "✅ Health endpoint responding"
    else
        echo "❌ Health endpoint not responding"
        exit 1
    fi
    
    # Check WebSocket endpoint
    if nc -z localhost 8001; then
        echo "✅ WebSocket endpoint responding"
    else
        echo "❌ WebSocket endpoint not responding"
        exit 1
    fi
    
    echo "🎉 All health checks passed!"
else
    echo "❌ Isaac Sim service is not running"
    exit 1
fi
EOF

chmod +x health_check.sh

# Create monitoring script
echo "📊 Creating monitoring script..."
cat > monitor_service.sh << 'EOF'
#!/bin/bash
# Monitor Isaac Sim service

echo "📊 Isaac Sim Service Monitor"
echo "=========================="

while true; do
    clear
    echo "📊 Isaac Sim Service Monitor - $(date)"
    echo "=========================="
    
    # Service status
    echo "🔧 Service Status:"
    systemctl status isaac-sim-sepulki --no-pager -l
    
    echo ""
    echo "💾 Resource Usage:"
    # GPU usage
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits
    fi
    
    # CPU and memory
    top -bn1 | grep "Cpu\|Mem\|Swap"
    
    echo ""
    echo "🌐 Network Status:"
    netstat -tlnp | grep -E ":(8000|8001|8002)"
    
    echo ""
    echo "📝 Recent Logs:"
    journalctl -u isaac-sim-sepulki --no-pager -n 5
    
    sleep 5
done
EOF

chmod +x monitor_service.sh

# Create deployment verification script
echo "✅ Creating deployment verification script..."
cat > verify_deployment.sh << 'EOF'
#!/bin/bash
# Verify Isaac Sim deployment

echo "🔍 Verifying Isaac Sim Deployment..."

# Check Isaac Sim installation
echo "1. Checking Isaac Sim installation..."
if python3 -c "import omni; print('Isaac Sim available')" 2>/dev/null; then
    echo "✅ Isaac Sim is installed and available"
else
    echo "❌ Isaac Sim is not available"
    exit 1
fi

# Check GPU availability
echo "2. Checking GPU availability..."
if nvidia-smi &>/dev/null; then
    echo "✅ GPU is available"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
else
    echo "❌ GPU is not available"
    exit 1
fi

# Check service files
echo "3. Checking service files..."
if [ -f "src/isaac_sim_real_renderer.py" ]; then
    echo "✅ Isaac Sim renderer found"
else
    echo "❌ Isaac Sim renderer not found"
    exit 1
fi

if [ -f "src/isaac_sim_assets.py" ]; then
    echo "✅ Isaac Sim assets configuration found"
else
    echo "❌ Isaac Sim assets configuration not found"
    exit 1
fi

# Test basic Isaac Sim functionality
echo "4. Testing Isaac Sim functionality..."
python3 -c "
import sys
sys.path.append('src')
from isaac_sim_real_renderer import IsaacSimRealRenderer
renderer = IsaacSimRealRenderer()
print('✅ Isaac Sim renderer initialized successfully')
"

echo "🎉 Deployment verification completed successfully!"
EOF

chmod +x verify_deployment.sh

# Run deployment verification
echo "🔍 Running deployment verification..."
./verify_deployment.sh

if [ $? -eq 0 ]; then
    echo "🎉 Isaac Sim deployment completed successfully!"
    echo ""
    echo "📋 Next Steps:"
    echo "1. Start the service: sudo systemctl start isaac-sim-sepulki"
    echo "2. Check status: sudo systemctl status isaac-sim-sepulki"
    echo "3. Monitor service: ./monitor_service.sh"
    echo "4. Health check: ./health_check.sh"
    echo ""
    echo "🌐 Service Endpoints:"
    echo "   Health Check: http://localhost:8002/health"
    echo "   WebSocket: ws://localhost:8001"
    echo "   gRPC API: http://localhost:8000"
    echo ""
    echo "📊 To start the service now:"
    echo "   sudo systemctl start isaac-sim-sepulki"
else
    echo "❌ Deployment verification failed. Please check the errors above."
    exit 1
fi

