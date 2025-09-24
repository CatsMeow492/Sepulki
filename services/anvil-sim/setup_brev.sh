#!/bin/bash

# 🚀 Anvil Sim - Brev Setup Script
# One-command setup for Isaac Sim integration on Brev instances
# 
# Usage: ./setup_brev.sh [dev|prod]
#   dev  - Development mode (mock Isaac Sim, fast startup)
#   prod - Production mode (real Isaac Sim, requires GPU)

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MODE="${1:-dev}"
PUBLIC_IP=$(curl -s ifconfig.me 2>/dev/null || echo "localhost")

echo "🔥 Anvil Sim - Brev Setup"
echo "========================="
echo "Mode: $MODE"
echo "Public IP: $PUBLIC_IP"
echo "Project Root: $PROJECT_ROOT"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Check system requirements
check_requirements() {
    log_info "Checking system requirements..."
    
    # Check if we're on Brev (has NVIDIA GPU)
    if nvidia-smi >/dev/null 2>&1; then
        GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader,nounits | head -1)
        log_success "NVIDIA GPU detected: $GPU_INFO"
        HAS_GPU=true
    else
        log_warning "No NVIDIA GPU detected - will run in development mode"
        HAS_GPU=false
    fi
    
    # Check Docker
    if docker info >/dev/null 2>&1; then
        log_success "Docker is running"
    else
        log_error "Docker is not running. Please start Docker."
        exit 1
    fi
    
    # Check Docker Compose
    if docker-compose --version >/dev/null 2>&1; then
        log_success "Docker Compose is available"
    else
        log_info "Installing Docker Compose..."
        sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        sudo chmod +x /usr/local/bin/docker-compose
        
        # Verify installation
        if docker-compose --version >/dev/null 2>&1; then
            log_success "Docker Compose installed successfully"
        else
            log_error "Failed to install Docker Compose"
            exit 1
        fi
    fi
    
    # Check Docker GPU support
    if docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi >/dev/null 2>&1; then
        log_success "Docker GPU support available"
        DOCKER_GPU=true
    else
        log_warning "Docker GPU support not available"
        DOCKER_GPU=false
    fi
    
    # Check Python
    if python3 --version >/dev/null 2>&1; then
        PYTHON_VERSION=$(python3 --version)
        log_success "Python available: $PYTHON_VERSION"
    else
        log_error "Python 3 not found"
        exit 1
    fi
    
    # Check if we're in the right directory
    if [[ ! -f "$SCRIPT_DIR/docker-compose.yml" ]]; then
        log_error "Please run this script from the anvil-sim directory"
        exit 1
    fi
}

# Install Python dependencies
install_dependencies() {
    log_info "Installing Python dependencies..."
    
    # Check if pip3 is available
    if ! command -v pip3 >/dev/null 2>&1; then
        log_error "pip3 not found. Please install Python 3 with pip."
        exit 1
    fi
    
    # Install required packages
    pip3 install --upgrade pip --user
    pip3 install --user -r requirements.txt
    
    # Add user bin to PATH if needed
    if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
        export PATH="$HOME/.local/bin:$PATH"
        log_info "Added $HOME/.local/bin to PATH"
    fi
    
    log_success "Python dependencies installed"
}

# Setup Isaac Sim for production mode
setup_isaac_sim() {
    if [[ "$MODE" != "prod" ]]; then
        log_info "Skipping Isaac Sim setup (development mode)"
        return
    fi
    
    log_info "Setting up Isaac Sim for production mode..."
    
    # Check if Isaac Sim is already installed
    ISAAC_SIM_PATH="/home/shadeform/isaac-sim"
    if [[ -d "$ISAAC_SIM_PATH" ]]; then
        log_success "Isaac Sim already installed at $ISAAC_SIM_PATH"
        return
    fi
    
    log_info "Isaac Sim not found. Setting up Isaac Sim container..."
    
    # Create directories
    mkdir -p ~/docker/isaac-sim/cache/kit
    mkdir -p ~/docker/isaac-sim/logs
    
    # Stop any existing Isaac Sim container
    docker stop isaac-sim 2>/dev/null || true
    docker rm isaac-sim 2>/dev/null || true
    
    # Start Isaac Sim container
    log_info "Starting Isaac Sim container (this may take a few minutes)..."
    docker run --name isaac-sim -d --runtime=nvidia --gpus all \
      -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
      -p 49100:49100 \
      -p 8765:8765 \
      --rm \
      -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
      -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
      nvcr.io/nvidia/isaac-sim:5.0.0 \
      ./runheadless.sh \
      --/app/livestream/publicEndpointAddress=$PUBLIC_IP \
      --/app/livestream/port=49100 \
      --/app/websocket/port=8765
    
    CONTAINER_ID=$(docker ps -q --filter name=isaac-sim)
    log_success "Isaac Sim container started (ID: $CONTAINER_ID)"
    
    # Wait for Isaac Sim to initialize
    log_info "Waiting for Isaac Sim to initialize..."
    sleep 30
    
    # Test connectivity
    if docker exec isaac-sim ls /isaac-sim/kit/python/ >/dev/null 2>&1; then
        log_success "Isaac Sim container is accessible"
    else
        log_warning "Isaac Sim container may not be fully ready yet"
    fi
}

# Create environment configuration
create_env_config() {
    log_info "Creating environment configuration..."
    
    # Generate secure JWT secret
    JWT_SECRET=$(openssl rand -hex 32 2>/dev/null || echo "dev-jwt-secret-$(date +%s)")
    
    # Create .env file based on mode
    if [[ "$MODE" == "prod" ]]; then
        ENV_FILE=".env.prod"
        log_info "Creating production environment configuration..."
        
        cat > "$SCRIPT_DIR/$ENV_FILE" << EOF
# Production environment configuration
COMPOSE_PROJECT_NAME=anvil-sim-prod
ANVIL_LOG_LEVEL=INFO
JWT_SECRET=$JWT_SECRET

# Isaac Sim configuration
ANVIL_HEADLESS=true
ANVIL_LIVESTREAM=true
ANVIL_WIDTH=1920
ANVIL_HEIGHT=1080
ANVIL_PHYSICS_HZ=240
ANVIL_RENDER_HZ=60

# GPU optimization
ANVIL_GPU_DYNAMICS=true
ANVIL_USE_FABRIC=true
ANVIL_CACHING=true
ANVIL_LOD=true
ANVIL_CULLING=true

# Security
ANVIL_AUTH=true
ANVIL_CORS_ORIGINS=https://forge.sepulki.com,https://sepulki.com

# Database
DATABASE_URL=postgresql://smith:forge_prod@postgres:5432/sepulki
REDIS_URL=redis://redis:6379

# Network
PUBLIC_IP=$PUBLIC_IP
ISAAC_SIM_WEBSOCKET_PORT=8765
ISAAC_SIM_LIVESTREAM_PORT=49100
EOF
    else
        ENV_FILE=".env"
        log_info "Creating development environment configuration..."
        
        cat > "$SCRIPT_DIR/$ENV_FILE" << EOF
# Development environment configuration
COMPOSE_PROJECT_NAME=anvil-sim-dev
ANVIL_LOG_LEVEL=DEBUG
JWT_SECRET=$JWT_SECRET

# Development mode (no Isaac Sim)
ANVIL_HEADLESS=false
ANVIL_LIVESTREAM=false
ANVIL_WIDTH=1280
ANVIL_HEIGHT=720
ANVIL_PHYSICS_HZ=60
ANVIL_RENDER_HZ=30

# Database
DATABASE_URL=postgresql://smith:forge_dev@postgres:5432/sepulki
REDIS_URL=redis://redis:6379

# Network
PUBLIC_IP=$PUBLIC_IP
EOF
    fi
    
    log_success "Environment configuration created: $ENV_FILE"
}

# Start services
start_services() {
    log_info "Starting services..."
    
    # Navigate to script directory
    cd "$SCRIPT_DIR"
    
    # Try to start Docker Compose services
    if [[ "$MODE" == "prod" ]]; then
        log_info "Building production images..."
        if docker-compose --env-file .env.prod build anvil-sim-prod 2>/dev/null; then
            log_info "Starting production services..."
            if docker-compose --env-file .env.prod up -d anvil-sim-prod postgres redis 2>/dev/null; then
                log_success "Production services started with Docker Compose"
                return
            fi
        fi
    else
        log_info "Starting development services..."
        if docker-compose up -d anvil-sim-dev postgres redis 2>/dev/null; then
            log_success "Development services started with Docker Compose"
            return
        fi
    fi
    
    # Fallback: Start WebSocket bridge service directly
    log_warning "Docker Compose services failed to start. Starting WebSocket bridge service directly..."
    start_websocket_bridge_direct
}

# Start WebSocket bridge service directly (fallback)
start_websocket_bridge_direct() {
    log_info "Starting WebSocket bridge service directly..."
    
    # Create the WebSocket bridge service
    cat > "$SCRIPT_DIR/websocket_bridge.py" << 'EOF'
#!/usr/bin/env python3
"""
WebSocket Bridge Service for Anvil Sim
Provides video streaming bridge between Isaac Sim and frontend
"""

import asyncio
import websockets
import json
import cv2
import numpy as np
from datetime import datetime
import logging
import signal
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class VideoStreamBridge:
    def __init__(self, port=8765):
        self.port = port
        self.connected_clients = set()
        self.frame_count = 0
        self.running = True
        
    async def start_server(self):
        """Start WebSocket server"""
        logger.info(f"🔌 Starting WebSocket bridge on port {self.port}")
        
        async def handle_client(websocket, path):
            logger.info(f"📱 New client connected: {websocket.remote_address}")
            self.connected_clients.add(websocket)
            
            try:
                # Send welcome message
                await websocket.send(json.dumps({
                    "type": "connection_established",
                    "message": "Connected to Anvil Sim WebSocket bridge",
                    "timestamp": datetime.now().isoformat()
                }))
                
                # Handle messages
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        await self.handle_message(data, websocket)
                    except json.JSONDecodeError:
                        logger.error(f"Invalid JSON: {message}")
                    except Exception as e:
                        logger.error(f"Error handling message: {e}")
                        
            except websockets.exceptions.ConnectionClosed:
                logger.info(f"Client disconnected: {websocket.remote_address}")
            finally:
                self.connected_clients.discard(websocket)
        
        # Start server
        server = await websockets.serve(handle_client, "0.0.0.0", self.port)
        logger.info(f"✅ WebSocket bridge running on port {self.port}")
        
        # Start video stream
        asyncio.create_task(self.generate_video_stream())
        
        # Keep server running
        try:
            await server.wait_closed()
        except KeyboardInterrupt:
            logger.info("🛑 Shutting down...")
            self.running = False
    
    async def handle_message(self, data, websocket):
        """Handle incoming messages"""
        message_type = data.get("type")
        
        if message_type == "start_video_stream":
            logger.info("🎬 Starting video stream")
            await websocket.send(json.dumps({
                "type": "video_stream_started",
                "message": "Video stream started",
                "timestamp": datetime.now().isoformat()
            }))
        elif message_type == "stop_video_stream":
            logger.info("⏹️ Stopping video stream")
        else:
            logger.warning(f"Unknown message type: {message_type}")
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames"""
        logger.info("🎨 Starting video frame generation...")
        
        while self.running:
            if self.connected_clients:
                try:
                    # Create mock Isaac Sim frame
                    frame = self.create_mock_frame()
                    
                    # Convert to base64
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    frame_base64 = buffer.tobytes().decode('latin-1')
                    
                    # Send to all connected clients
                    message = json.dumps({
                        "type": "video_frame",
                        "frame_data": frame_base64,
                        "frame_number": self.frame_count,
                        "timestamp": datetime.now().isoformat(),
                        "width": frame.shape[1],
                        "height": frame.shape[0]
                    })
                    
                    # Send to all clients
                    disconnected = set()
                    for client in self.connected_clients:
                        try:
                            await client.send(message)
                        except websockets.exceptions.ConnectionClosed:
                            disconnected.add(client)
                    
                    # Remove disconnected clients
                    self.connected_clients -= disconnected
                    
                    self.frame_count += 1
                    
                    if self.frame_count % 30 == 0:
                        logger.info(f"📹 Sent {self.frame_count} frames to {len(self.connected_clients)} clients")
                    
                except Exception as e:
                    logger.error(f"Error generating frame: {e}")
            
            # 15 FPS
            await asyncio.sleep(1/15)
    
    def create_mock_frame(self):
        """Create mock Isaac Sim frame"""
        # Create 1920x1080 frame
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        # Background gradient
        for y in range(1080):
            color = int(30 + (y / 1080) * 20)
            frame[y, :] = [color, color + 10, color + 20]
        
        # Isaac Sim HUD
        cv2.rectangle(frame, (10, 10), (300, 150), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (300, 150), (0, 255, 0), 2)
        cv2.putText(frame, "NVIDIA Isaac Sim", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "PHYSICS", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"Frame: {self.frame_count}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, "Robot: Franka Emika Panda", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Mock robot visualization
        center_x, center_y = 960, 540
        robot_color = (100, 150, 255)
        
        # Robot base
        cv2.circle(frame, (center_x, center_y), 50, robot_color, -1)
        
        # Animated arm
        arm_length = 100
        for i in range(3):
            angle = (self.frame_count * 2 + i * 60) % 360
            end_x = int(center_x + arm_length * np.cos(np.radians(angle)))
            end_y = int(center_y + arm_length * np.sin(np.radians(angle)))
            
            cv2.line(frame, (center_x, center_y), (end_x, end_y), robot_color, 8)
            cv2.circle(frame, (end_x, end_y), 15, robot_color, -1)
            
            center_x, center_y = end_x, end_y
        
        return frame

def signal_handler(signum, frame):
    logger.info("🛑 Received interrupt signal")
    sys.exit(0)

async def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    bridge = VideoStreamBridge()
    await bridge.start_server()

if __name__ == "__main__":
    asyncio.run(main())
EOF
    
    # Start the bridge service in background
    python3 "$SCRIPT_DIR/websocket_bridge.py" &
    BRIDGE_PID=$!
    echo $BRIDGE_PID > "$SCRIPT_DIR/websocket_bridge.pid"
    
    log_success "WebSocket bridge service started directly (PID: $BRIDGE_PID)"
}

# Start WebSocket bridge service
start_websocket_bridge() {
    log_info "Starting WebSocket bridge service..."
    
    # Create a simple WebSocket bridge for video streaming
    cat > "$SCRIPT_DIR/websocket_bridge.py" << 'EOF'
#!/usr/bin/env python3
"""
WebSocket Bridge Service for Anvil Sim
Provides video streaming bridge between Isaac Sim and frontend
"""

import asyncio
import websockets
import json
import cv2
import numpy as np
from datetime import datetime
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class VideoStreamBridge:
    def __init__(self, port=8765):
        self.port = port
        self.connected_clients = set()
        self.frame_count = 0
        
    async def start_server(self):
        """Start WebSocket server"""
        logger.info(f"🔌 Starting WebSocket bridge on port {self.port}")
        
        async def handle_client(websocket, path):
            logger.info(f"📱 New client connected: {websocket.remote_address}")
            self.connected_clients.add(websocket)
            
            try:
                # Send welcome message
                await websocket.send(json.dumps({
                    "type": "connection_established",
                    "message": "Connected to Anvil Sim WebSocket bridge",
                    "timestamp": datetime.now().isoformat()
                }))
                
                # Handle messages
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        await self.handle_message(data, websocket)
                    except json.JSONDecodeError:
                        logger.error(f"Invalid JSON: {message}")
                    except Exception as e:
                        logger.error(f"Error handling message: {e}")
                        
            except websockets.exceptions.ConnectionClosed:
                logger.info(f"Client disconnected: {websocket.remote_address}")
            finally:
                self.connected_clients.discard(websocket)
        
        # Start server
        server = await websockets.serve(handle_client, "0.0.0.0", self.port)
        logger.info(f"✅ WebSocket bridge running on port {self.port}")
        
        # Start video stream
        asyncio.create_task(self.generate_video_stream())
        
        # Keep server running
        await server.wait_closed()
    
    async def handle_message(self, data, websocket):
        """Handle incoming messages"""
        message_type = data.get("type")
        
        if message_type == "start_video_stream":
            logger.info("🎬 Starting video stream")
            await websocket.send(json.dumps({
                "type": "video_stream_started",
                "message": "Video stream started",
                "timestamp": datetime.now().isoformat()
            }))
        elif message_type == "stop_video_stream":
            logger.info("⏹️ Stopping video stream")
        else:
            logger.warning(f"Unknown message type: {message_type}")
    
    async def generate_video_stream(self):
        """Generate and broadcast video frames"""
        logger.info("🎨 Starting video frame generation...")
        
        while True:
            if self.connected_clients:
                # Create mock Isaac Sim frame
                frame = self.create_mock_frame()
                
                # Convert to base64
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_base64 = buffer.tobytes().decode('latin-1')
                
                # Send to all connected clients
                message = json.dumps({
                    "type": "video_frame",
                    "frame_data": frame_base64,
                    "frame_number": self.frame_count,
                    "timestamp": datetime.now().isoformat(),
                    "width": frame.shape[1],
                    "height": frame.shape[0]
                })
                
                # Send to all clients
                disconnected = set()
                for client in self.connected_clients:
                    try:
                        await client.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected.add(client)
                
                # Remove disconnected clients
                self.connected_clients -= disconnected
                
                self.frame_count += 1
                
                if self.frame_count % 30 == 0:
                    logger.info(f"📹 Sent {self.frame_count} frames to {len(self.connected_clients)} clients")
            
            # 15 FPS
            await asyncio.sleep(1/15)
    
    def create_mock_frame(self):
        """Create mock Isaac Sim frame"""
        # Create 1920x1080 frame
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        
        # Background gradient
        for y in range(1080):
            color = int(30 + (y / 1080) * 20)
            frame[y, :] = [color, color + 10, color + 20]
        
        # Isaac Sim HUD
        cv2.rectangle(frame, (10, 10), (300, 150), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (300, 150), (0, 255, 0), 2)
        cv2.putText(frame, "NVIDIA Isaac Sim", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "PHYSICS", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"Frame: {self.frame_count}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, "Robot: Franka Emika Panda", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Mock robot visualization
        center_x, center_y = 960, 540
        robot_color = (100, 150, 255)
        
        # Robot base
        cv2.circle(frame, (center_x, center_y), 50, robot_color, -1)
        
        # Animated arm
        arm_length = 100
        for i in range(3):
            angle = (self.frame_count * 2 + i * 60) % 360
            end_x = int(center_x + arm_length * np.cos(np.radians(angle)))
            end_y = int(center_y + arm_length * np.sin(np.radians(angle)))
            
            cv2.line(frame, (center_x, center_y), (end_x, end_y), robot_color, 8)
            cv2.circle(frame, (end_x, end_y), 15, robot_color, -1)
            
            center_x, center_y = end_x, end_y
        
        return frame

async def main():
    bridge = VideoStreamBridge()
    await bridge.start_server()

if __name__ == "__main__":
    asyncio.run(main())
EOF
    
    # Start the bridge service in background
    python3 "$SCRIPT_DIR/websocket_bridge.py" &
    BRIDGE_PID=$!
    echo $BRIDGE_PID > "$SCRIPT_DIR/websocket_bridge.pid"
    
    log_success "WebSocket bridge service started (PID: $BRIDGE_PID)"
}

# Display status and next steps
show_status() {
    echo ""
    echo "🎉 Anvil Sim Setup Complete!"
    echo "============================"
    echo ""
    
    if [[ "$MODE" == "prod" ]]; then
        echo "📊 Production Mode - Isaac Sim Integration"
        echo "  🔧 Isaac Sim Container: $(docker ps -q --filter name=isaac-sim | wc -l) running"
        echo "  🌐 Public IP: $PUBLIC_IP"
        echo "  📺 Livestream: http://$PUBLIC_IP:49100"
        echo "  🔌 WebSocket: ws://$PUBLIC_IP:8765"
    else
        echo "📊 Development Mode - Mock Isaac Sim"
        echo "  🎨 Mock Isaac Sim: Running"
        echo "  🔌 WebSocket: ws://localhost:8765"
    fi
    
    echo ""
    echo "📊 Service Endpoints:"
    echo "  WebSocket Stream: ws://$PUBLIC_IP:8765"
    echo "  Health Check:    http://localhost:8002/health (if Docker services running)"
    echo ""
    echo "🎯 Frontend Integration:"
    echo "  Local Frontend:  http://localhost:3000/isaac-video"
    echo "  Remote Frontend: http://$PUBLIC_IP:3000/isaac-video"
    echo "  WebSocket URL:   ws://$PUBLIC_IP:8765"
    echo ""
    echo "🔧 Management Commands:"
    if [[ -f "$SCRIPT_DIR/websocket_bridge.pid" ]]; then
        echo "  Stop WebSocket:  kill \$(cat websocket_bridge.pid)"
        echo "  View logs:       tail -f websocket_bridge.log"
    else
        echo "  View logs:       docker-compose logs -f"
        echo "  Stop services:   docker-compose down"
    fi
    echo "  Restart setup:   ./setup_brev.sh $MODE"
    echo ""
    echo "🚀 Ready for development!"
}

# Cleanup function
cleanup() {
    log_info "Cleaning up..."
    
    # Stop WebSocket bridge
    if [[ -f "$SCRIPT_DIR/websocket_bridge.pid" ]]; then
        BRIDGE_PID=$(cat "$SCRIPT_DIR/websocket_bridge.pid")
        kill $BRIDGE_PID 2>/dev/null || true
        rm "$SCRIPT_DIR/websocket_bridge.pid"
    fi
    
    # Stop Isaac Sim container
    docker stop isaac-sim 2>/dev/null || true
    
    log_success "Cleanup complete"
}

# Main execution
main() {
    # Set up signal handlers
    trap cleanup EXIT
    
    # Run setup steps
    check_requirements
    install_dependencies
    setup_isaac_sim
    create_env_config
    start_services
    start_websocket_bridge
    show_status
    
    log_success "Setup complete! Services are running in the background."
    log_info "Press Ctrl+C to stop all services"
    
    # Keep script running
    while true; do
        sleep 60
    done
}

# Run main function
main "$@"
