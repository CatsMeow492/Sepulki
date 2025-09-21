#!/bin/bash
# Isaac Sim Integration Demo Startup Script
# Starts the complete Sepulki + Isaac Sim development environment

set -e

echo "🌟 Starting Isaac Sim Integration Demo"
echo "======================================"

# Check if we're in the correct directory
if [[ ! -f "package.json" ]] || [[ ! -d "services/anvil-sim" ]]; then
    echo "❌ Please run this script from the Sepulki project root"
    exit 1
fi

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop."
    exit 1
fi

echo "📋 Demo Configuration:"
echo "   Frontend:     Forge UI with Enhanced 3D rendering"
echo "   Backend:      Hammer Orchestrator (GraphQL)"
echo "   Simulation:   Anvil Sim (Isaac Sim Integration)"
echo "   Rendering:    Intelligent fallback (Isaac Sim → Three.js)"
echo ""

# Start Anvil Sim service first
echo "🔧 Starting Anvil Sim service..."
cd services/anvil-sim
./scripts/dev-start.sh
cd ../..

echo ""
echo "⏳ Waiting for Anvil Sim to be ready..."
sleep 5

# Start main application services
echo "🚀 Starting Sepulki Forge application..."

# Check if npm dependencies are installed
if [[ ! -d "node_modules" ]]; then
    echo "📦 Installing dependencies..."
    npm install
fi

# Start the main application
echo "🎨 Starting Forge UI and Hammer Orchestrator..."
npm run dev &
APP_PID=$!

echo ""
echo "⏳ Waiting for services to initialize..."
sleep 10

# Check service health
echo "🔍 Checking service health..."

# Check Forge UI
if curl -f http://localhost:3000 > /dev/null 2>&1; then
    echo "✅ Forge UI is running"
else
    echo "⚠️  Forge UI may still be starting..."
fi

# Check Hammer Orchestrator
if curl -f http://localhost:4000/graphql > /dev/null 2>&1; then
    echo "✅ Hammer Orchestrator is running"
else
    echo "⚠️  Hammer Orchestrator may still be starting..."
fi

# Check Anvil Sim
if curl -f http://localhost:8002/health > /dev/null 2>&1; then
    echo "✅ Anvil Sim is running"
else
    echo "⚠️  Anvil Sim may still be starting..."
fi

echo ""
echo "🎉 Isaac Sim Integration Demo is Ready!"
echo "====================================="
echo ""
echo "🌐 Access Points:"
echo "   Forge UI:          http://localhost:3000"
echo "   Configure Page:    http://localhost:3000/configure"
echo "   My Designs:        http://localhost:3000/designs"
echo "   GraphQL API:       http://localhost:4000/graphql"
echo ""
echo "🔧 Isaac Sim Services:"
echo "   Health Check:      http://localhost:8002/health"
echo "   WebRTC Streaming:  ws://localhost:8001"
echo "   Metrics:           http://localhost:9090"
echo "   Dashboard:         http://localhost:3000 (Grafana)"
echo ""
echo "🎯 Demo Workflow:"
echo "   1. Open http://localhost:3000/configure"
echo "   2. Enter robot requirements (e.g., '25kg warehouse automation')"
echo "   3. Click 'Analyze with AI' → See AI analysis"
echo "   4. Configure robot parameters"
echo "   5. Watch Enhanced 3D rendering (automatically chooses best renderer)"
echo "   6. Save design (sign in required)"
echo "   7. View in My Designs dashboard"
echo ""
echo "🎪 Rendering Modes:"
echo "   • macOS: Automatically uses Three.js (Isaac Sim unavailable)"
echo "   • Linux+GPU: Automatically uses Isaac Sim streaming"
echo "   • Manual: Toggle renderer in 3D viewport settings"
echo ""
echo "🔧 Development:"
echo "   • Frontend changes: Auto-reload enabled"
echo "   • Backend changes: Restart with 'npm run dev'"
echo "   • Isaac Sim logs: 'cd services/anvil-sim && docker-compose logs -f'"
echo ""
echo "🛑 To Stop All Services:"
echo "   Press Ctrl+C or run: ./stop-isaac-sim-demo.sh"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Stopping Isaac Sim Integration Demo..."
    
    # Stop main application
    if [[ -n "$APP_PID" ]]; then
        kill $APP_PID 2>/dev/null || true
    fi
    
    # Stop Anvil Sim services
    cd services/anvil-sim
    docker-compose down
    cd ../..
    
    echo "✅ All services stopped"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Keep script running
echo "🔄 Demo is running... Press Ctrl+C to stop"
wait $APP_PID
