#!/bin/bash
# Stop Isaac Sim Integration Demo

echo "🛑 Stopping Isaac Sim Integration Demo"

# Stop main application processes
echo "📱 Stopping Forge UI and Hammer Orchestrator..."
pkill -f "npm run dev" || true
pkill -f "next" || true
pkill -f "node.*hammer-orchestrator" || true

# Stop Anvil Sim services
if [[ -d "services/anvil-sim" ]]; then
    echo "🔧 Stopping Anvil Sim services..."
    cd services/anvil-sim
    docker-compose down
    cd ../..
fi

# Stop any remaining Docker containers
echo "🐳 Cleaning up Docker containers..."
docker-compose down 2>/dev/null || true

echo ""
echo "✅ Isaac Sim Integration Demo stopped"
echo ""
echo "🔄 To restart the demo:"
echo "   ./start-isaac-sim-demo.sh"
