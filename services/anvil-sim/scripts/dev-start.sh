#!/bin/bash
# Development startup script for Anvil Sim
# Starts the development environment with hot reload

set -e

echo "🔥 Starting Anvil Sim Development Environment"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop."
    exit 1
fi

# Check if we're in the correct directory
if [[ ! -f "docker-compose.yml" ]]; then
    echo "❌ Please run this script from the anvil-sim directory"
    exit 1
fi

# Create .env file if it doesn't exist
if [[ ! -f ".env" ]]; then
    echo "📝 Creating .env file for development"
    cat > .env << EOF
# Development environment configuration
COMPOSE_PROJECT_NAME=anvil-sim-dev
ANVIL_LOG_LEVEL=DEBUG
JWT_SECRET=dev-jwt-secret-not-for-production
EOF
fi

echo "🐳 Starting development containers..."
docker-compose up -d anvil-sim-dev postgres redis

echo "⏳ Waiting for services to be ready..."
sleep 5

# Wait for database to be ready
echo "🗄️ Waiting for database..."
until docker-compose exec -T postgres pg_isready -U smith -d sepulki > /dev/null 2>&1; do
    echo "   Database is starting up..."
    sleep 2
done

echo "✅ Database is ready!"

# Check if Anvil Sim service is healthy
echo "🔧 Waiting for Anvil Sim service..."
for i in {1..30}; do
    if curl -f http://localhost:8002/health > /dev/null 2>&1; then
        echo "✅ Anvil Sim service is ready!"
        break
    fi
    echo "   Service is starting up... ($i/30)"
    sleep 2
done

echo ""
echo "🎉 Anvil Sim Development Environment Ready!"
echo ""
echo "📊 Service Endpoints:"
echo "   gRPC API:        http://localhost:8000"
echo "   WebRTC Stream:   ws://localhost:8001"
echo "   Health Check:    http://localhost:8002/health"
echo "   Mock Isaac Sim:  http://localhost:8211"
echo ""
echo "🗄️ Database:"
echo "   PostgreSQL:      localhost:5432 (smith/forge_dev)"
echo "   Redis:           localhost:6379"
echo ""
echo "📈 Monitoring:"
echo "   Prometheus:      http://localhost:9090"
echo "   Grafana:         http://localhost:3000 (admin/admin)"
echo ""
echo "🔧 Development Commands:"
echo "   View logs:       docker-compose logs -f anvil-sim-dev"
echo "   Shell access:    docker-compose exec anvil-sim-dev bash"
echo "   Restart:         docker-compose restart anvil-sim-dev"
echo "   Stop all:        docker-compose down"
echo ""
echo "🚀 Ready for development! The service will auto-reload on code changes."
