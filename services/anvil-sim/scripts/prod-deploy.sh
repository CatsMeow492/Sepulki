#!/bin/bash
# Production deployment script for Anvil Sim with Isaac Sim
# Requires NVIDIA GPU and Docker with GPU support

set -e

echo "🏭 Anvil Sim Production Deployment"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker."
    exit 1
fi

# Check if NVIDIA Docker runtime is available
if ! docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi > /dev/null 2>&1; then
    echo "❌ NVIDIA Docker runtime not available. Isaac Sim requires GPU support."
    echo "   Please install:"
    echo "   - NVIDIA drivers"
    echo "   - NVIDIA Container Toolkit"
    echo "   - Docker with GPU support"
    exit 1
fi

echo "✅ NVIDIA GPU support detected"

# Check if we're in the correct directory
if [[ ! -f "docker-compose.yml" ]]; then
    echo "❌ Please run this script from the anvil-sim directory"
    exit 1
fi

# Create production .env file
if [[ ! -f ".env.prod" ]]; then
    echo "📝 Creating production environment configuration"
    
    # Generate secure JWT secret
    JWT_SECRET=$(openssl rand -hex 32)
    
    cat > .env.prod << EOF
# Production environment configuration
COMPOSE_PROJECT_NAME=anvil-sim-prod
ANVIL_LOG_LEVEL=INFO
JWT_SECRET=${JWT_SECRET}

# Isaac Sim configuration
ANVIL_HEADLESS=true
ANVIL_LIVESTREAM=true
ANVIL_WIDTH=1920
ANVIL_HEIGHT=1080
ANVIL_PHYSICS_HZ=240
ANVIL_RENDER_HZ=60

# Performance optimization
ANVIL_GPU_DYNAMICS=true
ANVIL_USE_FABRIC=true
ANVIL_CACHING=true
ANVIL_LOD=true
ANVIL_CULLING=true

# Security
ANVIL_AUTH=true
ANVIL_CORS_ORIGINS=https://forge.sepulki.com,https://sepulki.com

# Database (update for production)
DATABASE_URL=postgresql://smith:forge_prod@postgres:5432/sepulki
REDIS_URL=redis://redis:6379
EOF
    
    echo "⚠️  Production .env.prod created with secure JWT secret"
    echo "   Please update DATABASE_URL and REDIS_URL for your production setup"
fi

echo "🔧 Building production images..."
docker-compose --env-file .env.prod build anvil-sim-prod

echo "🚀 Starting production deployment..."
docker-compose --env-file .env.prod up -d anvil-sim-prod postgres redis

echo "⏳ Waiting for services to initialize..."
sleep 10

# Wait for database
echo "🗄️ Waiting for database..."
until docker-compose --env-file .env.prod exec -T postgres pg_isready -U smith -d sepulki > /dev/null 2>&1; do
    echo "   Database is starting up..."
    sleep 5
done

echo "✅ Database is ready!"

# Wait for Anvil Sim with Isaac Sim (takes longer to initialize)
echo "🔧 Waiting for Isaac Sim to initialize..."
for i in {1..60}; do
    if curl -f http://localhost:8002/health > /dev/null 2>&1; then
        echo "✅ Isaac Sim service is ready!"
        break
    fi
    echo "   Isaac Sim initializing... ($i/60)"
    sleep 5
done

if ! curl -f http://localhost:8002/health > /dev/null 2>&1; then
    echo "⚠️  Service health check failed. Checking logs..."
    docker-compose --env-file .env.prod logs anvil-sim-prod
    echo ""
    echo "📋 Troubleshooting tips:"
    echo "   - Check GPU availability: nvidia-smi"
    echo "   - Verify Isaac Sim image: docker images | grep isaac-sim"
    echo "   - Check container logs: docker-compose logs anvil-sim-prod"
    exit 1
fi

echo ""
echo "🎉 Anvil Sim Production Deployment Complete!"
echo ""
echo "📊 Service Endpoints:"
echo "   gRPC API:        http://localhost:8000"
echo "   WebRTC Stream:   ws://localhost:8001"  
echo "   Health Check:    http://localhost:8002/health"
echo "   Isaac Sim Stream: http://localhost:8211"
echo ""
echo "🗄️ Database:"
echo "   PostgreSQL:      localhost:5432"
echo "   Redis:           localhost:6379"
echo ""
echo "📈 Monitoring:"
echo "   Prometheus:      http://localhost:9090"
echo "   Grafana:         http://localhost:3000"
echo ""
echo "🔧 Management Commands:"
echo "   View logs:       docker-compose --env-file .env.prod logs -f"
echo "   Scale service:   docker-compose --env-file .env.prod up -d --scale anvil-sim-prod=2"
echo "   Stop all:        docker-compose --env-file .env.prod down"
echo "   Update images:   docker-compose --env-file .env.prod pull && docker-compose --env-file .env.prod up -d"
echo ""
echo "🚀 Production deployment ready for Isaac Sim streaming!"
