#!/bin/bash

# Sepulki Development Tools
# Convenient scripts for local development workflow

set -e

echo "🔥 Sepulki Development Tools"
echo "============================"

# Function to check service health
check_service() {
    local service_name=$1
    local port=$2
    local endpoint=${3:-"/health"}
    
    echo -n "📡 Checking $service_name (port $port)... "
    
    if curl -s -f "http://localhost:$port$endpoint" > /dev/null 2>&1; then
        echo "✅ Healthy"
        return 0
    else
        echo "❌ Not responding"
        return 1
    fi
}

# Function to start all services
start_all() {
    echo "🚀 Starting Sepulki development environment..."
    
    # Start infrastructure
    echo "📦 Starting infrastructure services..."
    docker-compose up -d postgres redis minio influxdb
    
    # Start auth services  
    echo "🔐 Starting local authentication services..."
    docker-compose up -d local-auth mailhog fake-sms
    
    # Wait for services to be ready
    echo "⏳ Waiting for services to initialize..."
    sleep 10
    
    # Check database is ready
    until docker exec sepulki-postgres-1 pg_isready -U smith -d sepulki > /dev/null 2>&1; do
        echo "   Waiting for PostgreSQL..."
        sleep 2
    done
    
    echo "✅ Infrastructure ready!"
    
    # Setup authentication services
    echo "🔧 Setting up local authentication..."
    ./scripts/setup-local-auth.sh
    
    # Start application services
    echo "🔨 Starting Hammer Orchestrator (GraphQL API)..."
    cd services/hammer-orchestrator
    DATABASE_URL=postgresql://smith:forge_dev@localhost:5432/sepulki REDIS_URL=redis://localhost:6379 npm run dev &
    HAMMER_PID=$!
    cd ../..
    
    echo "⚒️ Starting Forge UI (Frontend)..."
    cd apps/forge-ui
    npm run dev &
    FORGE_PID=$!
    cd ../..
    
    # Save PIDs for cleanup
    echo $HAMMER_PID > .dev-pids/hammer.pid
    echo $FORGE_PID > .dev-pids/forge.pid
    
    echo "⏳ Waiting for services to start..."
    sleep 8
    
    # Check service health
    echo "🏥 Health Check:"
    check_service "Forge UI" 3000 "/"
    check_service "Hammer Orchestrator" 4000 "/health"
    
    echo ""
    echo "🎉 Development environment ready!"
    echo ""
    echo "📍 Access Points:"
    echo "   🔥 Forge UI:         http://localhost:3000"
    echo "   🔨 GraphQL API:      http://localhost:4000/graphql"  
    echo "   📦 MinIO Console:    http://localhost:9001 (sepulki / vault_dev_key)"
    echo "   🗄️ PostgreSQL:       localhost:5432 (smith / forge_dev)"
    echo ""
    echo "🔐 Authentication Services:"
    echo "   🔑 Local Auth:       http://localhost:4446 (LocalStack for Auth.js)"
    echo "   📧 Email Capture:    http://localhost:8025"  
    echo "   📱 SMS Mock:         http://localhost:1081"
    echo ""
    echo "👥 Test Users (Database):"
    echo "   🔧 dev@sepulki.com (dev123) - Over-Smith"
    echo "   🎯 demo@sepulki.com (demo123) - Smith"
    echo "   🧪 test@sepulki.com (test123) - Over-Smith"
    echo "   👑 admin@sepulki.com (admin123) - Admin"
    echo ""
    echo "👥 OAuth Test Users:"
    echo "   📧 admin@sepulki.local (admin123) - Admin"
    echo "   📧 dev@sepulki.local (dev123) - Over-Smith"
    echo "   📧 test@sepulki.local (test123) - Smith"
}

# Function to stop all services
stop_all() {
    echo "🛑 Stopping Sepulki development environment..."
    
    # Kill application processes
    if [ -f .dev-pids/hammer.pid ]; then
        kill $(cat .dev-pids/hammer.pid) 2>/dev/null || true
        rm .dev-pids/hammer.pid
    fi
    
    if [ -f .dev-pids/forge.pid ]; then
        kill $(cat .dev-pids/forge.pid) 2>/dev/null || true
        rm .dev-pids/forge.pid
    fi
    
    # Stop infrastructure
    docker-compose down
    
    echo "✅ All services stopped"
}

# Function to show service status
status() {
    echo "📊 Service Status"
    echo "================="
    
    echo "🐳 Infrastructure:"
    docker-compose ps
    echo ""
    
    echo "🏥 Health Checks:"
    check_service "Forge UI" 3000 "/" || true
    check_service "Hammer Orchestrator" 4000 "/health" || true
    check_service "MinIO API" 9000 "/minio/health/live" || true
    check_service "Redis" 6379 "" || true
    echo ""
    
    echo "🔐 Auth Services:"
    check_service "Local Auth Service" 4446 "/health" || true
    check_service "MailHog" 8025 "/" || true
    check_service "SMS Mock" 1080 "/mockserver/status" || true
    echo ""
    
    echo "💾 Database Status:"
    ./scripts/db-setup.sh status
}

# Function to reset everything (nuclear option)
reset_all() {
    echo "💥 NUCLEAR RESET - This will destroy all local data!"
    read -p "Are you sure? (type 'yes' to confirm): " confirm
    
    if [ "$confirm" != "yes" ]; then
        echo "❌ Reset cancelled"
        exit 0
    fi
    
    echo "🔥 Resetting everything..."
    
    # Stop all services
    stop_all
    
    # Remove Docker volumes (this destroys all data)
    docker-compose down -v
    
    # Reset database
    docker-compose up -d postgres redis minio
    sleep 10
    ./scripts/db-setup.sh reset
    
    echo "✅ Nuclear reset complete - environment is clean"
}

# Function to generate sample data for testing
generate_test_data() {
    echo "🧪 Generating additional test data..."
    
    # This would generate more realistic test data
    # For now, we'll add a few more entities
    
    docker exec sepulki-postgres-1 psql -U smith -d sepulki << 'EOF'
-- Add more test robots
INSERT INTO robots (name, sepulka_id, fleet_id, current_ingot_id, status, battery_level, health_score) VALUES 
(
  'TestBot-Gamma',
  (SELECT id FROM sepulkas WHERE name = 'DevBot-001'),
  (SELECT id FROM fleets WHERE name = 'Dev Fleet Alpha'),
  (SELECT id FROM ingots WHERE build_hash = 'build_dev001_abc123'),
  'MAINTENANCE',
  15.2,
  45.8
),
(
  'TestBot-Delta',
  (SELECT id FROM sepulkas WHERE name = 'AssemblyBot-Test'),
  (SELECT id FROM fleets WHERE name = 'Factory Test Units'),
  NULL,
  'OFFLINE',
  0.0,
  0.0
);

-- Add more tasks
INSERT INTO tasks (name, description, type, parameters, status, priority, created_by) VALUES 
(
  'Stress Test Assembly',
  'High-frequency assembly operations for performance testing',
  'ASSEMBLY',
  '{"cycles": 1000, "speed": "maximum", "precision": 0.001}',
  'PENDING',
  'LOW',
  (SELECT id FROM smiths WHERE email = 'dev@sepulki.com')
),
(
  'Emergency Response Test',
  'Test emergency stop and safety systems',
  'MAINTENANCE',
  '{"test_type": "emergency_stop", "scenarios": ["power_loss", "collision", "force_limit"]}',
  'PENDING',
  'URGENT',
  (SELECT id FROM smiths WHERE email = 'test@sepulki.com')
);

SELECT 'Additional test data generated!' as message;
EOF

    echo "✅ Test data generated"
}

# Create PID directory
mkdir -p .dev-pids

# Main script logic
case "${1:-help}" in
    "start")
        start_all
        ;;
    "stop")
        stop_all
        ;;
    "restart")
        stop_all
        sleep 2
        start_all
        ;;
    "status")
        status
        ;;
    "reset")
        reset_all
        ;;
    "test-data")
        generate_test_data
        ;;
    "logs")
        service=${2:-all}
        if [ "$service" = "all" ]; then
            docker-compose logs -f
        else
            docker-compose logs -f "$service"
        fi
        ;;
    "db")
        shift
        ./scripts/db-setup.sh "$@"
        ;;
    "help"|*)
        echo "Usage: $0 {start|stop|restart|status|reset|test-data|logs|db|help}"
        echo ""
        echo "🔥 Main Commands:"
        echo "  start     - Start complete development environment"
        echo "  stop      - Stop all services"
        echo "  restart   - Stop and start all services"
        echo "  status    - Show status of all services"
        echo "  reset     - Nuclear reset (destroys all data)"
        echo ""
        echo "🛠️ Utility Commands:"
        echo "  test-data - Generate additional test data"
        echo "  logs [service] - Show logs (all services or specific service)"
        echo "  db <cmd>  - Database management (check|reset|migrate|seed|status)"
        echo ""
        echo "📍 Quick Examples:"
        echo "  $0 start           # Start everything"
        echo "  $0 db status       # Check database"
        echo "  $0 logs hammer     # View GraphQL logs"
        echo "  $0 test-data       # Add more test data"
        echo ""
        echo "🎯 After starting, access:"
        echo "  • Forge UI: http://localhost:3000"
        echo "  • GraphQL: http://localhost:4000/graphql"
        echo "  • MinIO: http://localhost:9001"
        ;;
esac
