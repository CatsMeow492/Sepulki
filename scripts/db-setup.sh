#!/bin/bash

# Sepulki Database Setup Script
# Provides easy database management for local development

set -e

DB_HOST=${DB_HOST:-localhost}
DB_PORT=${DB_PORT:-5432}
DB_NAME=${DB_NAME:-sepulki}
DB_USER=${DB_USER:-smith}
DB_PASSWORD=${DB_PASSWORD:-forge_dev}

DATABASE_URL="postgresql://${DB_USER}:${DB_PASSWORD}@${DB_HOST}:${DB_PORT}/${DB_NAME}"

echo "🔥 Sepulki Database Setup"
echo "================================"

# Function to run SQL commands
run_sql() {
    docker exec sepulki-postgres-1 psql -U "$DB_USER" -d "$DB_NAME" -c "$1"
}

# Function to run SQL files
run_sql_file() {
    docker exec -i sepulki-postgres-1 psql -U "$DB_USER" -d "$DB_NAME" < "$1"
}

# Check if PostgreSQL is running
check_postgres() {
    echo "📡 Checking PostgreSQL connection..."
    if ! docker exec sepulki-postgres-1 pg_isready -U "$DB_USER" -d "$DB_NAME" > /dev/null 2>&1; then
        echo "❌ PostgreSQL is not running. Start it with: npm run docker:up"
        exit 1
    fi
    echo "✅ PostgreSQL is running"
}

# Reset database (drop and recreate all tables)
reset_db() {
    echo "🔄 Resetting database..."
    
    # Drop all tables in correct order (respecting foreign keys)
    run_sql "DROP TABLE IF EXISTS audit_stamps CASCADE;"
    run_sql "DROP TABLE IF EXISTS policy_violations CASCADE;"
    run_sql "DROP TABLE IF EXISTS runs CASCADE;"
    run_sql "DROP TABLE IF EXISTS task_robots CASCADE;"
    run_sql "DROP TABLE IF EXISTS tasks CASCADE;"
    run_sql "DROP TABLE IF EXISTS robots CASCADE;"
    run_sql "DROP TABLE IF EXISTS fleets CASCADE;"
    run_sql "DROP TABLE IF EXISTS loci CASCADE;"
    run_sql "DROP TABLE IF EXISTS ingots CASCADE;"
    run_sql "DROP TABLE IF EXISTS sepulka_alloys CASCADE;"
    run_sql "DROP TABLE IF EXISTS sepulkas CASCADE;"
    run_sql "DROP TABLE IF EXISTS patterns CASCADE;"
    run_sql "DROP TABLE IF EXISTS alloys CASCADE;"
    run_sql "DROP TABLE IF EXISTS edicts CASCADE;"
    run_sql "DROP TABLE IF EXISTS smiths CASCADE;"
    
    # Drop types
    run_sql "DROP TYPE IF EXISTS sepulka_status CASCADE;"
    run_sql "DROP TYPE IF EXISTS alloy_type CASCADE;"
    run_sql "DROP TYPE IF EXISTS pattern_category CASCADE;"
    run_sql "DROP TYPE IF EXISTS ingot_status CASCADE;"
    run_sql "DROP TYPE IF EXISTS fleet_status CASCADE;"
    run_sql "DROP TYPE IF EXISTS robot_status CASCADE;"
    run_sql "DROP TYPE IF EXISTS task_status CASCADE;"
    run_sql "DROP TYPE IF EXISTS task_priority CASCADE;"
    run_sql "DROP TYPE IF EXISTS task_type CASCADE;"
    run_sql "DROP TYPE IF EXISTS run_status CASCADE;"
    run_sql "DROP TYPE IF EXISTS smith_role CASCADE;"
    run_sql "DROP TYPE IF EXISTS edict_type CASCADE;"
    run_sql "DROP TYPE IF EXISTS edict_severity CASCADE;"
    
    echo "✅ Database reset complete"
}

# Run schema migrations
migrate() {
    echo "🗂️ Running schema migrations..."
    run_sql_file "infrastructure/sql/init.sql"
    echo "✅ Schema migration complete"
}

# Seed development data
seed() {
    echo "🌱 Seeding development data..."
    run_sql_file "scripts/seed-dev-data.sql"
    echo "✅ Development data seeded"
}

# Show database status
status() {
    echo "📊 Database Status"
    echo "=================="
    echo "Database URL: $DATABASE_URL"
    echo
    
    echo "📋 Tables:"
    run_sql "\dt"
    echo
    
    echo "👥 Smiths:"
    run_sql "SELECT id, email, name, role, is_active FROM smiths;"
    echo
    
    echo "🤖 Sepulkas:"
    run_sql "SELECT id, name, status, version FROM sepulkas;"
    echo
    
    echo "🏭 Fleets:"
    run_sql "SELECT id, name, status FROM fleets;"
    echo
    
    echo "🦾 Robots:"
    run_sql "SELECT id, name, status, fleet_id FROM robots;"
}

# Main script logic
case "${1:-help}" in
    "check")
        check_postgres
        ;;
    "reset")
        check_postgres
        reset_db
        migrate
        seed
        ;;
    "migrate")
        check_postgres
        migrate
        ;;
    "seed")
        check_postgres
        seed
        ;;
    "status")
        check_postgres
        status
        ;;
    "help"|*)
        echo "Usage: $0 {check|reset|migrate|seed|status}"
        echo ""
        echo "Commands:"
        echo "  check   - Check if PostgreSQL is running"
        echo "  reset   - Drop all tables and recreate (DESTRUCTIVE)"
        echo "  migrate - Run schema migrations"
        echo "  seed    - Seed development data"
        echo "  status  - Show database status and data"
        echo ""
        echo "Examples:"
        echo "  $0 reset    # Complete database reset with fresh data"
        echo "  $0 seed     # Add more test data"
        echo "  $0 status   # Check current state"
        ;;
esac
