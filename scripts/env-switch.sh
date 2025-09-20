#!/bin/bash

# Environment Switching Helper for Sepulki Development
# Easily switch between mock auth and real OAuth for testing

set -e

echo "🔧 Sepulki Environment Switcher"
echo "================================"

case "${1:-help}" in
    "mock")
        echo "🧪 Switching to MOCK authentication mode..."
        
        # Comment out OAuth in frontend .env.local
        sed -i.bak 's/^GITHUB_CLIENT_ID=/# GITHUB_CLIENT_ID=/' apps/forge-ui/.env.local
        sed -i.bak 's/^GITHUB_CLIENT_SECRET=/# GITHUB_CLIENT_SECRET=/' apps/forge-ui/.env.local
        sed -i.bak 's/^GOOGLE_CLIENT_ID=/# GOOGLE_CLIENT_ID=/' apps/forge-ui/.env.local
        sed -i.bak 's/^GOOGLE_CLIENT_SECRET=/# GOOGLE_CLIENT_SECRET=/' apps/forge-ui/.env.local
        
        echo "✅ Switched to mock authentication"
        echo "   - Auto-login as 'Development Smith'"
        echo "   - Over-Smith permissions"
        echo "   - No OAuth setup required"
        ;;
        
    "github")
        echo "🔐 Switching to GITHUB OAuth mode..."
        echo ""
        echo "First, set up GitHub OAuth app:"
        echo "1. Go to: https://github.com/settings/applications/new"
        echo "2. Set Authorization callback URL: http://localhost:3000/api/auth/callback/github"
        echo "3. Copy your Client ID and Secret"
        echo ""
        
        read -p "Enter GitHub Client ID: " client_id
        read -p "Enter GitHub Client Secret: " client_secret
        
        # Update frontend .env.local
        sed -i.bak "s/# GITHUB_CLIENT_ID=.*/GITHUB_CLIENT_ID=$client_id/" apps/forge-ui/.env.local
        sed -i.bak "s/# GITHUB_CLIENT_SECRET=.*/GITHUB_CLIENT_SECRET=$client_secret/" apps/forge-ui/.env.local
        
        echo "✅ Switched to GitHub OAuth"
        echo "   - Real GitHub authentication required"
        echo "   - Users must sign in with GitHub"
        ;;
        
    "status")
        echo "📊 Current Environment Status"
        echo "============================="
        
        # Check NODE_ENV
        if grep -q "NODE_ENV=development" apps/forge-ui/.env.local; then
            echo "🏷️  Environment: DEVELOPMENT"
        else
            echo "🏷️  Environment: PRODUCTION"
        fi
        
        # Check auth mode
        if grep -q "^GITHUB_CLIENT_ID=" apps/forge-ui/.env.local; then
            echo "🔐 Authentication: GITHUB OAUTH"
        elif grep -q "^GOOGLE_CLIENT_ID=" apps/forge-ui/.env.local; then
            echo "🔐 Authentication: GOOGLE OAUTH"
        else
            echo "🧪 Authentication: MOCK (Development Smith)"
        fi
        
        # Check services
        echo "🗄️  Database: $(grep DATABASE_URL .env.local | cut -d'=' -f2 | cut -d'@' -f2)"
        echo "🔨 GraphQL: $(grep GRAPHQL_ENDPOINT .env.local | cut -d'=' -f2)"
        
        ;;
        
    "production")
        echo "🚀 Production Environment Setup"
        echo "=============================="
        echo ""
        echo "To deploy to production, set these environment variables:"
        echo ""
        echo "# Required"
        echo "NODE_ENV=production"
        echo "GITHUB_CLIENT_ID=your_github_client_id"
        echo "GITHUB_CLIENT_SECRET=your_github_client_secret"
        echo "NEXTAUTH_SECRET=\$(openssl rand -base64 32)"
        echo "DATABASE_URL=postgresql://user:pass@host:port/database"
        echo ""
        echo "# Platform Examples:"
        echo ""
        echo "📦 Vercel:"
        echo "  vercel env add GITHUB_CLIENT_ID"
        echo "  vercel env add GITHUB_CLIENT_SECRET"
        echo "  vercel env add NEXTAUTH_SECRET"
        echo "  vercel env add DATABASE_URL"
        echo ""
        echo "🚂 Railway:"
        echo "  railway variables:set GITHUB_CLIENT_ID=xxx"
        echo "  railway variables:set GITHUB_CLIENT_SECRET=xxx"
        echo "  railway variables:set NEXTAUTH_SECRET=xxx"
        echo ""
        ;;
        
    "help"|*)
        echo "Usage: $0 {mock|github|status|production}"
        echo ""
        echo "Commands:"
        echo "  mock       - Switch to mock authentication (development)"
        echo "  github     - Switch to GitHub OAuth (testing)"
        echo "  status     - Show current environment configuration"
        echo "  production - Show production deployment instructions"
        echo ""
        echo "Examples:"
        echo "  $0 mock      # Use mock auth (default)"
        echo "  $0 github    # Test real GitHub OAuth locally"
        echo "  $0 status    # Check current auth mode"
        ;;
esac
