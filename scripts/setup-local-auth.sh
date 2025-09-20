#!/bin/bash

# Setup Local Authentication Services
# Initializes OAuth provider, creates test users, and configures services

set -e

echo "🔐 Setting up local authentication services..."

# Wait for services to be ready
echo "⏳ Waiting for services to start..."
sleep 10

# Check if Hydra is ready
check_hydra() {
    curl -f http://localhost:4445/health/ready > /dev/null 2>&1
}

# Wait for Hydra to be ready
until check_hydra; do
    echo "   Waiting for Hydra OAuth server..."
    sleep 2
done

echo "✅ Hydra OAuth server is ready"

# Create OAuth client for Sepulki
echo "🔧 Creating OAuth client..."
HYDRA_ADMIN_URL="http://localhost:4445"

# Check if client already exists
if curl -f "${HYDRA_ADMIN_URL}/admin/clients/sepulki-forge" > /dev/null 2>&1; then
    echo "   OAuth client already exists, updating..."
    curl -X PUT "${HYDRA_ADMIN_URL}/admin/clients/sepulki-forge" \
        -H "Content-Type: application/json" \
        -d '{
            "client_id": "sepulki-forge",
            "client_secret": "sepulki-forge-secret",
            "grant_types": ["authorization_code", "refresh_token"],
            "response_types": ["code"],
            "scope": "openid email profile offline_access",
            "redirect_uris": [
                "http://localhost:3000/api/auth/callback/local-oauth",
                "http://localhost:3000/api/auth/callback/mock"
            ],
            "post_logout_redirect_uris": ["http://localhost:3000"],
            "client_name": "Sepulki Forge",
            "logo_uri": "http://localhost:3000/favicon.ico"
        }'
else
    echo "   Creating new OAuth client..."
    curl -X POST "${HYDRA_ADMIN_URL}/admin/clients" \
        -H "Content-Type: application/json" \
        -d '{
            "client_id": "sepulki-forge",
            "client_secret": "sepulki-forge-secret", 
            "grant_types": ["authorization_code", "refresh_token"],
            "response_types": ["code"],
            "scope": "openid email profile offline_access",
            "redirect_uris": [
                "http://localhost:3000/api/auth/callback/local-oauth",
                "http://localhost:3000/api/auth/callback/mock"
            ],
            "post_logout_redirect_uris": ["http://localhost:3000"],
            "client_name": "Sepulki Forge",
            "logo_uri": "http://localhost:3000/favicon.ico"
        }'
fi

echo "✅ OAuth client configured"

# Check MailHog
echo "📧 Checking MailHog email service..."
if curl -f http://localhost:8025 > /dev/null 2>&1; then
    echo "✅ MailHog is ready at http://localhost:8025"
else
    echo "⚠️  MailHog not accessible - emails will not be captured"
fi

# Check MockServer for SMS
echo "📱 Checking SMS mock service..."
if curl -f http://localhost:1080/mockserver/status > /dev/null 2>&1; then
    echo "✅ SMS MockServer is ready at http://localhost:1080"
else
    echo "⚠️  SMS MockServer not accessible - SMS testing unavailable"
fi

echo ""
echo "🎉 Local authentication setup complete!"
echo ""
echo "📍 Service Access Points:"
echo "   🔑 OAuth Provider:   http://localhost:4444"
echo "   🖥️  OAuth Login UI:   http://localhost:3001"
echo "   📧 Email Capture:    http://localhost:8025"  
echo "   📱 SMS Mock:         http://localhost:1081"
echo "   🔥 Your App:         http://localhost:3000"
echo ""
echo "🧪 Test Users (for OAuth login):"
echo "   📧 admin@sepulki.local   (password: admin123) - Admin"
echo "   📧 dev@sepulki.local     (password: dev123)   - Over-Smith"
echo "   📧 test@sepulki.local    (password: test123)  - Smith"
echo ""
echo "💡 Next steps:"
echo "   1. Start your app: npm run dev:forge"
echo "   2. Go to: http://localhost:3000/auth/signin"
echo "   3. Try 'Sign in with Local OAuth'"
echo "   4. Use any of the test users above"
echo ""
echo "For more details, see: dev-services/README.md"
