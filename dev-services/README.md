# 🔐 Local Authentication Services

This directory contains configurations for local authentication services - the "LocalStack equivalent" for Auth.js development.

## 🚀 Services Overview

### 1. **Local OAuth2/OIDC Provider (Ory Hydra)**
- **Purpose**: Replaces GitHub/Google/other OAuth providers locally
- **Access**: http://localhost:4444 (public), http://localhost:4445 (admin)
- **Use**: Test OAuth flows without external dependencies

### 2. **OAuth UI (Login/Consent)**
- **Purpose**: Provides login and consent screens for OAuth flows
- **Access**: http://localhost:3001
- **Use**: Complete OAuth user experience locally

### 3. **MailHog (Email Service)**
- **Purpose**: Catches all outbound emails locally (like local SendGrid/SES)
- **Access**: http://localhost:8025 (web UI), localhost:1025 (SMTP)
- **Use**: Test email verification, password resets, notifications

### 4. **MockServer (SMS Service)**
- **Purpose**: Mocks SMS providers like Twilio for 2FA testing
- **Access**: http://localhost:1080 (API), http://localhost:1081 (dashboard)
- **Use**: Test SMS-based 2FA without real SMS costs

## 🛠️ Setup Instructions

### 1. Start All Auth Services
```bash
# Start infrastructure + auth services
npm run docker:auth-up

# Or start everything
npm run dev
```

### 2. Initialize OAuth Provider
```bash
# Create OAuth client for your app
curl -X POST http://localhost:4445/admin/clients \
  -H "Content-Type: application/json" \
  -d '{
    "client_id": "sepulki-forge",
    "client_secret": "sepulki-forge-secret",
    "grant_types": ["authorization_code", "refresh_token"],
    "response_types": ["code"],
    "scope": "openid email profile",
    "redirect_uris": ["http://localhost:3000/api/auth/callback/local-oauth"]
  }'
```

### 3. Configure Auth.js
Update your `.env.local`:
```bash
# Local OAuth Provider
LOCAL_OAUTH_CLIENT_ID=sepulki-forge
LOCAL_OAUTH_CLIENT_SECRET=sepulki-forge-secret
LOCAL_OAUTH_ISSUER=http://localhost:4444

# Local Email Service
EMAIL_SERVER=smtp://localhost:1025
EMAIL_FROM=noreply@sepulki.local

# Local SMS Service
SMS_PROVIDER_URL=http://localhost:1080/sms/send
```

## 🎯 Usage Examples

### OAuth Flow Testing
1. Go to http://localhost:3000/auth/signin
2. Click "Sign in with Local OAuth"
3. Complete login at http://localhost:3001
4. Get redirected back with real OAuth tokens

### Email Testing
1. Trigger password reset
2. Check http://localhost:8025 for the email
3. Click reset link and complete flow

### SMS Testing
1. Enable 2FA in your app
2. Enter phone number
3. Check MockServer dashboard for "sent" SMS
4. Use any 6-digit code to complete (it's mocked)

## 📊 Service Endpoints

| Service | Purpose | Web UI | API |
|---------|---------|--------|-----|
| **Hydra OAuth** | OAuth2/OIDC Provider | http://localhost:3001 | http://localhost:4444 |
| **MailHog** | Email Capture | http://localhost:8025 | smtp://localhost:1025 |
| **MockServer** | SMS Mock | http://localhost:1081 | http://localhost:1080 |
| **PostgreSQL** | All Data | pgAdmin or CLI | postgresql://localhost:5432 |
| **Redis** | Sessions/Cache | RedisInsight | redis://localhost:6379 |

## 🔧 Advanced Configuration

### Custom OAuth Users
Create test users directly in the OAuth UI or via API:

```javascript
// Create test user
const response = await fetch('http://localhost:4444/admin/identities', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    schema_id: 'default',
    traits: {
      email: 'test@sepulki.local',
      name: 'Test Smith'
    },
    credentials: {
      password: {
        config: { hashed_password: 'test123' }
      }
    }
  })
});
```

### Email Templates
MailHog captures all emails - perfect for testing:
- Password reset flows
- Email verification
- Welcome emails
- Notification emails

### SMS Responses
MockServer can simulate different SMS provider responses:
- Success: 200 with message ID
- Rate limit: 429 Too Many Requests
- Invalid number: 400 Bad Request
- Service error: 500 Internal Server Error

## 🎉 Benefits

✅ **Zero External Dependencies**: No GitHub OAuth apps needed  
✅ **Faster Development**: No API rate limits or network delays  
✅ **Complete Control**: Mock any authentication scenario  
✅ **Cost Effective**: No SMS/email charges during development  
✅ **Realistic Testing**: Full OAuth flows with real tokens  
✅ **Team Consistency**: Everyone has identical auth environment  

This setup gives you the same level of control over authentication that LocalStack provides for AWS services!
