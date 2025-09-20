# 🔐 Sepulki Authentication Setup

Sepulki uses NextAuth.js v5 with a developer-friendly configuration that works seamlessly in both local development and production environments.

## 🚀 Quick Start (Development)

**No setup required!** The authentication system automatically uses a mock "Development Smith" account in development mode when no GitHub credentials are provided.

Simply start the development server and you'll be automatically authenticated as:
- **Name**: Development Smith
- **Email**: dev@sepulki.com  
- **Role**: OVER_SMITH (elevated permissions)
- **Avatar**: Default developer avatar

## 🔧 Development Features

### 1. **Mock Authentication Provider**
- Bypasses OAuth flow completely in development
- Provides a realistic user experience without external dependencies
- Includes all necessary user data (name, email, role, avatar)

### 2. **Environment Detection**
```typescript
const isDevelopment = process.env.NODE_ENV === "development"
const useDevAuth = isDevelopment && !process.env.GITHUB_CLIENT_ID
```

### 3. **Role-Based Permissions**
Uses Sepulki's metallurgy-themed role system:
- **🔨 SMITH**: Basic user permissions
- **⚒️ OVER_SMITH**: Advanced permissions  
- **👑 ADMIN**: Full system access

### 4. **Beautiful Sign-In UI**
- Sepulki-branded authentication pages
- Automatic development mode detection
- Clear setup instructions for production OAuth

## 🏗️ Production Setup (Optional)

For production-like testing with real GitHub OAuth:

### 1. Create GitHub OAuth App
1. Go to [github.com/settings/applications/new](https://github.com/settings/applications/new)
2. Set **Application name**: `Sepulki Forge (Development)`
3. Set **Homepage URL**: `http://localhost:3000`
4. Set **Authorization callback URL**: `http://localhost:3000/api/auth/callback/github`

### 2. Environment Configuration
Create `.env.local` in the `apps/forge-ui` directory:

```bash
# GitHub OAuth (Optional for Development)
GITHUB_CLIENT_ID=your_github_client_id
GITHUB_CLIENT_SECRET=your_github_client_secret

# NextAuth Configuration
NEXTAUTH_SECRET=your_random_secret_string
NEXTAUTH_URL=http://localhost:3000

# OpenAI (for AI analysis features)
OPENAI_API_KEY=your_openai_api_key
```

### 3. Generate NEXTAUTH_SECRET
```bash
openssl rand -base64 32
```

## 🛡️ Security Features

### 1. **Middleware Protection**
Protected routes automatically redirect unauthenticated users:
```typescript
const protectedRoutes = [
  '/configure',
  '/review', 
  '/quote',
  '/dashboard',
  '/api/analyze',
]
```

### 2. **Session Management**
- JWT-based sessions (24-hour expiry)
- Automatic token refresh
- Secure sign-out with callback

### 3. **Development Safety**
- Mock authentication only enabled in development
- Production always requires real OAuth
- Clear environment detection

## 🎨 UI Components

### SmithProfile Component
Displays authenticated user with:
- **User avatar** (GitHub or initials)
- **Role emoji** (🔨/⚒️/👑)
- **Dropdown menu** with sign-out option

### Authentication Pages
- **Sign In**: `/auth/signin` - Branded sign-in page
- **Error**: `/auth/error` - Helpful error messages
- **Callbacks**: Handled automatically by NextAuth

## 🔄 Integration with GraphQL

The authentication system is designed to integrate seamlessly with the Hammer Orchestrator GraphQL API:

```typescript
// Future: Connect to Sepulki backend
const session = await auth()
const smithId = session?.user?.id

// Query Smith permissions from database
const permissions = await getSmithPermissions(smithId)
```

## 📱 Developer Experience

### Immediate Benefits
✅ **Zero Configuration**: Works out of the box  
✅ **Realistic Testing**: Full auth flow without setup  
✅ **Role-Based UI**: Test permission-based features  
✅ **Production Ready**: Easy transition to real OAuth  

### Development Commands
```bash
# Start with mock auth (default)
npm run dev

# Start with GitHub OAuth (if configured)
GITHUB_CLIENT_ID=xxx GITHUB_CLIENT_SECRET=xxx npm run dev
```

## 🔮 Future Enhancements

1. **Database Integration**: Connect to Smith profiles in PostgreSQL
2. **Organization Permissions**: GitHub org-based role assignment  
3. **SSO Support**: Add additional providers (Google, Microsoft)
4. **API Key Authentication**: For programmatic access
5. **Session Analytics**: Track smith activity and usage

This authentication system provides the perfect balance of developer convenience and production readiness for the Sepulki platform! 🔥
