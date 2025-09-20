# 🔐 Local Auth Service

**LocalStack equivalent for Auth.js** - provides the complete production authentication experience, but running entirely locally.

## 🎯 **Purpose**

Instead of "instant login" that bypasses the real authentication flow, this service provides:

✅ **Real login forms** (just like production)  
✅ **Database authentication** (using your test users)  
✅ **JWT sessions** (same format as production)  
✅ **Session management** (sign out, expiry, refresh)  
✅ **Auth.js API compatibility** (same endpoints)

## 🚀 **How It Works**

### **Development Experience**
```
User visits app → Redirected to login form → Enter credentials → JWT session created → Access granted
```

**Just like production Auth.js**, but using local services!

### **vs. Current "Instant Login"**
```
❌ Current: Auto-login bypass (unrealistic)
User visits app → Automatically logged in → No real auth experience

✅ New: Realistic local auth (production-like)  
User visits app → Real login form → Database verification → Session creation
```

## 📊 **Endpoints (Auth.js Compatible)**

| Endpoint | Purpose | Description |
|----------|---------|-------------|
| `GET /auth/signin` | Login page | Beautiful sign-in form with test users |
| `POST /auth/signin` | Login handler | Validates credentials, creates session |
| `GET /auth/session` | Session check | Returns current user or null |
| `POST /auth/signout` | Logout | Clears session, redirects to home |
| `GET /auth/csrf` | CSRF token | Security token for forms |
| `GET /auth/providers` | Available providers | Lists auth methods |

## 🧪 **Test Users (From Database)**

| Email | Password | Role | Permissions |
|-------|----------|------|-------------|
| **dev@sepulki.com** | `dev123` | Over-Smith | Full development access |
| **demo@sepulki.com** | `demo123` | Smith | Basic permissions |
| **test@sepulki.com** | `test123` | Over-Smith | Advanced testing |
| **admin@sepulki.com** | `admin123` | Admin | Complete system access |

## 🔧 **Integration**

### **Frontend (Updated AuthProvider)**
```typescript
// Check session on app start
const checkSession = async () => {
  const response = await fetch('http://localhost:4446/auth/session', {
    credentials: 'include'
  });
  const session = await response.json();
  
  if (session.user) {
    setSmith(session.user);
  }
};
```

### **Protected Routes**
```typescript
// Redirect to login if not authenticated
if (!smith) {
  window.location.href = 'http://localhost:4446/auth/signin?callbackUrl=' + 
                        encodeURIComponent(window.location.href);
}
```

## 🎉 **Benefits**

### **🎯 Realistic Testing**
- Test actual login flows
- Verify session management
- Test sign-out behavior
- Debug authentication issues

### **⚡ Development Speed**
- No external OAuth setup needed
- Instant user switching (click test users)
- Local database integration
- Hot reload friendly

### **🔒 Production Parity**
- Same JWT format as production
- Same session management
- Same API endpoints
- Same security model

## 🚀 **Usage**

### **Start Service**
```bash
cd services/local-auth
npm install
npm run dev
```

### **Update Frontend**
Update AuthProvider to use real session checks instead of instant login.

### **Test Authentication**
1. Visit http://localhost:3000
2. Get redirected to http://localhost:4446/auth/signin
3. Click "Dev Smith" test user or enter credentials
4. Get redirected back with real session
5. Access protected data with proper authentication

## 🌐 **Production Transition**

When deploying to production:

### **Development**
```javascript
Auth service: http://localhost:4446
Users: Database test users
Flow: Local login form → Local session
```

### **Production**
```javascript
Auth service: NextAuth.js with GitHub/Google
Users: Real OAuth providers
Flow: OAuth provider → Real session
```

**Same API, same experience, different backing services!**

This gives you the **LocalStack experience for authentication** - complete production parity with local control! 🔥
