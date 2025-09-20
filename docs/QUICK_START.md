# 🚀 Sepulki Quick Start Guide

Get up and running with the complete Sepulki development environment in under 2 minutes.

## ⚡ **30-Second Setup**

```bash
git clone https://github.com/CatsMeow492/Sepulki.git
cd Sepulki
npm install
npm run dev
```

**That's it!** 🎉

- ✅ Frontend: http://localhost:3000
- ✅ GraphQL API: http://localhost:4000/graphql  
- ✅ Auto-login as "Development Smith"
- ✅ Complete test data ready

## 🎯 **What You Get Instantly**

### **🧪 Mock Authentication**
- **Auto-login** as "Development Smith" (Over-Smith role)
- **No OAuth setup** required
- **Full permissions** to test all features
- **Instant development** - no login friction

### **📊 Rich Test Data**
- **4 Test Users** with different roles
- **3 Robot Designs** (DevBot, WarehouseWorker, AssemblyBot)
- **4 Active Fleets** with working robots
- **4 Live Tasks** (Pick & Place, Assembly, Inspection, Patrol)
- **8 Components** in the Alloy library

### **🛠️ Complete Infrastructure**
- **PostgreSQL**: Full schema with indexes
- **Redis**: Session storage and caching
- **MinIO**: File storage with web console
- **MailHog**: Email testing (http://localhost:8025)

## 🔧 **Environment Management**

### **Check Current Environment**
```bash
npm run env:status
```
Output:
```
🏷️  Environment: DEVELOPMENT
🧪 Authentication: MOCK (Development Smith)
🗄️  Database: localhost:5432/sepulki
🔨 GraphQL: http://localhost:4000/graphql
```

### **Switch Authentication Modes**
```bash
# Use mock auth (default - instant development)
npm run env:mock

# Test real GitHub OAuth locally
npm run env:github

# See production deployment instructions
npm run env:production
```

## 🌐 **Access Points**

| Service | URL | Purpose |
|---------|-----|---------|
| **🔥 Forge UI** | http://localhost:3000 | Main application |
| **🔨 GraphQL API** | http://localhost:4000/graphql | API playground |
| **📦 MinIO Console** | http://localhost:9001 | File storage (sepulki/vault_dev_key) |
| **📧 Email Testing** | http://localhost:8025 | MailHog email capture |
| **🗄️ Database** | localhost:5432 | PostgreSQL (smith/forge_dev) |

## 👥 **Test Users**

| Email | Password | Role | Permissions |
|-------|----------|------|-------------|
| **dev@sepulki.com** | `dev123` | Over-Smith | Full development access |
| **demo@sepulki.com** | `demo123` | Smith | Basic user permissions |
| **test@sepulki.com** | `test123` | Over-Smith | Advanced testing |
| **admin@sepulki.com** | `admin123` | Admin | Complete system access |

## 🧪 **Development Workflow**

### **1. Start Environment**
```bash
npm run dev
```

### **2. Open Application**
```bash
open http://localhost:3000
```

### **3. Explore Features**
- **Dashboard**: View fleets, robots, and tasks
- **Configure**: Design robots with 3D viewer
- **Review**: Check robot specifications
- **Quote**: Get deployment estimates

### **4. Check Data**
```bash
npm run db:status
```

### **5. Reset When Needed**
```bash
npm run db:reset     # Reset database with fresh data
npm run restart      # Restart all services
```

## 🚀 **Production Deployment**

When ready to deploy, the app **automatically switches to production mode**:

### **Set Environment Variables**
```bash
# Required for production
NODE_ENV=production                    # Set by platform
GITHUB_CLIENT_ID=your_github_client_id # GitHub OAuth
GITHUB_CLIENT_SECRET=your_github_secret
NEXTAUTH_SECRET=random_secure_string   # Generate with: openssl rand -base64 32
DATABASE_URL=postgresql://...          # Production database
```

### **Deploy Commands**
```bash
# Vercel
vercel --prod

# Railway  
railway up

# Docker
docker build -t sepulki . && docker run -p 3000:3000 sepulki
```

### **Result: Automatic Environment Switching! ✨**
- **Development**: Mock auth, local services, instant startup
- **Production**: Real OAuth, cloud database, secure sessions

## 🛠️ **Available Commands**

### **Core Development**
```bash
npm run dev          # Start everything
npm run stop         # Stop all services
npm run restart      # Restart everything
npm run status       # Check service health
```

### **Database Management**
```bash
npm run db:status    # View database contents
npm run db:reset     # Reset with fresh data
npm run db:seed      # Add more test data
```

### **Environment Control**
```bash
npm run env:status     # Check current mode
npm run env:mock       # Switch to mock auth
npm run env:github     # Switch to GitHub OAuth
npm run env:production # Show deployment guide
```

### **Service Utilities**
```bash
npm run docker:logs    # View container logs
npm run email:open     # Open MailHog email UI
npm run build          # Build for production
```

## 🏥 **Troubleshooting**

### **"Services not starting"**
```bash
npm run docker:up     # Start infrastructure
npm run db:status     # Check database
```

### **"GraphQL not responding"**
```bash
cd services/hammer-orchestrator
npm run dev          # Start backend manually
```

### **"Frontend compilation errors"**
```bash
rm -rf apps/forge-ui/.next
npm run dev:forge    # Restart frontend
```

## 🎉 **Success Indicators**

You'll know everything is working when:

✅ **Frontend loads** at http://localhost:3000  
✅ **Auto-login works** - see "Development Smith" in top-right  
✅ **Dashboard shows data** - fleets, robots, tasks displayed  
✅ **GraphQL responds** - API playground accessible  
✅ **Database has data** - `npm run db:status` shows test data  

## 🔮 **Next Steps**

Once you're running locally:

1. **🎨 UI Development** - Customize the Forge interface
2. **📊 Data Integration** - Connect real robot telemetry
3. **🤖 Add Features** - Build new robot capabilities
4. **🚀 Deploy** - Push to production with automatic OAuth

Your Sepulki development environment is **production-ready** and **developer-friendly**! 🔥
