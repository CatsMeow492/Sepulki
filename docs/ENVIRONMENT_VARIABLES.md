# 🔧 Environment Variables Reference

Complete reference for all environment variables used in Sepulki.

## 📋 **Required Variables by Environment**

### **Local Development (Automatic)**
```bash
# These are set automatically - no action needed
NODE_ENV=development
DATABASE_URL=postgresql://smith:forge_dev@localhost:5432/sepulki
REDIS_URL=redis://localhost:6379
NEXT_PUBLIC_GRAPHQL_ENDPOINT=http://localhost:4000/graphql
```

### **Production (Must Configure)**
```bash
# Essential production variables
NODE_ENV=production                    # Set by deployment platform
GITHUB_CLIENT_ID=your_github_client_id # GitHub OAuth
GITHUB_CLIENT_SECRET=your_secret       # GitHub OAuth
NEXTAUTH_SECRET=random_secure_string   # Session signing
DATABASE_URL=postgresql://...          # Production database
```

## 🔐 **Authentication Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `GITHUB_CLIENT_ID` | Production | Yes | GitHub OAuth app client ID |
| `GITHUB_CLIENT_SECRET` | Production | Yes | GitHub OAuth app secret |
| `GOOGLE_CLIENT_ID` | Production | Optional | Google OAuth client ID |
| `GOOGLE_CLIENT_SECRET` | Production | Optional | Google OAuth secret |
| `LOCAL_OAUTH_CLIENT_ID` | Development | Optional | Local OAuth testing |
| `LOCAL_OAUTH_CLIENT_SECRET` | Development | Optional | Local OAuth testing |
| `NEXTAUTH_SECRET` | Production | Yes | JWT signing secret |
| `NEXTAUTH_URL` | Production | Yes | App's public URL |

## 💾 **Database Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `DATABASE_URL` | Production | Yes | PostgreSQL connection string |
| `REDIS_URL` | Production | Optional | Redis connection (sessions/cache) |

### **Database URL Examples**
```bash
# PostgreSQL (Supabase)
DATABASE_URL=postgresql://postgres:password@db.xxx.supabase.co:5432/postgres

# PostgreSQL (Railway)  
DATABASE_URL=postgresql://postgres:password@containers-us-west-xx.railway.app:5432/railway

# PostgreSQL (PlanetScale - MySQL compatible)
DATABASE_URL=mysql://username:password@aws.connect.psdb.cloud/database-name?sslaccept=strict

# Redis (Upstash)
REDIS_URL=rediss://default:password@us1-xxx.upstash.io:6379

# Redis (Railway)
REDIS_URL=redis://default:password@containers-us-west-xx.railway.app:6379
```

## 📧 **Email Service Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `EMAIL_SERVER` | Production | Optional | SMTP server connection |
| `EMAIL_FROM` | Production | Optional | From address for emails |

### **Email Provider Examples**
```bash
# SendGrid
EMAIL_SERVER=smtps://apikey:SG.xxx@smtp.sendgrid.net:465
EMAIL_FROM=noreply@yourdomain.com

# Resend
EMAIL_SERVER=smtps://resend:re_xxx@smtp.resend.com:465
EMAIL_FROM=noreply@yourdomain.com

# AWS SES
EMAIL_SERVER=smtps://username:password@email-smtp.us-east-1.amazonaws.com:465
EMAIL_FROM=noreply@yourdomain.com

# Local Development (MailHog)
EMAIL_SERVER=smtp://localhost:1025
EMAIL_FROM=noreply@sepulki.local
```

## 🌐 **API & Frontend Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `NEXT_PUBLIC_GRAPHQL_ENDPOINT` | All | Auto-detected | GraphQL API URL |
| `CORS_ORIGIN` | Production | Optional | CORS allowed origins |
| `PORT` | Production | Optional | Server port (default: 3000) |

### **API Endpoint Examples**
```bash
# Local Development
NEXT_PUBLIC_GRAPHQL_ENDPOINT=http://localhost:4000/graphql

# Vercel Production
NEXT_PUBLIC_GRAPHQL_ENDPOINT=https://your-app.vercel.app/api/graphql

# Custom Domain
NEXT_PUBLIC_GRAPHQL_ENDPOINT=https://api.yourdomain.com/graphql
```

## 📦 **File Storage Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `AWS_ACCESS_KEY_ID` | Production | Optional | AWS S3 access key |
| `AWS_SECRET_ACCESS_KEY` | Production | Optional | AWS S3 secret key |
| `AWS_REGION` | Production | Optional | AWS region |
| `AWS_S3_BUCKET` | Production | Optional | S3 bucket name |
| `MINIO_ENDPOINT` | Development | Auto-set | MinIO local storage |

### **Storage Examples**
```bash
# AWS S3 (Production)
AWS_ACCESS_KEY_ID=AKIAIOSFODNN7EXAMPLE
AWS_SECRET_ACCESS_KEY=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
AWS_REGION=us-east-1
AWS_S3_BUCKET=sepulki-production-assets

# MinIO (Local Development - Auto-configured)
MINIO_ENDPOINT=http://localhost:9000
MINIO_ACCESS_KEY=sepulki
MINIO_SECRET_KEY=vault_dev_key
```

## 🤖 **External Service Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `OPENAI_API_KEY` | All | Optional | OpenAI API for AI features |
| `TWILIO_ACCOUNT_SID` | Production | Optional | Twilio SMS service |
| `TWILIO_AUTH_TOKEN` | Production | Optional | Twilio authentication |
| `SENTRY_DSN` | Production | Optional | Error monitoring |

## 🏭 **Platform-Specific Variables**

### **Vercel**
```bash
# Automatically set by Vercel
VERCEL=1
VERCEL_URL=your-app.vercel.app
VERCEL_ENV=production
```

### **Railway**
```bash
# Automatically set by Railway  
RAILWAY_STATIC_URL=your-app.railway.app
RAILWAY_GIT_COMMIT_SHA=abc123
DATABASE_URL=postgresql://...  # Auto-generated
REDIS_URL=redis://...          # Auto-generated
```

### **Netlify**
```bash
# Automatically set by Netlify
NETLIFY=true
URL=https://your-app.netlify.app
CONTEXT=production
```

## 🔒 **Security Variables**

| Variable | Environment | Required | Description |
|----------|-------------|----------|-------------|
| `NEXTAUTH_SECRET` | Production | Yes | Must be 32+ random characters |
| `JWT_SECRET` | Production | Optional | Backend JWT signing (defaults to NEXTAUTH_SECRET) |
| `SESSION_MAX_AGE` | All | Optional | Session duration in seconds |

### **Generating Secure Secrets**
```bash
# Generate NEXTAUTH_SECRET
openssl rand -base64 32

# Or use online generator
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"
```

## 📱 **Development vs Production Detection**

The app automatically detects environment based on:

### **Development Indicators**
```bash
NODE_ENV=development
# OR missing GITHUB_CLIENT_ID
# OR localhost in URLs
```

### **Production Indicators**  
```bash
NODE_ENV=production
# AND GITHUB_CLIENT_ID is set
# AND production URLs detected
```

## ⚡ **Quick Setup Commands**

### **Local Development**
```bash
# No setup needed - runs automatically with mock auth
npm run dev
```

### **Production (Vercel)**
```bash
# Set required variables in Vercel dashboard
vercel env add GITHUB_CLIENT_ID
vercel env add GITHUB_CLIENT_SECRET  
vercel env add NEXTAUTH_SECRET
vercel env add DATABASE_URL

# Deploy
vercel --prod
```

### **Production (Railway)**
```bash
# Set variables via CLI
railway variables:set GITHUB_CLIENT_ID=xxx
railway variables:set GITHUB_CLIENT_SECRET=xxx
railway variables:set NEXTAUTH_SECRET=xxx

# Deploy
railway up
```

## 🏥 **Environment Health Check**

Your app logs will show environment detection:

### **Development**
```
🧪 Sepulki Environment Configuration: {
  platform: 'local',
  auth: ['mock'],
  graphql: 'http://localhost:4000/graphql',
  mockAuth: true
}
```

### **Production**
```
🔐 Sepulki Environment Configuration: {
  platform: 'vercel',
  auth: ['github'],
  graphql: 'https://your-app.vercel.app/api/graphql',
  mockAuth: false
}
```

## 🚨 **Common Environment Issues**

### **"No auth providers configured"**
```bash
# Missing: GITHUB_CLIENT_ID in production
# Fix: Set OAuth credentials
```

### **"Database connection failed"**
```bash
# Issue: Wrong DATABASE_URL format
# Fix: Check connection string format
```

### **"NEXTAUTH_URL mismatch"**  
```bash
# Issue: NEXTAUTH_URL doesn't match deployment domain
# Fix: Update to match production URL
```

This environment system ensures your app **automatically works locally** with mock auth and **seamlessly transitions to production** with real OAuth providers! 🚀
