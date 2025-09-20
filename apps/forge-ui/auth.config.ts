import type { NextAuthConfig } from "next-auth"
import type { Provider } from "next-auth/providers"

// Local development OAuth provider
const localOAuthProvider: Provider = {
  id: "local-oauth",
  name: "Local OAuth",
  type: "oidc",
  issuer: process.env.LOCAL_OAUTH_ISSUER || "http://localhost:4444",
  clientId: process.env.LOCAL_OAUTH_CLIENT_ID || "sepulki-forge",
  clientSecret: process.env.LOCAL_OAUTH_CLIENT_SECRET || "sepulki-forge-secret",
  authorization: {
    params: {
      scope: "openid email profile",
    },
  },
  profile(profile) {
    return {
      id: profile.sub,
      name: profile.name || profile.email?.split('@')[0],
      email: profile.email,
      image: profile.picture,
      role: profile.role || 'SMITH', // Default role
    }
  },
}

// GitHub OAuth provider (for production)
const githubProvider: Provider = {
  id: "github", 
  name: "GitHub",
  type: "oauth",
  authorization: "https://github.com/login/oauth/authorize?scope=read:user+user:email",
  token: "https://github.com/login/oauth/access_token",
  userinfo: "https://api.github.com/user",
  clientId: process.env.GITHUB_CLIENT_ID,
  clientSecret: process.env.GITHUB_CLIENT_SECRET,
  profile(profile) {
    return {
      id: profile.id.toString(),
      name: profile.name || profile.login,
      email: profile.email,
      image: profile.avatar_url,
      role: determineUserRole(profile.email), // Custom role logic
    }
  },
}

// Mock provider for development (when no other auth is configured)
const mockProvider: Provider = {
  id: "mock",
  name: "Development Smith",
  type: "credentials",
  credentials: {
    email: { label: "Email", type: "email" },
    password: { label: "Password", type: "password" }
  },
  async authorize(credentials) {
    // In development, always return the dev user
    if (process.env.NODE_ENV === 'development') {
      return {
        id: 'dev-smith-001',
        name: 'Development Smith',
        email: credentials?.email || 'dev@sepulki.com',
        role: 'OVER_SMITH'
      }
    }
    return null
  },
}

// Determine user role based on email or other criteria
function determineUserRole(email: string | null): 'SMITH' | 'OVER_SMITH' | 'ADMIN' {
  if (!email) return 'SMITH'
  
  // Admin emails
  if (email.includes('admin@') || email.includes('@sepulki.com')) {
    return 'ADMIN'
  }
  
  // Over-smith patterns
  if (email.includes('lead@') || email.includes('senior@')) {
    return 'OVER_SMITH'
  }
  
  return 'SMITH'
}

// Google OAuth provider (production alternative)
const googleProvider: Provider = {
  id: "google",
  name: "Google",
  type: "oauth",
  authorization: "https://accounts.google.com/oauth/authorize?scope=openid email profile",
  token: "https://oauth2.googleapis.com/token",
  userinfo: "https://openidconnect.googleapis.com/v1/userinfo",
  clientId: process.env.GOOGLE_CLIENT_ID,
  clientSecret: process.env.GOOGLE_CLIENT_SECRET,
  profile(profile) {
    return {
      id: profile.sub,
      name: profile.name,
      email: profile.email,
      image: profile.picture,
      role: determineUserRole(profile.email),
    }
  },
}

// Environment-based provider selection
function getProviders(): Provider[] {
  const isDevelopment = process.env.NODE_ENV === 'development'
  const isProduction = process.env.NODE_ENV === 'production'
  const hasLocalOAuth = process.env.LOCAL_OAUTH_CLIENT_ID
  const hasGitHubOAuth = process.env.GITHUB_CLIENT_ID
  const hasGoogleOAuth = process.env.GOOGLE_CLIENT_ID
  
  const providers: Provider[] = []
  
  // Production: Always include configured OAuth providers
  if (hasGitHubOAuth) {
    providers.push(githubProvider)
  }
  
  if (hasGoogleOAuth) {
    providers.push(googleProvider)
  }
  
  // Development: Include local/mock auth if no production auth is configured
  if (isDevelopment) {
    if (hasLocalOAuth) {
      providers.push(localOAuthProvider)
    } else if (providers.length === 0) {
      // Only use mock auth if no real auth providers are configured
      providers.push(mockProvider)
    }
  }
  
  // Fallback warning for production without auth
  if (isProduction && providers.length === 0) {
    console.error('🚨 PRODUCTION ERROR: No authentication providers configured!')
  }
  
  console.log(`🔐 Auth providers configured: ${providers.map(p => p.name).join(', ')}`)
  
  return providers
}

export const authConfig: NextAuthConfig = {
  providers: getProviders(),
  pages: {
    signIn: '/auth/signin',
    error: '/auth/error',
  },
  callbacks: {
    authorized({ auth, request: { nextUrl } }) {
      const isLoggedIn = !!auth?.user
      const isOnDashboard = nextUrl.pathname.startsWith('/dashboard')
      const isOnConfigure = nextUrl.pathname.startsWith('/configure')
      const isOnReview = nextUrl.pathname.startsWith('/review')
      const isOnQuote = nextUrl.pathname.startsWith('/quote')
      
      if (isOnDashboard || isOnConfigure || isOnReview || isOnQuote) {
        if (isLoggedIn) return true
        return false // Redirect unauthenticated users to login page
      }
      
      return true
    },
    jwt({ token, user }) {
      if (user) {
        token.role = user.role
      }
      return token
    },
    session({ session, token }) {
      if (token?.role) {
        session.user.role = token.role as string
      }
      return session
    },
  },
  session: {
    strategy: "jwt",
    maxAge: 24 * 60 * 60, // 24 hours
  },
  // Use local services in development
  ...(process.env.NODE_ENV === 'development' && {
    // Local email service (MailHog)
    email: {
      server: process.env.EMAIL_SERVER || "smtp://localhost:1025",
      from: process.env.EMAIL_FROM || "noreply@sepulki.local",
    }
  })
}

export default authConfig
