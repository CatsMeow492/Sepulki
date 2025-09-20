'use client'

import { ReactNode, createContext, useContext, useEffect, useState } from "react"
import { env, shouldUseMockAuth, shouldUseRealAuth } from '../lib/env'

interface Smith {
  id: string
  name?: string
  email?: string
  image?: string
  role: 'SMITH' | 'OVER_SMITH' | 'ADMIN'
}

interface AuthContextType {
  smith: Smith | null
  loading: boolean
  signOut: () => void
  authMode: 'mock' | 'real' | 'none'
}

const AuthContext = createContext<AuthContextType | undefined>(undefined)

interface AuthProviderProps {
  children: ReactNode
}

export function AuthProvider({ children }: AuthProviderProps) {
  const [smith, setSmith] = useState<Smith | null>(null)
  const [loading, setLoading] = useState(true)
  const [authMode, setAuthMode] = useState<'mock' | 'real' | 'none'>('none')

  // Update global auth state for GraphQL client
  useEffect(() => {
    if (typeof window !== 'undefined') {
      (window as any).__SEPULKI_AUTH__ = { smith, authMode };
    }
  }, [smith, authMode]);

  useEffect(() => {
    // Environment-aware authentication setup
    if (shouldUseMockAuth()) {
      // Development mode with LOCAL AUTH SERVICE (like LocalStack)
      console.log('🔐 Using local Auth.js service (LocalStack equivalent)')
      setAuthMode('mock')
      checkLocalSession()
    } else if (shouldUseRealAuth()) {
      // Production mode - NextAuth.js will handle authentication
      setAuthMode('real')
      console.log('🔐 Using real authentication providers:', env.authProviders)
      // TODO: Initialize NextAuth.js session here
      setLoading(false)
    } else {
      // No authentication configured
      setAuthMode('none')
      console.warn('⚠️ No authentication providers configured')
      setLoading(false)
    }
  }, [])

  // Check session with local auth service
  const checkLocalSession = async () => {
    try {
      const response = await fetch('http://localhost:4446/auth/session', {
        credentials: 'include'
      })
      const session = await response.json()
      
      if (session.user) {
        setSmith({
          id: session.user.id,
          name: session.user.name,
          email: session.user.email,
          image: session.user.image || '',
          role: session.user.role
        })
        console.log('✅ Local auth session found:', session.user.email)
      } else {
        console.log('❌ No local auth session - redirecting to sign in')
        // Redirect to local auth sign-in page
        const callbackUrl = encodeURIComponent(window.location.href)
        window.location.href = `http://localhost:4446/auth/signin?callbackUrl=${callbackUrl}`
      }
    } catch (error) {
      console.error('❌ Local auth service not available:', error)
      console.log('🔄 Falling back to instant mock auth')
      // Fallback to instant login if local auth service is down
      setSmith({
        id: 'c1f8431f-e264-4701-9ce6-2fcc3362c649',
        name: 'Development Smith',
        email: 'dev@sepulki.com',
        image: '',
        role: 'OVER_SMITH'
      })
    }
    setLoading(false)
  }

  const signOut = async () => {
    setSmith(null)
    if (authMode === 'real') {
      // Production: NextAuth.js signOut
      // signOut() from next-auth/react
    } else if (authMode === 'mock') {
      // Development: Local auth service signOut
      try {
        await fetch('http://localhost:4446/auth/signout', {
          method: 'POST',
          credentials: 'include'
        })
        console.log('✅ Signed out via local auth service')
        window.location.href = '/'
      } catch (error) {
        console.error('❌ Local auth signout failed:', error)
        // Fallback: just redirect
        window.location.href = '/'
      }
    }
  }

  return (
    <AuthContext.Provider value={{ smith, loading, signOut, authMode }}>
      {children}
    </AuthContext.Provider>
  )
}

export function useAuth() {
  const context = useContext(AuthContext)
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider')
  }
  return context
}
