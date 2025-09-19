import { useState, useEffect, useContext, createContext, ReactNode } from 'react';
import type { Smith, AuthSession } from '@sepulki/shared-types';
import { SepulkiClient } from '../client';

export interface AuthContextValue {
  smith: Smith | null;
  session: AuthSession | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  hasPermission: (permission: string) => boolean;
}

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
  client: SepulkiClient;
}

export function AuthProvider({ children, client }: AuthProviderProps) {
  const [smith, setSmith] = useState<Smith | null>(null);
  const [session, setSession] = useState<AuthSession | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Initialize auth state from storage
    const currentSmith = client.getCurrentSmith();
    const currentSession = client.auth.getSession();

    setSmith(currentSmith);
    setSession(currentSession);
    setIsLoading(false);
  }, [client]);

  const login = async (email: string, password: string): Promise<void> => {
    setIsLoading(true);
    try {
      const loginResponse = await client.login(email, password);
      setSmith(loginResponse.smith);
      setSession(loginResponse.session);
    } finally {
      setIsLoading(false);
    }
  };

  const logout = async (): Promise<void> => {
    setIsLoading(true);
    try {
      await client.logout();
      setSmith(null);
      setSession(null);
    } finally {
      setIsLoading(false);
    }
  };

  const hasPermission = (permission: string): boolean => {
    return client.auth.hasPermission(permission);
  };

  const value: AuthContextValue = {
    smith,
    session,
    isAuthenticated: smith !== null && session !== null,
    isLoading,
    login,
    logout,
    hasPermission
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);
  
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  
  return context;
}
