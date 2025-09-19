import type { 
  Smith, 
  AuthSession, 
  LoginCredentials, 
  LoginResponse, 
  RefreshTokenRequest 
} from '@sepulki/shared-types';

export interface AuthStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

export interface AuthManagerConfig {
  apiUrl: string;
  storage?: AuthStorage;
}

const STORAGE_KEYS = {
  TOKEN: 'sepulki_token',
  REFRESH_TOKEN: 'sepulki_refresh_token',
  SMITH: 'sepulki_smith',
  SESSION: 'sepulki_session'
} as const;

export class AuthManager {
  private config: AuthManagerConfig;
  private storage: AuthStorage;

  constructor(config: AuthManagerConfig) {
    this.config = config;
    this.storage = config.storage || new MemoryStorage();
  }

  async login(email: string, password: string): Promise<LoginResponse> {
    const response = await fetch(`${this.config.apiUrl}/graphql`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        query: `
          mutation Login($credentials: LoginCredentials!) {
            login(credentials: $credentials) {
              smith {
                id
                email
                name
                role
                permissions
                preferences
              }
              session {
                token
                refreshToken
                expiresAt
                permissions
                role
              }
            }
          }
        `,
        variables: {
          credentials: { email, password }
        }
      })
    });

    const { data, errors } = await response.json();

    if (errors) {
      throw new Error(errors[0].message);
    }

    const loginResponse = data.login;

    // Store authentication data
    this.setToken(loginResponse.session.token);
    this.setRefreshToken(loginResponse.session.refreshToken);
    this.setSmith(loginResponse.smith);
    this.setSession(loginResponse.session);

    return loginResponse;
  }

  async logout(): Promise<void> {
    // Call logout mutation if we have a token
    const token = this.getToken();
    
    if (token) {
      try {
        await fetch(`${this.config.apiUrl}/graphql`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${token}`
          },
          body: JSON.stringify({
            query: `
              mutation Logout {
                logout
              }
            `
          })
        });
      } catch (error) {
        console.error('Error during logout:', error);
      }
    }

    // Clear stored data
    this.clearStoredAuth();
  }

  async refreshToken(): Promise<AuthSession | null> {
    const refreshToken = this.getRefreshToken();
    
    if (!refreshToken) {
      return null;
    }

    try {
      const response = await fetch(`${this.config.apiUrl}/graphql`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          query: `
            mutation RefreshToken($refreshToken: String!) {
              refreshToken(refreshToken: $refreshToken) {
                smith {
                  id
                  email
                  name
                  role
                  permissions
                  preferences
                }
                session {
                  token
                  refreshToken
                  expiresAt
                  permissions
                  role
                }
              }
            }
          `,
          variables: { refreshToken }
        })
      });

      const { data, errors } = await response.json();

      if (errors) {
        this.clearStoredAuth();
        return null;
      }

      const refreshResponse = data.refreshToken;

      // Update stored authentication data
      this.setToken(refreshResponse.session.token);
      this.setRefreshToken(refreshResponse.session.refreshToken);
      this.setSmith(refreshResponse.smith);
      this.setSession(refreshResponse.session);

      return refreshResponse.session;
    } catch (error) {
      console.error('Error refreshing token:', error);
      this.clearStoredAuth();
      return null;
    }
  }

  async getToken(): Promise<string | null> {
    const token = this.storage.getItem(STORAGE_KEYS.TOKEN);
    const session = this.getSession();

    if (!token || !session) {
      return null;
    }

    // Check if token is expired
    const expiresAt = new Date(session.expiresAt);
    const now = new Date();

    if (now >= expiresAt) {
      // Try to refresh the token
      const newSession = await this.refreshToken();
      return newSession?.token || null;
    }

    return token;
  }

  getRefreshToken(): string | null {
    return this.storage.getItem(STORAGE_KEYS.REFRESH_TOKEN);
  }

  getCurrentSmith(): Smith | null {
    const smithData = this.storage.getItem(STORAGE_KEYS.SMITH);
    return smithData ? JSON.parse(smithData) : null;
  }

  getSession(): AuthSession | null {
    const sessionData = this.storage.getItem(STORAGE_KEYS.SESSION);
    return sessionData ? JSON.parse(sessionData) : null;
  }

  isAuthenticated(): boolean {
    const token = this.storage.getItem(STORAGE_KEYS.TOKEN);
    const session = this.getSession();

    if (!token || !session) {
      return false;
    }

    // Check if session is expired
    const expiresAt = new Date(session.expiresAt);
    const now = new Date();

    return now < expiresAt;
  }

  hasPermission(permission: string): boolean {
    const smith = this.getCurrentSmith();
    const session = this.getSession();

    if (!smith || !session) {
      return false;
    }

    return session.permissions.includes(permission);
  }

  private setToken(token: string): void {
    this.storage.setItem(STORAGE_KEYS.TOKEN, token);
  }

  private setRefreshToken(refreshToken: string): void {
    this.storage.setItem(STORAGE_KEYS.REFRESH_TOKEN, refreshToken);
  }

  private setSmith(smith: Smith): void {
    this.storage.setItem(STORAGE_KEYS.SMITH, JSON.stringify(smith));
  }

  private setSession(session: AuthSession): void {
    this.storage.setItem(STORAGE_KEYS.SESSION, JSON.stringify(session));
  }

  private clearStoredAuth(): void {
    Object.values(STORAGE_KEYS).forEach(key => {
      this.storage.removeItem(key);
    });
  }
}

// Fallback storage for environments without localStorage
class MemoryStorage implements AuthStorage {
  private store: Record<string, string> = {};

  getItem(key: string): string | null {
    return this.store[key] || null;
  }

  setItem(key: string, value: string): void {
    this.store[key] = value;
  }

  removeItem(key: string): void {
    delete this.store[key];
  }
}
