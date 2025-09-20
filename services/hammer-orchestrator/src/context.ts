import { Pool } from 'pg';
import Redis from 'ioredis';
import { jwtVerify } from 'jose';
import type { Smith, AuthSession } from '@sepulki/shared-types';
import { Permission, SmithRole } from '@sepulki/shared-types';
import { setupDataLoaders } from './dataloaders';

export interface Context {
  db: Pool;
  redis: Redis;
  smith?: Smith;
  session?: AuthSession;
  dataloaders: ReturnType<typeof setupDataLoaders>;
}

// Database connection
const db = new Pool({
  connectionString: process.env.DATABASE_URL || 'postgresql://smith:forge_dev@localhost:5432/sepulki',
  max: 20,
  idleTimeoutMillis: 30000,
  connectionTimeoutMillis: 5000,
});

// Redis connection
const redis = new Redis(process.env.REDIS_URL || 'redis://localhost:6379');

// JWT secret
const JWT_SECRET = new TextEncoder().encode(
  process.env.JWT_SECRET || 'your-super-secret-jwt-key-change-in-production'
);

export async function createContext({ token }: { token?: string }): Promise<Context> {
  const dataloaders = setupDataLoaders(db);

  if (!token) {
    return { db, redis, dataloaders };
  }

  console.log('🔧 NODE_ENV:', process.env.NODE_ENV);
  console.log('🔧 Token ends with mock signature:', token.endsWith('.mock-signature-for-development'));

  try {
    let payload: any;
    let smithId: string;
    let sessionId: string;

    // Check if this is a mock development token (only in local development)
    const isLocalDev = (process.env.NODE_ENV === 'development' || process.env.NODE_ENV === 'dev') && 
                      (process.env.DATABASE_URL?.includes('localhost') || process.env.DATABASE_URL?.includes('127.0.0.1'));
    
    if (isLocalDev && token.endsWith('.mock-signature-for-development')) {
      // Parse mock JWT token (development only)
      const parts = token.split('.');
      if (parts.length === 3) {
        try {
          payload = JSON.parse(Buffer.from(parts[1], 'base64').toString('utf-8'));
          console.log('🔧 Using mock JWT token for development:', payload.email);
        } catch (e) {
          throw new Error('Invalid mock token format');
        }
      } else {
        throw new Error('Invalid token format');
      }
    } else {
      // Use real JWT verification
      const result = await jwtVerify(token, JWT_SECRET);
      payload = result.payload;
    }
    
    // Fetch smith and session from token claims
    smithId = payload.sub as string;
    sessionId = payload.sessionId as string;

    // For development mock tokens, use fallback mock smith data
    if (isLocalDev && sessionId === 'mock-session-001') {
      // Create mock smith and session for development
      const mockSmith: Smith = {
        id: smithId,
        name: payload.name || 'Development Smith',
        email: payload.email || 'dev@sepulki.com',
        role: payload.role || SmithRole.OVER_SMITH,
        permissions: [Permission.VIEW_CATALOG, Permission.FORGE_SEPULKA, Permission.CAST_INGOT, Permission.TEMPER_INGOT, Permission.QUENCH_TO_FLEET],
        isActive: true,
        createdAt: new Date(),
        updatedAt: new Date(),
        preferences: {
          theme: 'dark' as const,
          language: 'en',
          timezone: 'UTC',
          notifications: {
            email: true,
            push: true
          },
          dashboard: {
            defaultView: 'overview',
            widgets: []
          }
        }
      };

      const mockSession: AuthSession = {
        smithId: smithId,
        token: 'mock-token-for-development',
        refreshToken: 'mock-refresh-token',
        expiresAt: new Date(payload.exp * 1000),
        permissions: mockSmith.permissions,
        role: payload.role || SmithRole.OVER_SMITH
      };

      console.log('✅ Mock authentication successful for development:', mockSmith.email);

      return {
        db,
        redis,
        smith: mockSmith,
        session: mockSession,
        dataloaders
      };
    }

    // Production path: Load smith from database
    const smithQuery = await db.query(
      'SELECT * FROM smiths WHERE id = $1 AND is_active = true',
      [smithId]
    );

    if (smithQuery.rows.length === 0) {
      throw new Error('Smith not found or inactive');
    }

    const smith: Smith = smithQuery.rows[0];

    // Load session from Redis
    const sessionKey = `session:${sessionId}`;
    const sessionData = await redis.get(sessionKey);
    
    if (!sessionData) {
      throw new Error('Session not found or expired');
    }

    const session: AuthSession = JSON.parse(sessionData);

    // Verify session hasn't expired
    if (new Date() > session.expiresAt) {
      await redis.del(sessionKey);
      throw new Error('Session expired');
    }

    return {
      db,
      redis,
      smith,
      session,
      dataloaders
    };
  } catch (error) {
    console.error('Auth context error:', error);
    throw new Error('Authentication failed');
  }
}

export async function requireAuth(context: Context): Promise<{ smith: Smith; session: AuthSession }> {
  if (!context.smith || !context.session) {
    throw new Error('Authentication required');
  }
  return { smith: context.smith, session: context.session };
}

export async function requirePermission(
  context: Context, 
  permission: Permission
): Promise<{ smith: Smith; session: AuthSession }> {
  const { smith, session } = await requireAuth(context);
  
  if (!session.permissions.includes(permission)) {
    throw new Error(`Permission denied: ${permission} required`);
  }

  return { smith, session };
}
