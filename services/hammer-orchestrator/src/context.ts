import { Pool } from 'pg';
import Redis from 'ioredis';
import { jwtVerify } from 'jose';
import type { Smith, AuthSession } from '@sepulki/shared-types';
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
const redis = new Redis(process.env.REDIS_URL || 'redis://localhost:6379', {
  retryDelayOnFailover: 100,
  maxRetriesPerRequest: 3,
});

// JWT secret
const JWT_SECRET = new TextEncoder().encode(
  process.env.JWT_SECRET || 'your-super-secret-jwt-key-change-in-production'
);

export async function createContext({ token }: { token?: string }): Promise<Context> {
  const dataloaders = setupDataLoaders(db);

  if (!token) {
    return { db, redis, dataloaders };
  }

  try {
    const { payload } = await jwtVerify(token, JWT_SECRET);
    
    // Fetch smith and session from token claims
    const smithId = payload.sub as string;
    const sessionId = payload.sessionId as string;

    // Load smith from database
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
  permission: string
): Promise<{ smith: Smith; session: AuthSession }> {
  const { smith, session } = await requireAuth(context);
  
  if (!session.permissions.includes(permission)) {
    throw new Error(`Permission denied: ${permission} required`);
  }

  return { smith, session };
}
