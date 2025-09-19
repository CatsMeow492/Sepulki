import type { Context } from '../context';
import { requirePermission, requireAuth } from '../context';
import { NotFoundError, ServiceError, ValidationError, AuthenticationError } from '../errors';
import { Permission } from '@sepulki/shared-types';
import { SignJWT } from 'jose';
import crypto from 'crypto';

const JWT_SECRET = new TextEncoder().encode(
  process.env.JWT_SECRET || 'your-super-secret-jwt-key-change-in-production'
);

export const authResolvers = {
  Mutation: {
    async login(parent: any, { credentials }: any, context: Context) {
      const { email, password } = credentials;

      // Validate input
      if (!email || !password) {
        throw new ValidationError('Email and password are required');
      }

      try {
        // Find smith by email
        const smithQuery = await context.db.query(
          'SELECT * FROM smiths WHERE email = $1 AND is_active = true',
          [email.toLowerCase()]
        );

        if (smithQuery.rows.length === 0) {
          throw new AuthenticationError('Invalid credentials');
        }

        const smith = smithQuery.rows[0];

        // TODO: Implement proper password hashing and verification
        // For now, this is a placeholder - in production, use bcrypt or similar
        const passwordHash = crypto.createHash('sha256').update(password).digest('hex');
        if (smith.password_hash !== passwordHash) {
          throw new AuthenticationError('Invalid credentials');
        }

        // Create session
        const sessionId = crypto.randomUUID();
        const expiresAt = new Date(Date.now() + 24 * 60 * 60 * 1000); // 24 hours

        const session = {
          smithId: smith.id,
          token: '', // Will be set below
          refreshToken: crypto.randomUUID(),
          expiresAt,
          permissions: smith.permissions || [],
          role: smith.role
        };

        // Create JWT token
        const token = await new SignJWT({
          sub: smith.id,
          email: smith.email,
          role: smith.role,
          sessionId
        })
          .setProtectedHeader({ alg: 'HS256' })
          .setExpirationTime(expiresAt)
          .setIssuedAt()
          .sign(JWT_SECRET);

        session.token = token;

        // Store session in Redis
        const sessionKey = `session:${sessionId}`;
        await context.redis.setex(sessionKey, 24 * 60 * 60, JSON.stringify(session));

        // Update last login
        await context.db.query(
          'UPDATE smiths SET last_login_at = NOW() WHERE id = $1',
          [smith.id]
        );

        return {
          smith,
          session
        };
      } catch (error) {
        if (error instanceof AuthenticationError) {
          throw error;
        }
        throw new ServiceError('auth', `Login failed: ${error}`);
      }
    },

    async refreshToken(parent: any, { refreshToken }: any, context: Context) {
      try {
        // Find session by refresh token
        const sessions = await context.redis.keys('session:*');
        let sessionData = null;
        let sessionKey = null;

        for (const key of sessions) {
          const data = await context.redis.get(key);
          if (data) {
            const parsed = JSON.parse(data);
            if (parsed.refreshToken === refreshToken) {
              sessionData = parsed;
              sessionKey = key;
              break;
            }
          }
        }

        if (!sessionData || !sessionKey) {
          throw new AuthenticationError('Invalid refresh token');
        }

        // Check if session is expired
        if (new Date() > new Date(sessionData.expiresAt)) {
          await context.redis.del(sessionKey);
          throw new AuthenticationError('Session expired');
        }

        // Get updated smith data
        const smithQuery = await context.db.query(
          'SELECT * FROM smiths WHERE id = $1 AND is_active = true',
          [sessionData.smithId]
        );

        if (smithQuery.rows.length === 0) {
          await context.redis.del(sessionKey);
          throw new AuthenticationError('Smith not found or inactive');
        }

        const smith = smithQuery.rows[0];

        // Create new session
        const newSessionId = crypto.randomUUID();
        const newExpiresAt = new Date(Date.now() + 24 * 60 * 60 * 1000);

        const newSession = {
          smithId: smith.id,
          token: '',
          refreshToken: crypto.randomUUID(),
          expiresAt: newExpiresAt,
          permissions: smith.permissions || [],
          role: smith.role
        };

        // Create new JWT token
        const newToken = await new SignJWT({
          sub: smith.id,
          email: smith.email,
          role: smith.role,
          sessionId: newSessionId
        })
          .setProtectedHeader({ alg: 'HS256' })
          .setExpirationTime(newExpiresAt)
          .setIssuedAt()
          .sign(JWT_SECRET);

        newSession.token = newToken;

        // Store new session and remove old one
        const newSessionKey = `session:${newSessionId}`;
        await context.redis.setex(newSessionKey, 24 * 60 * 60, JSON.stringify(newSession));
        await context.redis.del(sessionKey);

        return {
          smith,
          session: newSession
        };
      } catch (error) {
        if (error instanceof AuthenticationError) {
          throw error;
        }
        throw new ServiceError('auth', `Token refresh failed: ${error}`);
      }
    },

    async logout(parent: any, args: any, context: Context) {
      const { session } = await requireAuth(context);

      try {
        // Remove session from Redis
        const sessionKey = `session:${session.smithId}`;
        await context.redis.del(sessionKey);

        return true;
      } catch (error) {
        throw new ServiceError('auth', `Logout failed: ${error}`);
      }
    }
  },

  Smith: {
    async preferences(parent: any, args: any, context: Context) {
      // Return default preferences if none set
      return parent.preferences || {
        theme: 'auto',
        language: 'en',
        timezone: 'UTC',
        notifications: {
          email: true,
          push: false
        },
        dashboard: {
          defaultView: 'overview',
          widgets: ['fleets', 'tasks', 'telemetry']
        }
      };
    }
  }
};
