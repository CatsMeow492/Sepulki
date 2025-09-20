import express from 'express';
import cors from 'cors';
import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import { Pool } from 'pg';
import Redis from 'ioredis';
import { v4 as uuidv4 } from 'uuid';

const app = express();
const PORT = process.env.PORT || 4446;

// Database connection (same as main app)
const db = new Pool({
  connectionString: process.env.DATABASE_URL || 'postgresql://smith:forge_dev@localhost:5432/sepulki',
});

// Redis for sessions
const redis = new Redis(process.env.REDIS_URL || 'redis://localhost:6379');

// JWT secret
const JWT_SECRET = process.env.JWT_SECRET || 'local-auth-jwt-secret';

// Middleware
app.use(cors({
  origin: process.env.CORS_ORIGIN || 'http://localhost:3000',
  credentials: true,
}));
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Serve login page (mimics Auth.js sign-in page)
app.get('/auth/signin', (req, res) => {
  const { callbackUrl = 'http://localhost:3000/dashboard' } = req.query;
  
  res.send(`
    <!DOCTYPE html>
    <html>
    <head>
        <title>🔥 Sepulki - Sign In</title>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { 
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', sans-serif;
                background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
                margin: 0;
                padding: 20px;
                min-height: 100vh;
                display: flex;
                align-items: center;
                justify-content: center;
            }
            .container { 
                background: white;
                padding: 40px;
                border-radius: 12px;
                box-shadow: 0 10px 25px rgba(0,0,0,0.1);
                width: 100%;
                max-width: 400px;
            }
            .logo { 
                text-align: center;
                font-size: 2.5em;
                margin-bottom: 10px;
            }
            .title {
                text-align: center;
                font-size: 1.8em;
                font-weight: 600;
                color: #1f2937;
                margin-bottom: 8px;
            }
            .subtitle {
                text-align: center;
                color: #6b7280;
                margin-bottom: 30px;
            }
            .form-group { 
                margin-bottom: 20px;
            }
            label { 
                display: block;
                margin-bottom: 8px;
                font-weight: 500;
                color: #374151;
            }
            input { 
                width: 100%;
                padding: 12px 16px;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                font-size: 16px;
                transition: border-color 0.2s;
                box-sizing: border-box;
            }
            input:focus { 
                outline: none;
                border-color: #ea580c;
                box-shadow: 0 0 0 3px rgba(234, 88, 12, 0.1);
            }
            .btn { 
                width: 100%;
                background: #ea580c;
                color: white;
                border: none;
                padding: 14px 20px;
                border-radius: 8px;
                font-size: 16px;
                font-weight: 600;
                cursor: pointer;
                transition: background-color 0.2s;
            }
            .btn:hover { 
                background: #dc2626;
            }
            .test-users {
                margin-top: 30px;
                padding: 20px;
                background: #f3f4f6;
                border-radius: 8px;
            }
            .test-users h3 {
                margin: 0 0 15px 0;
                color: #374151;
                font-size: 1.1em;
            }
            .user-badge {
                display: inline-block;
                background: #dbeafe;
                color: #1e40af;
                padding: 4px 8px;
                border-radius: 4px;
                font-size: 0.85em;
                margin: 2px;
                cursor: pointer;
                transition: background-color 0.2s;
            }
            .user-badge:hover {
                background: #bfdbfe;
            }
            .dev-mode {
                text-align: center;
                margin-top: 20px;
                padding: 15px;
                background: #fef3c7;
                border-radius: 8px;
                color: #92400e;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="logo">🔥</div>
            <h1 class="title">Sepulki Forge</h1>
            <p class="subtitle">Local Authentication Service</p>
            
            <form id="loginForm" action="/auth/signin" method="POST">
                <input type="hidden" name="callbackUrl" value="${callbackUrl}">
                
                <div class="form-group">
                    <label for="email">Email Address</label>
                    <input type="email" id="email" name="email" placeholder="dev@sepulki.com" required>
                </div>
                
                <div class="form-group">
                    <label for="password">Password</label>
                    <input type="password" id="password" name="password" placeholder="dev123" required>
                </div>
                
                <button type="submit" class="btn">Sign In to Sepulki</button>
            </form>
            
            <div class="test-users">
                <h3>🧪 Test Users (Click to Auto-Fill)</h3>
                <div onclick="fillUser('dev@sepulki.com', 'dev123')" class="user-badge">
                    🔧 Dev Smith (Over-Smith)
                </div>
                <div onclick="fillUser('demo@sepulki.com', 'demo123')" class="user-badge">
                    🎯 Demo Smith (Smith)
                </div>
                <div onclick="fillUser('test@sepulki.com', 'test123')" class="user-badge">
                    🧪 Test Smith (Over-Smith)
                </div>
                <div onclick="fillUser('admin@sepulki.com', 'admin123')" class="user-badge">
                    👑 Admin (Admin)
                </div>
            </div>
            
            <div class="dev-mode">
                🏗️ Local Development Mode<br>
                <small>This mimics Auth.js production experience locally</small>
            </div>
        </div>
        
        <script>
            function fillUser(email, password) {
                document.getElementById('email').value = email;
                document.getElementById('password').value = password;
            }
            
            // Handle form submission
            document.getElementById('loginForm').addEventListener('submit', async (e) => {
                e.preventDefault();
                const formData = new FormData(e.target);
                
                try {
                    const response = await fetch('/auth/signin', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({
                            email: formData.get('email'),
                            password: formData.get('password'),
                            callbackUrl: formData.get('callbackUrl')
                        }),
                        credentials: 'include'
                    });
                    
                    const result = await response.json();
                    
                    if (result.success) {
                        window.location.href = result.callbackUrl;
                    } else {
                        alert('Login failed: ' + result.error);
                    }
                } catch (error) {
                    alert('Login error: ' + error.message);
                }
            });
        </script>
    </body>
    </html>
  `);
});

// Login endpoint (mimics Auth.js callback)
app.post('/auth/signin', async (req, res) => {
  const { email, password, callbackUrl } = req.body;

  try {
    // Find user in database (same as production)
    const userQuery = await db.query(
      'SELECT id, email, name, password_hash, role, permissions FROM smiths WHERE email = $1 AND is_active = true',
      [email.toLowerCase()]
    );

    if (userQuery.rows.length === 0) {
      return res.json({ success: false, error: 'Invalid credentials' });
    }

    const user = userQuery.rows[0];

    // Verify password (using the same SHA256 hash as your seed data)
    const crypto = require('crypto');
    const passwordHash = crypto.createHash('sha256').update(password).digest('hex');
    
    if (user.password_hash !== passwordHash) {
      return res.json({ success: false, error: 'Invalid credentials' });
    }

    // Create session (mimics Auth.js session)
    const sessionId = uuidv4();
    const sessionData = {
      sessionToken: sessionId,
      userId: user.id,
      expires: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
    };

    // Store session in Redis (same as production)
    await redis.setex(`session:${sessionId}`, 24 * 60 * 60, JSON.stringify({
      smith: {
        id: user.id,
        email: user.email,
        name: user.name,
        role: user.role,
        permissions: user.permissions,
      },
      ...sessionData
    }));

    // Create JWT (same format as production)
    const token = jwt.sign({
      sub: user.id,
      email: user.email,
      name: user.name,
      role: user.role,
      sessionId: sessionId,
    }, JWT_SECRET, { expiresIn: '24h' });

    // Set session cookie (mimics NextAuth.js)
    res.cookie('next-auth.session-token', sessionId, {
      httpOnly: true,
      secure: false, // true in production
      sameSite: 'lax',
      maxAge: 24 * 60 * 60 * 1000
    });

    // Update last login (same as production)
    await db.query(
      'UPDATE smiths SET last_login_at = NOW() WHERE id = $1',
      [user.id]
    );

    res.json({ 
      success: true, 
      callbackUrl: callbackUrl || 'http://localhost:3000/dashboard',
      token // For GraphQL API calls
    });

  } catch (error) {
    console.error('Login error:', error);
    res.json({ success: false, error: 'Internal server error' });
  }
});

// Session endpoint (mimics Auth.js API)
app.get('/auth/session', async (req, res) => {
  const sessionToken = req.cookies['next-auth.session-token'] || 
                      req.headers.authorization?.replace('Bearer ', '');

  if (!sessionToken) {
    return res.json({ user: null });
  }

  try {
    // Get session from Redis
    const sessionData = await redis.get(`session:${sessionToken}`);
    
    if (!sessionData) {
      return res.json({ user: null });
    }

    const session = JSON.parse(sessionData);
    
    // Check if session is expired
    if (new Date() > new Date(session.expires)) {
      await redis.del(`session:${sessionToken}`);
      return res.json({ user: null });
    }

    res.json({
      user: session.smith,
      expires: session.expires
    });

  } catch (error) {
    console.error('Session error:', error);
    res.json({ user: null });
  }
});

// Sign out endpoint (mimics Auth.js)
app.post('/auth/signout', async (req, res) => {
  const sessionToken = req.cookies['next-auth.session-token'];

  if (sessionToken) {
    await redis.del(`session:${sessionToken}`);
  }

  res.clearCookie('next-auth.session-token');
  res.json({ success: true, url: 'http://localhost:3000' });
});

// CSRF token endpoint (mimics Auth.js)
app.get('/auth/csrf', (req, res) => {
  const csrfToken = uuidv4();
  res.json({ csrfToken });
});

// Providers endpoint (mimics Auth.js)
app.get('/auth/providers', (req, res) => {
  res.json({
    'local-credentials': {
      id: 'local-credentials',
      name: 'Local Database',
      type: 'credentials',
      signinUrl: 'http://localhost:4446/auth/signin',
      callbackUrl: 'http://localhost:4446/auth/signin'
    }
  });
});

// Health check
app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    service: 'local-auth',
    version: '1.0.0',
    timestamp: new Date().toISOString(),
    mode: 'LocalStack equivalent for Auth.js'
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`🔐 Local Auth Service ready at http://localhost:${PORT}`);
  console.log(`🧪 This provides the same experience as production Auth.js, but locally`);
  console.log(`📋 Sign in page: http://localhost:${PORT}/auth/signin`);
  console.log(`🎯 Test users: dev@sepulki.com (dev123), demo@sepulki.com (demo123)`);
});

// Graceful shutdown
process.on('SIGTERM', () => {
  console.log('SIGTERM received, shutting down gracefully');
  process.exit(0);
});

process.on('SIGINT', () => {
  console.log('SIGINT received, shutting down gracefully');
  process.exit(0);
});
