import { ApolloServer } from '@apollo/server';
import { expressMiddleware } from '@apollo/server/express4';
import { ApolloServerPluginLandingPageLocalDefault } from '@apollo/server-plugin-landing-page-local-default';
import express from 'express';
import cors from 'cors';
import { createServer } from 'http';
import { readFileSync } from 'fs';
import { join } from 'path';

import { createContext } from './context';
import { resolvers } from './resolvers';
import { setupDataLoaders } from './dataloaders';
import { AuthenticationError } from './errors';

const typeDefs = readFileSync(
  join(__dirname, '../../../packages/graphql-schema/schema.graphql'),
  'utf8'
);

async function startServer() {
  // Create Express app
  const app = express();
  const httpServer = createServer(app);

  // Create Apollo Server
  const server = new ApolloServer({
    typeDefs,
    resolvers,
    plugins: [
      ApolloServerPluginLandingPageLocalDefault({ embed: true }),
    ],
  });

  await server.start();

  // Apply middleware
  app.use(
    '/graphql',
    cors<cors.CorsRequest>({
      origin: process.env.CORS_ORIGIN || 'http://localhost:3000',
      credentials: true,
    }),
    express.json(),
    expressMiddleware(server, {
      context: async ({ req }) => {
        const authHeader = req.headers.authorization;
        const token = authHeader?.replace('Bearer ', '');
        
        try {
          return await createContext({ token });
        } catch (error) {
          throw new AuthenticationError('Invalid authentication token');
        }
      },
    }),
  );

  // Health check endpoint
  app.get('/health', (req, res) => {
    res.json({ 
      status: 'ok',
      service: 'hammer-orchestrator',
      version: process.env.npm_package_version || '1.0.0',
      timestamp: new Date().toISOString()
    });
  });

  const PORT = process.env.PORT || 4000;
  
  httpServer.listen(PORT, () => {
    console.log(`🔨 Hammer Orchestrator ready at http://localhost:${PORT}/graphql`);
  });
}

// Handle shutdown gracefully
process.on('SIGTERM', () => {
  console.log('SIGTERM received, shutting down gracefully');
  process.exit(0);
});

process.on('SIGINT', () => {
  console.log('SIGINT received, shutting down gracefully');
  process.exit(0);
});

startServer().catch((error) => {
  console.error('Failed to start server:', error);
  process.exit(1);
});
