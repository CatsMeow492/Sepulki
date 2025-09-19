FROM node:18-alpine

# Install dependencies for native modules
RUN apk add --no-cache python3 make g++

# Set working directory
WORKDIR /app

# Copy package files
COPY package*.json ./
COPY packages/shared-types/package*.json ./packages/shared-types/
COPY packages/graphql-schema/package*.json ./packages/graphql-schema/

# Install dependencies
RUN npm ci --only=production

# Copy source code
COPY packages/ ./packages/
COPY services/hammer-orchestrator/ ./services/hammer-orchestrator/

# Build shared packages
RUN cd packages/shared-types && npm run build

# Build the service
WORKDIR /app/services/hammer-orchestrator
RUN npm run build

# Expose port
EXPOSE 4000

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:4000/health || exit 1

# Start the service
CMD ["npm", "start"]
