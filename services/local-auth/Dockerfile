FROM node:18-alpine

# Install dependencies for native modules
RUN apk add --no-cache python3 make g++

# Set working directory
WORKDIR /app

# Copy package files
COPY services/local-auth/package*.json ./

# Install dependencies
RUN npm install

# Copy source code
COPY services/local-auth/ ./

# Build the service
RUN npm run build

# Expose port
EXPOSE 4446

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:4446/health || exit 1

# Start the service
CMD ["npm", "start"]
