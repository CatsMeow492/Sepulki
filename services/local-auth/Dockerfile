# Multi-stage Dockerfile for Local Auth Service
FROM node:25-alpine AS builder
RUN apk add --no-cache python3 make g++
WORKDIR /app
COPY package*.json ./
COPY services/local-auth/package*.json ./services/local-auth/
RUN npm ci --workspace=@sepulki/local-auth --include-workspace-root
COPY services/local-auth ./services/local-auth/
WORKDIR /app/services/local-auth
RUN npm run build

FROM node:25-alpine AS production
RUN apk add --no-cache dumb-init
WORKDIR /app
COPY package*.json ./
COPY services/local-auth/package*.json ./services/local-auth/
RUN npm ci --workspace=@sepulki/local-auth --include-workspace-root --omit=dev
COPY --from=builder /app/services/local-auth/dist ./services/local-auth/dist
RUN addgroup -g 1001 -S nodejs && adduser -S nodejs -u 1001
RUN chown -R nodejs:nodejs /app
USER nodejs
WORKDIR /app/services/local-auth
EXPOSE 3001
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 CMD node -e "require('http').get('http://localhost:3001/health', (r) => {process.exit(r.statusCode === 200 ? 0 : 1)})"
ENTRYPOINT ["dumb-init", "--"]
CMD ["node", "dist/index.js"]
