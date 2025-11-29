# Isaac Sim Integration Service Dockerfile
# Multi-stage build for development and production deployment

# Stage 1: Base Isaac Sim environment (for production with NVIDIA GPUs)
FROM nvcr.io/nvidia/isaac-sim:2023.1.1 as isaac-sim-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    curl \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set Isaac Sim environment variables
ENV ISAAC_SIM_ROOT=/isaac-sim
ENV PYTHONPATH="${PYTHONPATH}:${ISAAC_SIM_ROOT}/python_packages"

# Stage 2: Development environment (for local development without Isaac Sim)
FROM python:3.14-slim as development-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create Isaac Sim mock directory structure for development
RUN mkdir -p /isaac-sim/python_packages
ENV ISAAC_SIM_ROOT=/isaac-sim
ENV PYTHONPATH="${PYTHONPATH}:${ISAAC_SIM_ROOT}/python_packages"

# Stage 3: Application setup (common for both environments)
FROM ${BUILD_TARGET:-development-base} as app-base

WORKDIR /app

# Copy requirements
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ ./src/
COPY config/ ./config/

# Create necessary directories
RUN mkdir -p /tmp/anvil/{urdf,meshes,scenes,textures} && \
    mkdir -p /assets/{environments,materials} && \
    mkdir -p /var/log/anvil

# Set permissions
RUN chmod +x src/main.py

# Expose service ports
EXPOSE 8000  # gRPC service
EXPOSE 8001  # WebSocket/WebRTC streaming  
EXPOSE 8002  # Metrics/monitoring
EXPOSE 8211  # Isaac Sim livestream

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:8002/health || exit 1

# Environment variables with defaults
ENV ANVIL_HEADLESS=true
ENV ANVIL_WIDTH=1920
ENV ANVIL_HEIGHT=1080
ENV ANVIL_PHYSICS_HZ=240
ENV ANVIL_RENDER_HZ=60
ENV ANVIL_LIVESTREAM=true
ENV ANVIL_LIVESTREAM_PORT=8211
ENV ANVIL_GRPC_PORT=8000
ENV ANVIL_WEBSOCKET_PORT=8001
ENV ANVIL_METRICS_PORT=8002
ENV ANVIL_LOG_LEVEL=INFO

# Default command (can be overridden)
CMD ["python3", "src/main.py"]

# Production stage (Isaac Sim with GPU support)
FROM isaac-sim-base as production

WORKDIR /app

# Copy requirements and install
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ ./src/
COPY config/ ./config/

# Create directories
RUN mkdir -p /tmp/anvil/{urdf,meshes,scenes,textures} && \
    mkdir -p /assets/{environments,materials} && \
    mkdir -p /var/log/anvil

# Set permissions
RUN chmod +x src/main.py

# Expose ports
EXPOSE 8000 8001 8002 8211

# GPU runtime configuration
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=120s --retries=3 \
    CMD curl -f http://localhost:8002/health || exit 1

# Production command with Isaac Sim
CMD ["python3", "src/main.py"]

# Development stage (no Isaac Sim, for local testing)
FROM development-base as development

WORKDIR /app

# Copy requirements and install minimal dependencies for MVP
COPY requirements-minimal.txt .
RUN pip install --no-cache-dir -r requirements-minimal.txt

# Copy application code
COPY src/ ./src/
COPY config/ ./config/

# Copy development utilities
COPY scripts/ ./scripts/

# Create directories
RUN mkdir -p /tmp/anvil/{urdf,meshes,scenes,textures} && \
    mkdir -p /assets/{environments,materials} && \
    mkdir -p /var/log/anvil

# Set permissions
RUN chmod +x src/main.py scripts/*.sh

# Expose ports
EXPOSE 8000 8001 8002 8211

# Development environment variables
ENV ANVIL_HEADLESS=false
ENV ANVIL_LOG_LEVEL=DEBUG
ENV ANVIL_LIVESTREAM=false

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD curl -f http://localhost:8002/health || exit 1

# Development command - real Isaac Sim service
CMD ["python3", "src/main.py"]
