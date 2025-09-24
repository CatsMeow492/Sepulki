# 🛠️ Developer Workflow Guide

This guide explains how to develop, test, and deploy the Anvil Sim service.

## 🚀 Getting Started

### 1. Clone and Setup
```bash
git clone https://github.com/CatsMeow492/Sepulki.git
cd Sepulki/services/anvil-sim
./setup_brev.sh dev
```

### 2. Verify Setup
```bash
# Check service health
curl http://localhost:8002/health

# Test WebSocket connection
wscat -c ws://localhost:8765
```

## 🔧 Development Workflow

### Local Development (macOS/Windows)

```bash
# Start development environment
./setup_brev.sh dev

# Make code changes
vim src/main.py

# Services auto-reload on changes
# Check logs
docker-compose logs -f
```

### Production Testing (Linux with GPU)

```bash
# Start production environment
./setup_brev.sh prod

# Test Isaac Sim integration
curl http://localhost:8002/health
```

## 📝 Code Structure

```
src/
├── main.py                    # Main service entry point
├── isaac_sim_manager.py       # Isaac Sim integration
├── webrtc_stream_manager.py   # Video streaming
├── anvil_sim_isaac_app.py     # Isaac Sim application
├── isaac_sim_real_renderer.py # Real rendering engine
├── isaac_sim_assets.py        # Asset management
├── media_source.py            # Media handling
└── services/
    └── simulation_service.py  # Simulation logic
```

## 🧪 Testing

### Unit Tests
```bash
# Run tests
pytest tests/

# Run specific test
pytest tests/test_isaac_sim.py
```

### Integration Tests
```bash
# Test WebSocket connection
python -c "
import asyncio
import websockets
import json

async def test():
    async with websockets.connect('ws://localhost:8765') as ws:
        await ws.send(json.dumps({'type': 'start_video_stream'}))
        response = await ws.recv()
        print('Response:', response)

asyncio.run(test())
"
```

### End-to-End Testing
```bash
# Start frontend
cd ../../apps/forge-ui
npm run dev

# Open browser
open http://localhost:3000/isaac-video
```

## 🐛 Debugging

### View Logs
```bash
# All services
docker-compose logs -f

# Specific service
docker-compose logs -f anvil-sim-dev

# Isaac Sim container (production)
docker logs isaac-sim
```

### Debug Mode
```bash
# Enable debug logging
export ANVIL_LOG_LEVEL=DEBUG
docker-compose restart
```

### Common Issues

**WebSocket Connection Failed**
```bash
# Check port availability
netstat -tlnp | grep 8765

# Test connection
telnet localhost 8765
```

**Isaac Sim Not Loading**
```bash
# Check GPU
nvidia-smi

# Check container
docker ps | grep isaac-sim
docker logs isaac-sim
```

**Service Health Check Failed**
```bash
# Check database
docker-compose exec postgres pg_isready

# Check Redis
docker-compose exec redis redis-cli ping
```

## 🔄 Deployment

### Development Deployment
```bash
./setup_brev.sh dev
```

### Production Deployment
```bash
./setup_brev.sh prod
```

### Update Deployment
```bash
# Pull latest code
git pull

# Rebuild and restart
docker-compose down
docker-compose build
docker-compose up -d
```

## 📊 Monitoring

### Health Checks
```bash
# Service health
curl http://localhost:8002/health

# Metrics
curl http://localhost:8002/metrics

# WebSocket status
curl -I http://localhost:8765
```

### Performance Monitoring
```bash
# GPU usage (production)
nvidia-smi

# Container stats
docker stats

# Memory usage
docker-compose exec anvil-sim-dev ps aux
```

## 🔧 Configuration

### Environment Variables
```bash
# Development
export ANVIL_LOG_LEVEL=DEBUG
export ANVIL_HEADLESS=false

# Production
export ANVIL_LOG_LEVEL=INFO
export ANVIL_HEADLESS=true
export ANVIL_GPU_DYNAMICS=true
```

### Docker Compose
```bash
# Scale services
docker-compose up -d --scale anvil-sim-dev=2

# Custom configuration
docker-compose --env-file .env.custom up -d
```

## 🚀 Performance Optimization

### Development Mode
- Use mock Isaac Sim for faster startup
- Lower resolution (1280x720)
- Reduced physics Hz (60)

### Production Mode
- Real Isaac Sim with GPU acceleration
- High resolution (1920x1080)
- Full physics Hz (240)

### Tuning Parameters
```bash
# Frame rate
ANVIL_RENDER_HZ=60

# Physics rate
ANVIL_PHYSICS_HZ=240

# Resolution
ANVIL_WIDTH=1920
ANVIL_HEIGHT=1080
```

## 📚 API Reference

### WebSocket Messages
```typescript
// Start video stream
{
  "type": "start_video_stream",
  "session_id": "session-123"
}

// Stop video stream
{
  "type": "stop_video_stream"
}

// Joint control
{
  "type": "joint_control",
  "joint_states": {
    "joint1": 0.5,
    "joint2": -0.3
  }
}
```

### HTTP Endpoints
```bash
# Health check
GET /health

# Metrics
GET /metrics

# Service info
GET /info
```

## 🔒 Security

### Authentication
```bash
# Enable authentication
export ANVIL_AUTH=true
export JWT_SECRET=your-secure-secret
```

### CORS
```bash
# Configure CORS origins
export ANVIL_CORS_ORIGINS=https://forge.sepulki.com
```

## 📞 Support

### Getting Help
1. Check logs: `docker-compose logs -f`
2. Verify configuration: `curl http://localhost:8002/health`
3. Test connectivity: `wscat -c ws://localhost:8765`
4. Check hardware: `nvidia-smi` (production)

### Common Commands
```bash
# Quick restart
docker-compose restart

# Clean restart
docker-compose down && docker-compose up -d

# View all logs
docker-compose logs --tail=100

# Shell access
docker-compose exec anvil-sim-dev bash
```

---

**Happy coding! 🚀**
