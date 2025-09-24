# 🔥 Anvil Sim - Isaac Sim Integration

A streamlined Isaac Sim integration service for the Sepulki Forge robotics platform. Provides real-time physics simulation and video streaming from NVIDIA Isaac Sim to web browsers.

## 🚀 Quick Start on Brev

**One-command setup for Brev instances:**

```bash
# Clone the repository
git clone https://github.com/CatsMeow492/Sepulki.git
cd Sepulki/services/anvil-sim

# Run setup (development mode)
./setup_brev.sh dev

# Or production mode (requires NVIDIA GPU)
./setup_brev.sh prod
```

That's it! The script will:
- ✅ Install all dependencies
- ✅ Set up Isaac Sim (if needed)
- ✅ Start all services
- ✅ Create WebSocket bridge for video streaming
- ✅ Display connection info

## 📊 Service Endpoints

After setup, these endpoints will be available:

| Service | URL | Description |
|---------|-----|-------------|
| **Health Check** | `http://localhost:8002/health` | Service status |
| **gRPC API** | `http://localhost:8000` | Robot simulation API |
| **WebSocket** | `ws://localhost:8765` | Video streaming |
| **Metrics** | `http://localhost:8002/metrics` | Performance data |

## 🎯 Frontend Integration

Connect your frontend to the video stream:

```typescript
const ws = new WebSocket('ws://localhost:8765')

ws.onmessage = (event) => {
  const data = JSON.parse(event.data)
  if (data.type === 'video_frame') {
    // Display frame on canvas
    drawFrameToCanvas(data.frame_data)
  }
}

// Start video stream
ws.send(JSON.stringify({
  type: 'start_video_stream',
  session_id: 'your-session-id'
}))
```

## 🔧 Development Workflow

### Local Development (macOS/Windows)
```bash
./setup_brev.sh dev
# Opens mock Isaac Sim with WebSocket streaming
```

### Production (Linux with GPU)
```bash
./setup_bred.sh prod
# Runs real Isaac Sim with GPU acceleration
```

### Management Commands
```bash
# View logs
docker-compose logs -f

# Stop services
docker-compose down

# Restart services
docker-compose restart
```

## 📁 Project Structure

```
anvil-sim/
├── setup_brev.sh           # Main setup script
├── docker-compose.yml      # Service orchestration
├── Dockerfile             # Container configuration
├── requirements.txt       # Python dependencies
├── src/
│   ├── main.py           # Main service entry point
│   ├── isaac_sim_manager.py    # Isaac Sim integration
│   ├── webrtc_stream_manager.py # Video streaming
│   └── services/         # Service modules
└── config/
    └── anvil_config.py   # Configuration management
```

## 🎨 Video Streaming

The service provides real-time video streaming with:

- **15 FPS** video frame rate
- **1920x1080** resolution (production)
- **JPEG compression** for efficient transmission
- **WebSocket protocol** for low latency
- **Automatic reconnection** on connection loss

### Frame Format
```json
{
  "type": "video_frame",
  "frame_data": "base64-encoded-jpeg",
  "frame_number": 123,
  "timestamp": "2024-01-01T12:00:00Z",
  "width": 1920,
  "height": 1080
}
```

## 🔧 Configuration

Environment variables can be set in `.env` (dev) or `.env.prod` (production):

```bash
# Isaac Sim settings
ANVIL_HEADLESS=true
ANVIL_WIDTH=1920
ANVIL_HEIGHT=1080
ANVIL_PHYSICS_HZ=240
ANVIL_RENDER_HZ=60

# Network settings
PUBLIC_IP=your-public-ip
ISAAC_SIM_WEBSOCKET_PORT=8765
ISAAC_SIM_LIVESTREAM_PORT=49100

# Security
JWT_SECRET=your-secure-secret
ANVIL_AUTH=true
```

## 🐛 Troubleshooting

### Common Issues

**"Isaac Sim not available"**
- Development mode: Expected, uses mock Isaac Sim
- Production mode: Check NVIDIA GPU and drivers

**"WebSocket connection failed"**
- Check if port 8765 is open
- Verify service is running: `curl http://localhost:8002/health`

**"Service health check failed"**
- Check logs: `docker-compose logs`
- Verify database connection
- Check disk space

### Debug Commands

```bash
# Check service status
curl http://localhost:8002/health

# View service logs
docker-compose logs -f

# Check GPU availability (production)
nvidia-smi

# Test WebSocket connection
wscat -c ws://localhost:8765
```

## 📈 Performance

### Recommended Hardware

**Development:**
- Any modern CPU
- 8GB RAM
- Docker Desktop

**Production:**
- NVIDIA GPU (RTX 3060+)
- 16GB RAM
- NVIDIA drivers 470+

### Performance Metrics

| Metric | Development | Production |
|--------|-------------|------------|
| **Resolution** | 1280x720 | 1920x1080 |
| **Frame Rate** | 30 FPS | 60 FPS |
| **Physics Hz** | 60 Hz | 240 Hz |
| **Latency** | ~100ms | ~50ms |

## 🔄 Updates

To update the service:

```bash
# Pull latest changes
git pull

# Rebuild and restart
docker-compose down
docker-compose build
docker-compose up -d
```

## 📞 Support

For issues or questions:

1. Check the troubleshooting section above
2. Review service logs: `docker-compose logs`
3. Verify hardware requirements
4. Check network connectivity

---

**🚀 Ready to build amazing robotics simulations with Isaac Sim!**