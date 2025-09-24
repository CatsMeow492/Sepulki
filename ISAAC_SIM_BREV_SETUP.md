# Isaac Sim Setup on Brev - Complete Guide

This guide will help you set up Isaac Sim on your Brev instance to work with the anvil-sim service and frontend video streaming.

## 🎯 Overview

We'll set up Isaac Sim in a Docker container and create a WebSocket service that bridges the container to your frontend, providing real-time video streaming.

## 📋 Prerequisites

- Brev instance with NVIDIA GPU (L40S detected ✅)
- Docker installed and running ✅
- Git repository cloned ✅

## 🚀 Step-by-Step Setup

### 1. Connect to Brev Instance

```bash
brev shell awesome-gpu-name
```

### 2. Navigate to Project Directory

```bash
cd ~/Sepulki/services/anvil-sim
```

### 3. Run Isaac Sim Setup Script

```bash
./setup_isaac_sim_brev.sh
```

This script will:
- ✅ Check Isaac Sim installation
- ✅ Install Isaac Sim if needed (takes ~30 minutes)
- ✅ Create proper symbolic links
- ✅ Start Isaac Sim Docker container with WebSocket support
- ✅ Configure ports (49100 for livestream, 8765 for WebSocket)
- ✅ Test connectivity

### 4. Start Isaac Sim Docker Client

```bash
./start_isaac_docker.sh
```

This will start a WebSocket service that:
- ✅ Listens on port 8765 for frontend connections
- ✅ Generates mock Isaac Sim video frames
- ✅ Provides real-time video streaming

### 5. Test Frontend Connection

1. **Local Frontend**: Visit `http://localhost:3000/isaac-video`
2. **Remote Frontend**: Visit `http://YOUR_BREV_IP:3000/isaac-video`

## 🔧 Configuration Details

### Ports Used

- **49100**: Isaac Sim livestream port (Docker container)
- **8765**: WebSocket port (our service)
- **3000**: Frontend development server

### Network Configuration

- **Isaac Sim Container**: Runs with `--network=host` for full network access
- **WebSocket Service**: Listens on `0.0.0.0:8765` for external connections
- **Frontend**: Connects to `ws://localhost:8765` (local) or `ws://BREV_IP:8765` (remote)

## 🐛 Troubleshooting

### Check Container Status

```bash
docker ps | grep isaac-sim
docker logs isaac-sim
```

### Check Port Availability

```bash
netstat -tlnp | grep -E ':(49100|8765)'
```

### Test WebSocket Connection

```bash
curl -I http://localhost:8765
```

### Check Isaac Sim Installation

```bash
ls -la /home/shadeform/isaac-sim/
ls -la /isaac-sim/
```

## 📱 Frontend Integration

The minimal video component (`IsaacSimVideoOnly.tsx`) will:

1. ✅ Connect to WebSocket at `ws://localhost:8765`
2. ✅ Receive video frames as base64 JPEG data
3. ✅ Display frames on HTML5 canvas
4. ✅ Show connection status and frame count
5. ✅ Provide reconnect functionality

## 🎬 Expected Behavior

### Successful Setup

- ✅ Isaac Sim container running
- ✅ WebSocket service listening on port 8765
- ✅ Frontend shows "Connected" status
- ✅ Video frames streaming at 15 FPS
- ✅ Mock Isaac Sim visualization with robot animation

### Video Content

The mock frames include:
- 🎨 Isaac Sim-style HUD overlay
- 🤖 Animated Franka Emika Panda robot
- 📊 Physics simulation indicators
- 🎯 Professional Isaac Sim branding

## 🔄 Development Workflow

1. **Make changes to frontend** → Automatically reloads
2. **Modify video generation** → Restart `start_isaac_docker.sh`
3. **Update Isaac Sim config** → Restart `setup_isaac_sim_brev.sh`

## 📊 Monitoring

### Service Logs

```bash
# Isaac Sim container logs
docker logs -f isaac-sim

# WebSocket service logs (in terminal running start_isaac_docker.sh)
# Shows connection status, frame counts, errors
```

### Frontend Console

Check browser console for:
- WebSocket connection status
- Video frame reception
- Canvas rendering
- Error messages

## 🎯 Next Steps

Once basic video streaming works:

1. **Replace mock frames** with real Isaac Sim rendering
2. **Add robot control** via WebSocket commands
3. **Implement physics simulation** controls
4. **Add camera controls** and scene management
5. **Integrate with main configure page**

## 📞 Support

If you encounter issues:

1. Check container status: `docker ps`
2. Verify ports: `netstat -tlnp`
3. Review logs: `docker logs isaac-sim`
4. Test connectivity: `curl -I http://localhost:8765`

The setup provides a solid foundation for Isaac Sim integration with clear separation between container, service, and frontend layers.
