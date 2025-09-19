# 🔧 Anvil Sim - Isaac Sim Integration Service

The Anvil Sim service provides physics simulation, validation, and 3D scene management using NVIDIA Isaac Sim. This service acts as the bridge between the Forge UI and Isaac Sim for realistic robot simulation.

## Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Forge UI      │◄──►│   Anvil Sim      │◄──►│   Isaac Sim     │
│  (React/R3F)    │    │  (Python/gRPC)   │    │  (Omniverse)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Features

### Core Simulation Capabilities
- **Physics Validation** - Real-time physics simulation with accurate dynamics
- **Collision Detection** - Advanced collision checking for robot designs
- **Stress Testing** - Material and structural stress analysis
- **Performance Modeling** - Latency, throughput, and efficiency prediction

### 3D Scene Management  
- **Scene Generation** - Automatic scene creation from Sepulka designs
- **Asset Loading** - Dynamic loading of URDF models and 3D meshes
- **Environment Setup** - Configurable simulation environments
- **Camera Management** - Multiple viewports and camera controls

### Integration Points
- **gRPC API** - High-performance communication with Forge UI
- **WebSocket Stream** - Real-time simulation data streaming
- **GraphQL Subscriptions** - Live updates through Hammer Orchestrator
- **File System Bridge** - Asset synchronization with Vault Registry

## Isaac Sim Integration

### Prerequisites
- NVIDIA Isaac Sim 2023.1.1+
- NVIDIA Omniverse Launcher
- CUDA-compatible GPU (RTX 3060+)
- Python 3.10+ with Isaac Sim Python API

### Installation & Setup

1. **Install Isaac Sim**
   ```bash
   # Download from NVIDIA Developer Portal
   # Install via Omniverse Launcher
   ```

2. **Configure Python Environment**
   ```bash
   # Use Isaac Sim's Python environment
   export ISAACSIM_PYTHON_EXE="~/.local/share/ov/pkg/isaac_sim-*/python.sh"
   ```

3. **Install Service Dependencies**
   ```bash
   cd services/anvil-sim
   $ISAACSIM_PYTHON_EXE -m pip install -r requirements.txt
   ```

### Service Configuration

```python
# anvil_config.py
ISAAC_SIM_CONFIG = {
    "headless": True,  # Set to False for development
    "width": 1920,
    "height": 1080, 
    "physics_dt": 1.0 / 240.0,  # 240Hz physics
    "rendering_dt": 1.0 / 60.0,  # 60Hz rendering
    "enable_livestream": True,
    "livestream_port": 8211
}

SIMULATION_ENVIRONMENTS = {
    "warehouse": "/World/Warehouse",
    "factory_floor": "/World/Factory", 
    "lab": "/World/Lab",
    "outdoor": "/World/Outdoor"
}
```

## API Reference

### gRPC Service Definition

```protobuf
syntax = "proto3";

service AnvilSim {
  // Scene Management
  rpc CreateScene(SceneRequest) returns (SceneResponse);
  rpc LoadRobot(RobotRequest) returns (RobotResponse);
  rpc UpdateRobot(UpdateRequest) returns (UpdateResponse);
  
  // Simulation Control
  rpc StartSimulation(SimRequest) returns (SimResponse);
  rpc StepSimulation(StepRequest) returns (StepResponse);
  rpc StopSimulation(StopRequest) returns (StopResponse);
  
  // Analysis & Validation
  rpc ValidateDesign(ValidateRequest) returns (ValidationReport);
  rpc RunStressTest(StressRequest) returns (StressReport);
  rpc CheckCollisions(CollisionRequest) returns (CollisionReport);
  
  // Streaming
  rpc StreamTelemetry(TelemetryRequest) returns (stream TelemetryData);
  rpc StreamVideo(VideoRequest) returns (stream VideoFrame);
}
```

### WebSocket Events

```json
{
  "event": "simulation_update",
  "data": {
    "timestamp": "2024-01-15T10:30:00Z",
    "robot_poses": {
      "robot_1": {
        "position": [0.5, 0.2, 0.8],
        "orientation": [0, 0, 0, 1],
        "joint_positions": {"joint_1": 0.5, "joint_2": -0.3}
      }
    },
    "physics_metrics": {
      "total_energy": 145.2,
      "collision_count": 0,
      "stability_score": 0.98
    }
  }
}
```

## Development Workflow

### 1. Design Validation Pipeline

```python
async def validate_sepulka_design(sepulka_id: str):
    # Load design from Vault Registry
    design = await load_sepulka_design(sepulka_id)
    
    # Generate Isaac Sim scene
    scene = await create_isaac_sim_scene(design)
    
    # Run validation tests
    physics_validation = await run_physics_test(scene)
    collision_validation = await run_collision_test(scene) 
    stress_validation = await run_stress_test(scene)
    
    # Generate report
    return ValidationReport(
        physics=physics_validation,
        collisions=collision_validation,
        stress=stress_validation,
        overall_score=calculate_score([
            physics_validation,
            collision_validation, 
            stress_validation
        ])
    )
```

### 2. Real-time Simulation

```python
class SimulationManager:
    def __init__(self):
        self.isaac_sim = IsaacSim(config=ISAAC_SIM_CONFIG)
        self.active_simulations = {}
        
    async def start_simulation(self, sepulka_id: str, environment: str):
        # Load robot and environment
        robot = await self.load_robot_model(sepulka_id)
        env = await self.load_environment(environment)
        
        # Initialize simulation
        sim_id = str(uuid.uuid4())
        simulation = Simulation(
            id=sim_id,
            robot=robot,
            environment=env,
            isaac_sim=self.isaac_sim
        )
        
        # Start simulation loop
        await simulation.start()
        self.active_simulations[sim_id] = simulation
        
        return sim_id
```

### 3. Performance Optimization

```python
# Optimize simulation performance
PERFORMANCE_SETTINGS = {
    "physics_substeps": 4,
    "enable_gpu_dynamics": True,
    "use_fabric": True,  # Isaac Sim Fabric for better performance
    "caching_enabled": True,
    "lod_enabled": True,  # Level-of-detail for complex meshes
    "culling_enabled": True  # Frustum culling
}
```

## Forge UI Integration

### React Component Integration

```typescript
// SimulationViewer.tsx
import { useAnvilSimulation } from '@sepulki/anvil-client';

export function SimulationViewer({ sepulkaId }: { sepulkaId: string }) {
  const { 
    startSimulation, 
    stopSimulation, 
    telemetryStream,
    videoStream 
  } = useAnvilSimulation();

  const handleValidate = async () => {
    const report = await startSimulation({
      sepulkaId,
      environment: 'warehouse',
      validationMode: true
    });
    
    console.log('Validation Report:', report);
  };

  return (
    <div className="simulation-viewer">
      <div className="video-stream">
        <IsaacSimStream stream={videoStream} />
      </div>
      
      <div className="telemetry-panel">
        <TelemetryDisplay data={telemetryStream} />
      </div>
      
      <div className="controls">
        <button onClick={handleValidate}>
          🔧 Run Anvil Validation
        </button>
      </div>
    </div>
  );
}
```

## Deployment

### Docker Configuration

```dockerfile
# Use Isaac Sim base image
FROM nvcr.io/nvidia/isaac-sim:2023.1.1

# Install service dependencies
COPY requirements.txt .
RUN python -m pip install -r requirements.txt

# Copy service code
COPY src/ ./src/
COPY config/ ./config/

# Expose ports
EXPOSE 8000  # gRPC
EXPOSE 8001  # WebSocket
EXPOSE 8211  # Isaac Sim Livestream

# Start service
CMD ["python", "src/main.py"]
```

### Kubernetes Deployment

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: anvil-sim
spec:
  replicas: 1  # Single replica due to GPU requirements
  selector:
    matchLabels:
      app: anvil-sim
  template:
    metadata:
      labels:
        app: anvil-sim
    spec:
      nodeSelector:
        accelerator: nvidia-gpu
      containers:
      - name: anvil-sim
        image: sepulki/anvil-sim:latest
        resources:
          limits:
            nvidia.com/gpu: 1
          requests:
            nvidia.com/gpu: 1
        ports:
        - containerPort: 8000
        - containerPort: 8001  
        - containerPort: 8211
```

## Testing Strategy

### Unit Tests
- Physics calculation accuracy
- URDF parsing and validation
- Scene generation logic

### Integration Tests  
- Isaac Sim API integration
- gRPC service endpoints
- WebSocket streaming

### Performance Tests
- Simulation step timing
- Memory usage optimization
- GPU utilization efficiency

### Validation Tests
- Collision detection accuracy
- Stress analysis precision
- Real-world physics correlation

This service bridges the gap between web-based 3D visualization and professional-grade physics simulation, enabling users to validate and optimize their robot designs before deployment.
