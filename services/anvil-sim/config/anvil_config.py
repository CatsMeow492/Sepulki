"""
Anvil Sim Configuration
Settings for Isaac Sim integration and service configuration.
"""

import os
from typing import Dict, Any

# Service Configuration
GRPC_PORT = int(os.getenv("ANVIL_GRPC_PORT", "8000"))
WEBSOCKET_PORT = int(os.getenv("ANVIL_WEBSOCKET_PORT", "8001"))
METRICS_PORT = int(os.getenv("ANVIL_METRICS_PORT", "8002"))

# Isaac Sim Configuration
ISAAC_SIM_CONFIG: Dict[str, Any] = {
    "headless": os.getenv("ANVIL_HEADLESS", "true").lower() == "true",
    "width": int(os.getenv("ANVIL_WIDTH", "1920")),
    "height": int(os.getenv("ANVIL_HEIGHT", "1080")),
    "physics_dt": 1.0 / float(os.getenv("ANVIL_PHYSICS_HZ", "240")),  # 240Hz physics
    "rendering_dt": 1.0 / float(os.getenv("ANVIL_RENDER_HZ", "60")),  # 60Hz rendering
    "enable_livestream": os.getenv("ANVIL_LIVESTREAM", "true").lower() == "true",
    "livestream_port": int(os.getenv("ANVIL_LIVESTREAM_PORT", "8211")),
    "experience": os.getenv("ANVIL_EXPERIENCE", ""),  # Custom Isaac Sim experience file
    "renderer": os.getenv("ANVIL_RENDERER", "RayTracedLighting"),
    "anti_aliasing": int(os.getenv("ANVIL_ANTI_ALIASING", "3")),
}

# Simulation Environments
SIMULATION_ENVIRONMENTS = {
    "warehouse": {
        "usd_path": "/World/Environments/Warehouse.usd",
        "description": "Warehouse environment with shelving and conveyor belts",
        "lighting": "warehouse_hdri",
        "physics_scene": "warehouse_physics"
    },
    "factory_floor": {
        "usd_path": "/World/Environments/FactoryFloor.usd", 
        "description": "Manufacturing floor with assembly lines",
        "lighting": "factory_hdri",
        "physics_scene": "factory_physics"
    },
    "lab": {
        "usd_path": "/World/Environments/Lab.usd",
        "description": "Clean laboratory environment",
        "lighting": "lab_hdri", 
        "physics_scene": "lab_physics"
    },
    "outdoor": {
        "usd_path": "/World/Environments/Outdoor.usd",
        "description": "Outdoor terrain for mobile robots",
        "lighting": "outdoor_hdri",
        "physics_scene": "outdoor_physics"
    },
    "custom": {
        "usd_path": "/World/Environments/Custom.usd",
        "description": "User-defined custom environment", 
        "lighting": "custom_hdri",
        "physics_scene": "custom_physics"
    }
}

# Performance Settings
PERFORMANCE_SETTINGS = {
    "physics_substeps": int(os.getenv("ANVIL_PHYSICS_SUBSTEPS", "4")),
    "enable_gpu_dynamics": os.getenv("ANVIL_GPU_DYNAMICS", "true").lower() == "true",
    "use_fabric": os.getenv("ANVIL_USE_FABRIC", "true").lower() == "true",
    "caching_enabled": os.getenv("ANVIL_CACHING", "true").lower() == "true",
    "lod_enabled": os.getenv("ANVIL_LOD", "true").lower() == "true",
    "culling_enabled": os.getenv("ANVIL_CULLING", "true").lower() == "true",
    "max_concurrent_simulations": int(os.getenv("ANVIL_MAX_SIMS", "4")),
    "memory_limit_gb": int(os.getenv("ANVIL_MEMORY_LIMIT", "16")),
}

# Validation Settings
VALIDATION_SETTINGS = {
    "physics_validation": {
        "enabled": True,
        "duration_seconds": 10.0,
        "stability_threshold": 0.95,
        "energy_conservation_threshold": 0.98
    },
    "collision_validation": {
        "enabled": True,
        "check_self_collision": True,
        "check_environment_collision": True,
        "tolerance_mm": 1.0
    },
    "stress_validation": {
        "enabled": True,
        "max_force_threshold": 1000.0,  # N
        "max_torque_threshold": 100.0,  # Nm  
        "safety_factor": 2.0
    },
    "performance_validation": {
        "enabled": True,
        "target_fps": 60.0,
        "max_simulation_time": 300.0  # seconds
    }
}

# Asset Paths
ASSET_PATHS = {
    "urdf_cache": os.getenv("ANVIL_URDF_CACHE", "/tmp/anvil/urdf"),
    "mesh_cache": os.getenv("ANVIL_MESH_CACHE", "/tmp/anvil/meshes"),
    "scene_cache": os.getenv("ANVIL_SCENE_CACHE", "/tmp/anvil/scenes"),
    "texture_cache": os.getenv("ANVIL_TEXTURE_CACHE", "/tmp/anvil/textures"),
    "environments": os.getenv("ANVIL_ENVIRONMENTS", "/assets/environments"),
    "materials": os.getenv("ANVIL_MATERIALS", "/assets/materials")
}

# Database Configuration  
DATABASE_CONFIG = {
    "url": os.getenv("DATABASE_URL", "postgresql://smith:forge_dev@localhost:5432/sepulki"),
    "pool_size": int(os.getenv("ANVIL_DB_POOL_SIZE", "5")),
    "max_overflow": int(os.getenv("ANVIL_DB_MAX_OVERFLOW", "10"))
}

# Redis Configuration
REDIS_CONFIG = {
    "url": os.getenv("REDIS_URL", "redis://localhost:6379"),
    "prefix": "anvil:",
    "ttl_seconds": int(os.getenv("ANVIL_CACHE_TTL", "3600"))
}

# Logging Configuration
LOGGING_CONFIG = {
    "level": os.getenv("ANVIL_LOG_LEVEL", "INFO"),
    "format": "json",
    "output": os.getenv("ANVIL_LOG_OUTPUT", "stdout"),
    "enable_metrics": os.getenv("ANVIL_METRICS", "true").lower() == "true"
}

# Security Configuration
SECURITY_CONFIG = {
    "enable_auth": os.getenv("ANVIL_AUTH", "true").lower() == "true",
    "jwt_secret": os.getenv("JWT_SECRET", "your-secret-key"),
    "allowed_origins": os.getenv("ANVIL_CORS_ORIGINS", "http://localhost:3000").split(","),
    "max_request_size": int(os.getenv("ANVIL_MAX_REQUEST_SIZE", "10485760"))  # 10MB
}

# Feature Flags
FEATURE_FLAGS = {
    "enable_video_streaming": os.getenv("ANVIL_VIDEO_STREAMING", "true").lower() == "true",
    "enable_telemetry_streaming": os.getenv("ANVIL_TELEMETRY_STREAMING", "true").lower() == "true", 
    "enable_stress_testing": os.getenv("ANVIL_STRESS_TESTING", "true").lower() == "true",
    "enable_ai_validation": os.getenv("ANVIL_AI_VALIDATION", "false").lower() == "true",
    "enable_cloud_rendering": os.getenv("ANVIL_CLOUD_RENDERING", "false").lower() == "true"
}
