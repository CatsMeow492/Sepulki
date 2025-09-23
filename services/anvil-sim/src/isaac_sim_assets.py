#!/usr/bin/env python3
"""
Isaac Sim Robot Assets Configuration
Defines robot models, materials, and Isaac Sim asset paths for photorealistic rendering.
"""

from typing import Dict, Any, List

# Isaac Sim Robot Asset Paths
ISAAC_SIM_ROBOT_ASSETS = {
    "franka_panda": {
        "name": "Franka Emika Panda",
        "isaac_sim_path": "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        "category": "manipulator",
        "manufacturer": "Franka Emika",
        "specifications": {
            "dof": 7,
            "payload_kg": 3,
            "reach_m": 0.855,
            "repeatability_mm": 0.1,
            "weight_kg": 18.4
        },
        "materials": {
            "base": "franka_white",
            "joints": "franka_orange", 
            "gripper": "franka_black"
        },
        "use_cases": [
            "pick and place",
            "assembly",
            "packaging",
            "research"
        ]
    },
    
    "ur10e": {
        "name": "Universal Robots UR10e",
        "isaac_sim_path": "/Isaac/Robots/UniversalRobots/ur10e.usd",
        "category": "manipulator", 
        "manufacturer": "Universal Robots",
        "specifications": {
            "dof": 6,
            "payload_kg": 12.5,
            "reach_m": 1.3,
            "repeatability_mm": 0.03,
            "weight_kg": 33.5
        },
        "materials": {
            "base": "ur_blue",
            "joints": "ur_silver",
            "gripper": "ur_orange"
        },
        "use_cases": [
            "industrial automation",
            "welding",
            "machine tending",
            "material handling"
        ]
    },
    
    "ur5e": {
        "name": "Universal Robots UR5e",
        "isaac_sim_path": "/Isaac/Robots/UniversalRobots/ur5e.usd",
        "category": "manipulator",
        "manufacturer": "Universal Robots", 
        "specifications": {
            "dof": 6,
            "payload_kg": 5,
            "reach_m": 0.85,
            "repeatability_mm": 0.03,
            "weight_kg": 20.6
        },
        "materials": {
            "base": "ur_blue",
            "joints": "ur_silver", 
            "gripper": "ur_orange"
        },
        "use_cases": [
            "light assembly",
            "packaging",
            "quality inspection",
            "laboratory automation"
        ]
    },
    
    "nova_carter": {
        "name": "NVIDIA Nova Carter",
        "isaac_sim_path": "/Isaac/Robots/Nova_Carter/nova_carter.usd",
        "category": "mobile",
        "manufacturer": "NVIDIA",
        "specifications": {
            "dof": 0,
            "payload_kg": 50,
            "reach_m": 0,
            "max_speed_mps": 1.5,
            "weight_kg": 75
        },
        "materials": {
            "platform": "carter_green",
            "wheels": "carter_black",
            "sensors": "carter_silver"
        },
        "use_cases": [
            "warehouse navigation",
            "delivery",
            "security patrol",
            "inspection"
        ]
    },
    
    "kuka_kr210": {
        "name": "KUKA KR210",
        "isaac_sim_path": "/Isaac/Robots/KUKA/kr210.usd",
        "category": "manipulator",
        "manufacturer": "KUKA",
        "specifications": {
            "dof": 6,
            "payload_kg": 210,
            "reach_m": 2.7,
            "repeatability_mm": 0.06,
            "weight_kg": 1200
        },
        "materials": {
            "base": "kuka_orange",
            "joints": "kuka_gray",
            "gripper": "kuka_black"
        },
        "use_cases": [
            "heavy lifting",
            "automotive assembly",
            "foundry operations",
            "large part handling"
        ]
    },
    
    "turtlebot3": {
        "name": "TurtleBot3",
        "isaac_sim_path": "/Isaac/Robots/TurtleBot3/turtlebot3.usd",
        "category": "mobile",
        "manufacturer": "ROBOTIS",
        "specifications": {
            "dof": 0,
            "payload_kg": 5,
            "reach_m": 0,
            "max_speed_mps": 0.22,
            "weight_kg": 1.8
        },
        "materials": {
            "platform": "turtlebot_blue",
            "wheels": "turtlebot_black",
            "lidar": "turtlebot_red"
        },
        "use_cases": [
            "education",
            "research",
            "indoor navigation",
            "SLAM development"
        ]
    }
}

# Isaac Sim Environment Assets
ISAAC_SIM_ENVIRONMENTS = {
    "warehouse": {
        "name": "Warehouse Environment",
        "isaac_sim_path": "/Isaac/Environments/Warehouse/warehouse.usd",
        "lighting": "warehouse_lighting",
        "materials": "industrial_materials"
    },
    
    "factory": {
        "name": "Factory Environment", 
        "isaac_sim_path": "/Isaac/Environments/Factory/factory.usd",
        "lighting": "factory_lighting",
        "materials": "industrial_materials"
    },
    
    "laboratory": {
        "name": "Laboratory Environment",
        "isaac_sim_path": "/Isaac/Environments/Lab/laboratory.usd", 
        "lighting": "lab_lighting",
        "materials": "clean_room_materials"
    },
    
    "outdoor": {
        "name": "Outdoor Environment",
        "isaac_sim_path": "/Isaac/Environments/Outdoor/outdoor.usd",
        "lighting": "natural_lighting",
        "materials": "outdoor_materials"
    }
}

# Isaac Sim Material Definitions
ISAAC_SIM_MATERIALS = {
    "franka_white": {
        "type": "PBR",
        "base_color": [0.95, 0.95, 0.95],
        "metallic": 0.1,
        "roughness": 0.3,
        "specular": 0.5
    },
    
    "franka_orange": {
        "type": "PBR", 
        "base_color": [1.0, 0.6, 0.0],
        "metallic": 0.2,
        "roughness": 0.4,
        "specular": 0.6
    },
    
    "franka_black": {
        "type": "PBR",
        "base_color": [0.1, 0.1, 0.1],
        "metallic": 0.8,
        "roughness": 0.2,
        "specular": 0.9
    },
    
    "ur_blue": {
        "type": "PBR",
        "base_color": [0.0, 0.3, 0.8],
        "metallic": 0.3,
        "roughness": 0.2,
        "specular": 0.7
    },
    
    "ur_silver": {
        "type": "PBR",
        "base_color": [0.8, 0.8, 0.9],
        "metallic": 0.9,
        "roughness": 0.1,
        "specular": 0.95
    },
    
    "ur_orange": {
        "type": "PBR",
        "base_color": [1.0, 0.4, 0.0],
        "metallic": 0.1,
        "roughness": 0.5,
        "specular": 0.4
    },
    
    "carter_green": {
        "type": "PBR",
        "base_color": [0.0, 0.7, 0.3],
        "metallic": 0.1,
        "roughness": 0.6,
        "specular": 0.3
    },
    
    "carter_black": {
        "type": "PBR",
        "base_color": [0.05, 0.05, 0.05],
        "metallic": 0.2,
        "roughness": 0.8,
        "specular": 0.1
    },
    
    "carter_silver": {
        "type": "PBR",
        "base_color": [0.9, 0.9, 0.9],
        "metallic": 0.8,
        "roughness": 0.2,
        "specular": 0.9
    },
    
    "kuka_orange": {
        "type": "PBR",
        "base_color": [1.0, 0.5, 0.0],
        "metallic": 0.2,
        "roughness": 0.4,
        "specular": 0.6
    },
    
    "kuka_gray": {
        "type": "PBR",
        "base_color": [0.6, 0.6, 0.6],
        "metallic": 0.7,
        "roughness": 0.3,
        "specular": 0.8
    },
    
    "kuka_black": {
        "type": "PBR",
        "base_color": [0.1, 0.1, 0.1],
        "metallic": 0.9,
        "roughness": 0.1,
        "specular": 0.95
    },
    
    "turtlebot_blue": {
        "type": "PBR",
        "base_color": [0.0, 0.4, 0.8],
        "metallic": 0.1,
        "roughness": 0.7,
        "specular": 0.2
    },
    
    "turtlebot_black": {
        "type": "PBR",
        "base_color": [0.1, 0.1, 0.1],
        "metallic": 0.3,
        "roughness": 0.8,
        "specular": 0.1
    },
    
    "turtlebot_red": {
        "type": "PBR",
        "base_color": [0.8, 0.1, 0.1],
        "metallic": 0.1,
        "roughness": 0.6,
        "specular": 0.3
    }
}

# Isaac Sim Rendering Settings
ISAAC_SIM_RENDERING_SETTINGS = {
    "photorealistic": {
        "renderer": "RayTracedLighting",
        "rtx_settings": {
            "enable_sampled_direct_lighting": True,
            "enable_denoising": True,
            "max_bounces": 16,
            "samples_per_pixel": 128,
            "enable_rtx_di": True,
            "enable_rtx_gi": True
        },
        "anti_aliasing": "TAA",
        "motion_blur": True,
        "depth_of_field": True
    },
    
    "engineering": {
        "renderer": "RayTracedLighting",
        "rtx_settings": {
            "enable_sampled_direct_lighting": True,
            "enable_denoising": True,
            "max_bounces": 8,
            "samples_per_pixel": 64,
            "enable_rtx_di": True,
            "enable_rtx_gi": False
        },
        "anti_aliasing": "TAA",
        "motion_blur": False,
        "depth_of_field": False
    },
    
    "demo": {
        "renderer": "RayTracedLighting",
        "rtx_settings": {
            "enable_sampled_direct_lighting": True,
            "enable_denoising": True,
            "max_bounces": 4,
            "samples_per_pixel": 32,
            "enable_rtx_di": False,
            "enable_rtx_gi": False
        },
        "anti_aliasing": "FXAA",
        "motion_blur": False,
        "depth_of_field": False
    }
}

def get_robot_asset(robot_name: str) -> Dict[str, Any]:
    """Get Isaac Sim robot asset configuration."""
    return ISAAC_SIM_ROBOT_ASSETS.get(robot_name.lower().replace(" ", "_"), {})

def get_environment_asset(environment_name: str) -> Dict[str, Any]:
    """Get Isaac Sim environment asset configuration."""
    return ISAAC_SIM_ENVIRONMENTS.get(environment_name.lower(), {})

def get_material_definition(material_name: str) -> Dict[str, Any]:
    """Get Isaac Sim material definition."""
    return ISAAC_SIM_MATERIALS.get(material_name, {})

def get_rendering_settings(quality_profile: str) -> Dict[str, Any]:
    """Get Isaac Sim rendering settings for quality profile."""
    return ISAAC_SIM_RENDERING_SETTINGS.get(quality_profile, ISAAC_SIM_RENDERING_SETTINGS["demo"])

def list_available_robots() -> List[str]:
    """List all available Isaac Sim robot models."""
    return list(ISAAC_SIM_ROBOT_ASSETS.keys())

def list_available_environments() -> List[str]:
    """List all available Isaac Sim environments."""
    return list(ISAAC_SIM_ENVIRONMENTS.keys())

