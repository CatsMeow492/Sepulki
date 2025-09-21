/**
 * Isaac Sim Robot Assets Catalog
 * Based on NVIDIA Isaac Sim 4.5.0 built-in robot models and asset packs
 * Reference: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#isaac-sim-setup-assets-content-pack
 */

export interface IsaacSimRobot {
  id: string
  name: string
  category: 'manipulator' | 'mobile' | 'humanoid' | 'industrial' | 'collaborative'
  manufacturer: string
  description: string
  
  // Isaac Sim asset paths (within Isaac Sim's asset library)
  isaac_sim_path: string
  urdf_path?: string
  usd_path?: string
  
  // Robot specifications
  specifications: {
    payload_kg?: number
    reach_m?: number
    repeatability_mm?: number
    dof: number
    workspace_description: string
  }
  
  // Use case suitability
  use_cases: string[]
  environments: ('warehouse' | 'factory' | 'lab' | 'outdoor' | 'cleanroom')[]
  
  // Isaac Sim specific features
  isaac_sim_features: {
    physics_simulation: boolean
    collision_detection: boolean
    force_feedback: boolean
    path_planning: boolean
    vision_integration: boolean
  }
}

/**
 * Isaac Sim Built-in Robot Assets Catalog
 * These are the robot models available in NVIDIA Isaac Sim's default asset library
 */
export const ISAAC_SIM_ROBOTS: IsaacSimRobot[] = [
  {
    id: 'franka_panda',
    name: 'Franka Emika Panda',
    category: 'collaborative',
    manufacturer: 'Franka Emika',
    description: 'Collaborative 7-DOF robot arm designed for human-robot interaction',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Franka/franka.usd',
    urdf_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Franka/franka.urdf',
    specifications: {
      payload_kg: 3,
      reach_m: 0.855,
      repeatability_mm: 0.1,
      dof: 7,
      workspace_description: 'Collaborative workspace, precision assembly tasks'
    },
    use_cases: ['assembly', 'packaging', 'quality_inspection', 'research', 'education'],
    environments: ['factory', 'lab', 'cleanroom'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: true,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'ur5e',
    name: 'Universal Robots UR5e',
    category: 'collaborative', 
    manufacturer: 'Universal Robots',
    description: 'Collaborative industrial robot arm with 5kg payload',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/UR5e/ur5e.usd',
    urdf_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/UR5e/ur5e.urdf',
    specifications: {
      payload_kg: 5,
      reach_m: 0.85,
      repeatability_mm: 0.03,
      dof: 6,
      workspace_description: 'Mid-range collaborative automation'
    },
    use_cases: ['assembly', 'machine_tending', 'packaging', 'palletizing'],
    environments: ['factory', 'warehouse', 'lab'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true, 
      force_feedback: true,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'ur10e',
    name: 'Universal Robots UR10e',
    category: 'industrial',
    manufacturer: 'Universal Robots', 
    description: 'Heavy-duty collaborative robot with 10kg payload',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/UR10e/ur10e.usd',
    urdf_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/UR10e/ur10e.urdf',
    specifications: {
      payload_kg: 10,
      reach_m: 1.3,
      repeatability_mm: 0.05,
      dof: 6,
      workspace_description: 'Heavy-duty automation and material handling'
    },
    use_cases: ['palletizing', 'material_handling', 'welding', 'machine_tending'],
    environments: ['warehouse', 'factory'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: true,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'nova_carter',
    name: 'NVIDIA Nova Carter',
    category: 'mobile',
    manufacturer: 'NVIDIA',
    description: 'Autonomous mobile robot platform with advanced AI navigation',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Carter/nova_carter.usd',
    specifications: {
      payload_kg: 20,
      reach_m: 0, // Mobile robot
      repeatability_mm: 10,
      dof: 0, // Mobile platform (wheels)
      workspace_description: 'Autonomous navigation and material transport'
    },
    use_cases: ['warehouse_logistics', 'material_transport', 'patrol', 'mapping'],
    environments: ['warehouse', 'factory', 'outdoor'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: false,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'jetbot',
    name: 'NVIDIA Jetbot',
    category: 'mobile',
    manufacturer: 'NVIDIA',
    description: 'Small educational autonomous robot for AI learning',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Jetbot/jetbot.usd',
    specifications: {
      payload_kg: 0.5,
      reach_m: 0,
      repeatability_mm: 50,
      dof: 0,
      workspace_description: 'Educational and research applications'
    },
    use_cases: ['education', 'research', 'prototyping', 'ai_development'],
    environments: ['lab', 'outdoor'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: false,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'kuka_kr210',
    name: 'KUKA KR210',
    category: 'industrial',
    manufacturer: 'KUKA',
    description: 'Heavy-duty industrial robot for automotive and manufacturing',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Kuka/kr210.usd',
    urdf_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Kuka/kr210.urdf',
    specifications: {
      payload_kg: 210,
      reach_m: 2.7,
      repeatability_mm: 0.06,
      dof: 6,
      workspace_description: 'Heavy manufacturing and automotive applications'
    },
    use_cases: ['automotive_assembly', 'heavy_lifting', 'welding', 'material_handling'],
    environments: ['factory'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: true, 
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'turtlebot3',
    name: 'TurtleBot3',
    category: 'mobile',
    manufacturer: 'ROBOTIS',
    description: 'Educational and research mobile robot platform',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/TurtleBot3/turtlebot3.usd',
    urdf_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/TurtleBot3/turtlebot3.urdf',
    specifications: {
      payload_kg: 2,
      reach_m: 0,
      repeatability_mm: 20,
      dof: 0,
      workspace_description: 'Indoor navigation and research'
    },
    use_cases: ['education', 'research', 'mapping', 'navigation'],
    environments: ['lab', 'warehouse'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: false,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'allegro_hand',
    name: 'Allegro Hand',
    category: 'manipulator',
    manufacturer: 'Wonik Robotics',
    description: 'Dexterous robotic hand with 16 DOF for complex manipulation',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/AllegroHand/allegro_hand.usd',
    specifications: {
      payload_kg: 0.5,
      reach_m: 0.2,
      repeatability_mm: 1,
      dof: 16,
      workspace_description: 'Dexterous manipulation and grasping'
    },
    use_cases: ['research', 'dexterous_manipulation', 'grasping', 'assembly'],
    environments: ['lab', 'cleanroom'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: true,
      path_planning: true,
      vision_integration: true
    }
  },
  
  {
    id: 'denso_cobotta',
    name: 'DENSO Cobotta',
    category: 'collaborative',
    manufacturer: 'DENSO',
    description: 'Compact collaborative robot for small workspace automation',
    isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta.usd',
    specifications: {
      payload_kg: 0.5,
      reach_m: 0.342,
      repeatability_mm: 0.05,
      dof: 6,
      workspace_description: 'Small parts assembly and testing'
    },
    use_cases: ['small_parts_assembly', 'testing', 'education', 'prototyping'],
    environments: ['lab', 'cleanroom', 'factory'],
    isaac_sim_features: {
      physics_simulation: true,
      collision_detection: true,
      force_feedback: true,
      path_planning: true,
      vision_integration: true
    }
  }
];

/**
 * Select appropriate Isaac Sim robot based on requirements analysis
 */
export function selectRobotFromRequirements(
  requirements: string,
  analysisText: string
): IsaacSimRobot[] {
  const req = requirements.toLowerCase();
  const analysis = analysisText.toLowerCase();
  
  const selectedRobots: IsaacSimRobot[] = [];
  
  // Heavy lifting / high payload requirements
  if (req.includes('heavy') || req.includes('210kg') || req.includes('200kg') || analysis.includes('heavy')) {
    selectedRobots.push(ISAAC_SIM_ROBOTS.find(r => r.id === 'kuka_kr210')!);
  }
  
  // Warehouse automation
  if (req.includes('warehouse') || req.includes('pick') || req.includes('pack') || req.includes('logistics')) {
    selectedRobots.push(
      ISAAC_SIM_ROBOTS.find(r => r.id === 'ur10e')!, // For heavier items
      ISAAC_SIM_ROBOTS.find(r => r.id === 'nova_carter')! // For mobile transport
    );
  }
  
  // Assembly and manufacturing
  if (req.includes('assembly') || req.includes('manufacturing') || req.includes('precision')) {
    selectedRobots.push(
      ISAAC_SIM_ROBOTS.find(r => r.id === 'franka_panda')!, // High precision
      ISAAC_SIM_ROBOTS.find(r => r.id === 'ur5e')! // Versatile collaborative
    );
  }
  
  // Quality inspection
  if (req.includes('inspection') || req.includes('quality') || req.includes('vision')) {
    selectedRobots.push(
      ISAAC_SIM_ROBOTS.find(r => r.id === 'denso_cobotta')!, // Precise positioning
      ISAAC_SIM_ROBOTS.find(r => r.id === 'franka_panda')! // Collaborative inspection
    );
  }
  
  // Research and education
  if (req.includes('research') || req.includes('education') || req.includes('prototype')) {
    selectedRobots.push(
      ISAAC_SIM_ROBOTS.find(r => r.id === 'turtlebot3')!,
      ISAAC_SIM_ROBOTS.find(r => r.id === 'jetbot')!,
      ISAAC_SIM_ROBOTS.find(r => r.id === 'franka_panda')!
    );
  }
  
  // Dexterous manipulation
  if (req.includes('dexterous') || req.includes('hand') || req.includes('grasp') || req.includes('finger')) {
    selectedRobots.push(ISAAC_SIM_ROBOTS.find(r => r.id === 'allegro_hand')!);
  }
  
  // Mobile applications
  if (req.includes('mobile') || req.includes('navigate') || req.includes('autonomous') || req.includes('patrol')) {
    selectedRobots.push(
      ISAAC_SIM_ROBOTS.find(r => r.id === 'nova_carter')!,
      ISAAC_SIM_ROBOTS.find(r => r.id === 'turtlebot3')!
    );
  }
  
  // Default to versatile robots if no specific match
  if (selectedRobots.length === 0) {
    selectedRobots.push(
      ISAAC_SIM_ROBOTS.find(r => r.id === 'franka_panda')!,
      ISAAC_SIM_ROBOTS.find(r => r.id === 'ur5e')!
    );
  }
  
  // Remove duplicates and limit to top 3 recommendations
  const uniqueRobots = selectedRobots.filter((robot, index, self) => 
    index === self.findIndex(r => r.id === robot.id)
  );
  
  return uniqueRobots.slice(0, 3);
}

/**
 * Generate Isaac Sim robot configuration for LLM analysis
 */
export function generateIsaacSimRobotCatalog(): string {
  return `
**ISAAC SIM ROBOT ASSET LIBRARY**

Available robot models in NVIDIA Isaac Sim 4.5.0:

**COLLABORATIVE ROBOTS:**
- Franka Emika Panda: 7-DOF, 3kg payload, 0.855m reach, ±0.1mm precision
  * Use cases: Assembly, packaging, quality inspection, research
  * Features: Force feedback, collision detection, path planning

- Universal Robots UR5e: 6-DOF, 5kg payload, 0.85m reach, ±0.03mm precision  
  * Use cases: Assembly, machine tending, packaging, palletizing
  * Features: Collaborative safety, vision integration

- DENSO Cobotta: 6-DOF, 0.5kg payload, 0.342m reach, ±0.05mm precision
  * Use cases: Small parts assembly, testing, education
  * Features: Compact design, high precision

**INDUSTRIAL ROBOTS:**
- Universal Robots UR10e: 6-DOF, 10kg payload, 1.3m reach, ±0.05mm precision
  * Use cases: Palletizing, material handling, welding, machine tending
  * Features: Heavy-duty automation, large workspace

- KUKA KR210: 6-DOF, 210kg payload, 2.7m reach, ±0.06mm precision
  * Use cases: Automotive assembly, heavy lifting, welding
  * Features: Extreme payload capacity, industrial environments

**MOBILE ROBOTS:**
- NVIDIA Nova Carter: Autonomous mobile platform, 20kg payload
  * Use cases: Warehouse logistics, material transport, patrol, mapping
  * Features: AI navigation, SLAM, autonomous operation

- TurtleBot3: Educational mobile robot, 2kg payload  
  * Use cases: Education, research, mapping, navigation
  * Features: ROS integration, modular design

**SPECIALIZED:**
- Allegro Hand: 16-DOF dexterous hand, 0.5kg payload
  * Use cases: Research, dexterous manipulation, grasping
  * Features: Human-like dexterity, multi-finger control

**ROBOT SELECTION GUIDELINES:**
- For warehouse/logistics: UR10e + Nova Carter combination
- For precision assembly: Franka Panda or DENSO Cobotta  
- For heavy manufacturing: KUKA KR210
- For research/education: TurtleBot3, Jetbot, or Franka Panda
- For complex manipulation: Franka Panda + Allegro Hand

All robots include full Isaac Sim physics simulation, collision detection, and WebRTC streaming support.
`.trim();
}
