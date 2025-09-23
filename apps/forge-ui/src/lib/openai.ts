import OpenAI from 'openai';
import { generateIsaacSimRobotCatalog, selectRobotFromRequirements, ISAAC_SIM_ROBOTS } from './isaac-sim-robots';

// Check if OpenAI API key is available
const hasOpenAIKey = !!process.env.OPENAI_API_KEY;

let openai: OpenAI | null = null;

if (hasOpenAIKey) {
  openai = new OpenAI({
    apiKey: process.env.OPENAI_API_KEY!,
  });
}

// Mock analysis for demo/development when OpenAI isn't available
function generateMockAnalysis(userInput: string, catalogText?: string): string {
  // Get Isaac Sim robot recommendations based on requirements
  const recommendedRobots = selectRobotFromRequirements(userInput, '');
  const isaacSimCatalog = generateIsaacSimRobotCatalog();
  const isWarehouse = userInput.toLowerCase().includes('warehouse') || userInput.toLowerCase().includes('pick');
  const isAssembly = userInput.toLowerCase().includes('assembly') || userInput.toLowerCase().includes('manufacturing');
  const isInspection = userInput.toLowerCase().includes('inspection') || userInput.toLowerCase().includes('quality');
  
  if (isWarehouse) {
    return `### ANALYSIS & QUESTIONS

**Overview:** Your warehouse automation requirements indicate a need for a high-payload, precision picking system with advanced vision capabilities. Based on your requirements, I recommend the Franka Emika Panda and Universal Robots UR10e for your warehouse operations.

**Recommended Robots:**
- Franka Emika Panda: Excellent for collaborative warehouse operations with 7kg payload capacity
- Universal Robots UR10e: Industrial-grade arm with 10kg payload for heavy warehouse tasks

**Safety Requirements:**
- What safety protocols are required for human-robot collaboration zones?
- Are emergency stop systems and light curtains needed around the workspace?
- What certification standards (ISO 10218, ANSI/RIA R15.06) must be met?

**Technical Specifications:**
- What is the maximum cycle time requirement for pick-and-pack operations?
- Are there specific accuracy requirements for item placement (±1mm, ±5mm)?
- What communication protocols are needed for WMS integration?

**Environmental Considerations:**
- What are the operating temperature and humidity ranges in your facility?
- Are there dust, moisture, or vibration considerations?
- What lighting conditions will the vision system operate under?

**Integration Requirements:**
- How should the robot integrate with your existing conveyor systems?
- What data exchange is needed with your WMS/ERP systems?
- Are there specific PLC or fieldbus communication requirements?

**Maintenance Needs:**
- What is your preferred maintenance schedule (daily, weekly, monthly)?
- Do you have in-house robotics technicians or need remote support?
- What predictive maintenance capabilities are desired?

### SUGGESTED CONFIGURATION (Catalog-aware)

**Primary Platform:** Industrial Arm - 6DOF
- Payload capacity: 50kg (exceeds your 25kg requirement)
- Reach: 2.8m (meets your 2.5m requirement)
- Repeatability: ±0.1mm for precision picking

**Recommended Components:**
- **Actuator:** ServoMax Pro 3000 (high-torque, industrial-grade)
- **End Effector:** GripForce Elite (adaptive gripper for varied item sizes)
- **Vision System:** VisionEye 4K (precision object recognition)
- **Controller:** MotionController X1 (real-time path planning)

### POTENTIAL REFINEMENTS

1. **Multi-gripper System:** Deploy multiple end effectors for different item types, increasing versatility and throughput
2. **Predictive Analytics:** Add AI-powered demand forecasting to optimize picking sequences
3. **Collaborative Zones:** Implement human-robot collaboration for complex items requiring manual intervention
4. **Fleet Coordination:** Scale to multiple robots with centralized task allocation for peak efficiency
5. **Quality Assurance:** Integrate weight verification and visual inspection at pick points

### ISAAC SIM ROBOT RECOMMENDATIONS

${recommendedRobots.map(robot => `
**${robot.name}** (${robot.manufacturer})
- Isaac Sim Path: ${robot.isaac_sim_path}
- Specifications: ${robot.specifications.payload_kg}kg payload, ${robot.specifications.reach_m}m reach, ${robot.specifications.dof} DOF
- Fit: ${robot.description}
- Isaac Sim Features: Real-time physics, collision detection, force feedback
`).join('\n')}

### SIMULATION CONFIGURATION
- Environment: Warehouse with shelving, conveyor systems, and loading docks
- Camera Angles: Isometric view for overview, close-up for precision tasks
- Physics: Full collision detection with safety boundaries
- End Effector: Adaptive gripper with force feedback

**Next Steps:** Selected Isaac Sim robots will be loaded with full physics simulation for real-time testing and validation.`;
  }
  
  if (isAssembly) {
    return `### ANALYSIS & QUESTIONS

**Overview:** Your electronics assembly requirements suggest a need for high-precision manipulation with micro-positioning capabilities. This application demands exceptional accuracy and gentle handling for delicate components.

**Safety Requirements:**
- What ESD (Electrostatic Discharge) protection is required for sensitive components?
- Are clean room protocols needed for your assembly environment?
- What emergency procedures are needed for component damage prevention?

**Technical Specifications:**
- What positioning accuracy is required for component placement (±0.01mm, ±0.1mm)?
- What is the smallest component size the robot must handle?
- What force feedback requirements exist for delicate assembly operations?

**Environmental Considerations:**
- What vibration isolation requirements exist for precision assembly?
- Are temperature-controlled environments needed for consistent accuracy?
- What contamination control measures are required?

**Integration Requirements:**
- How should the robot interface with your SMT lines and testing equipment?
- What quality control integration is needed (AOI, ICT, functional test)?
- Are there specific industry standards (IPC, J-STD) that must be followed?

**Maintenance Needs:**
- What calibration frequency is required to maintain precision?
- How should consumable parts (grippers, tips) be managed and replaced?
- What documentation is needed for FDA/ISO compliance?

### SUGGESTED CONFIGURATION (Catalog-aware)

**Primary Platform:** Precision Assembler - 4DOF
- Positioning accuracy: ±0.01mm
- Force control: 0.1N to 50N range
- Vibration isolation: Integrated dampening

**Recommended Components:**
- **Actuator:** PrecisionDrive Mini (ultra-precise positioning)
- **End Effector:** MicroGripper v2 (handles 0.1mm to 10mm components)
- **Sensor:** ForceGuard Pro (tactile feedback for delicate operations)
- **Vision:** MicroVision 8K (sub-pixel component recognition)

### POTENTIAL REFINEMENTS

1. **Vision-Guided Placement:** Add real-time fiducial tracking for ±0.005mm placement accuracy
2. **Multi-Tool Turret:** Deploy tool changer for different assembly operations (pick, place, press, test)
3. **Quality Integration:** Inline inspection with automatic rework for failed assemblies
4. **Traceability System:** Component-level tracking for complete manufacturing history
5. **Adaptive Control:** Machine learning for optimizing force profiles based on component types

### ISAAC SIM ROBOT RECOMMENDATIONS

${recommendedRobots.map(robot => `
**${robot.name}** (${robot.manufacturer})
- Isaac Sim Path: ${robot.isaac_sim_path}
- Specifications: ${robot.specifications.payload_kg}kg payload, ${robot.specifications.reach_m}m reach, ${robot.specifications.dof} DOF
- Fit: ${robot.description}
- Isaac Sim Features: Real-time physics, collision detection, force feedback
`).join('\n')}

### SIMULATION CONFIGURATION
- Environment: Clean factory floor with assembly stations and tooling
- Camera Angles: Side view for assembly monitoring, close-up for precision verification
- Physics: High-precision collision detection with force constraints
- End Effector: Precision gripper with tactile feedback

**Next Steps:** Selected Isaac Sim robots will demonstrate assembly operations with realistic physics simulation.`;
  }
  
  if (isInspection) {
    return `### ANALYSIS & QUESTIONS

**Overview:** Your quality inspection requirements indicate a need for advanced machine vision with AI-powered defect detection. This application requires high-resolution imaging and intelligent analysis capabilities.

**Safety Requirements:**
- What safety measures are needed around high-speed inspection lines?
- Are there radiation or laser safety considerations for imaging systems?
- What fail-safe procedures are required for defective product handling?

**Technical Specifications:**
- What defect size detection threshold is required (0.1mm, 0.5mm, 1mm)?
- What inspection speed requirements exist (parts per minute)?
- Are there specific lighting or contrast requirements for optimal imaging?

**Environmental Considerations:**
- What vibration isolation is needed for stable high-resolution imaging?
- Are there electromagnetic interference considerations from production equipment?
- What ambient lighting variations must the system accommodate?

**Integration Requirements:**
- How should the system integrate with your quality management software?
- What data logging and reporting capabilities are required?
- Are there specific industry compliance standards (ISO 9001, Six Sigma)?

**Maintenance Needs:**
- What calibration procedures are required for consistent inspection accuracy?
- How should camera and lighting systems be maintained and cleaned?
- What backup and redundancy systems are needed for critical quality checks?

### SUGGESTED CONFIGURATION (Catalog-aware)

**Primary Platform:** Inspection Platform - 3DOF
- Multi-axis positioning for optimal viewing angles
- Vibration isolation for stable imaging
- Precision repeatability: ±0.05mm

**Recommended Components:**
- **Actuator:** StealthMove Linear (smooth, vibration-free motion)
- **Vision System:** VisionEye 8K Pro (ultra-high resolution defect detection)
- **Controller:** AI EdgeBox v3 (real-time AI inference and decision making)
- **Lighting:** MultiSpec Illumination (programmable lighting for various materials)

### POTENTIAL REFINEMENTS

1. **360° Inspection:** Add rotating platform for complete product examination from all angles
2. **Multi-Spectral Imaging:** Deploy UV, infrared, and visible light for comprehensive defect detection
3. **AI Learning System:** Continuously improve defect detection through machine learning feedback
4. **Statistical Integration:** Real-time SPC (Statistical Process Control) with trend analysis
5. **Automated Sorting:** Add reject handling system for automated defective product removal

### ISAAC SIM ROBOT RECOMMENDATIONS

${recommendedRobots.map(robot => `
**${robot.name}** (${robot.manufacturer})
- Isaac Sim Path: ${robot.isaac_sim_path}
- Specifications: ${robot.specifications.payload_kg}kg payload, ${robot.specifications.reach_m}m reach, ${robot.specifications.dof} DOF
- Fit: ${robot.description}
- Isaac Sim Features: Real-time physics, collision detection, force feedback
`).join('\n')}

### SIMULATION CONFIGURATION
- Environment: Controlled inspection station with optimal lighting
- Camera Angles: Multiple inspection angles with automated positioning
- Physics: Precise collision detection for delicate part handling
- End Effector: Vision-guided inspection probe

**Next Steps:** Selected Isaac Sim robots will perform quality inspection with real-time AI analysis and reporting.`;
  }
  
  // Generic fallback
  return `### ANALYSIS & QUESTIONS

**Overview:** Your automation requirements have been analyzed and we've identified key areas for clarification to ensure the optimal robot configuration.

**Safety Requirements:**
- What safety standards and protocols must be followed in your workspace?
- Are there specific emergency stop or safety zone requirements?
- What certification levels are required for your industry?

**Technical Specifications:**
- What are the precision and speed requirements for your application?
- What payload and reach specifications are needed?
- Are there specific environmental operating conditions?

**Environmental Considerations:**
- What are the temperature, humidity, and cleanliness requirements?
- Are there vibration, noise, or electromagnetic considerations?
- What space constraints exist in the deployment area?

**Integration Requirements:**
- How should the robot integrate with existing systems and workflows?
- What communication protocols and data exchange are needed?
- Are there specific software or hardware interface requirements?

**Maintenance Needs:**
- What maintenance schedule and support level is preferred?
- Are there specific uptime or reliability requirements?
- What training and documentation needs exist for your team?

### SUGGESTED CONFIGURATION (Catalog-aware)

**Primary Platform:** Industrial Arm - 6DOF
- Versatile multi-axis configuration suitable for various automation tasks
- High payload capacity and precision for demanding applications

**Recommended Components:**
- **Actuator:** ServoMax Pro 3000 (reliable industrial performance)
- **End Effector:** GripForce Elite (adaptive handling capabilities)
- **Sensor:** VisionEye 4K (intelligent object recognition)
- **Controller:** MotionController X1 (advanced motion planning)

### POTENTIAL REFINEMENTS

1. **Enhanced Sensing:** Add force/torque feedback for delicate operations and improved safety
2. **Vision Integration:** Deploy advanced computer vision for intelligent object recognition and handling
3. **Fleet Coordination:** Scale to multiple robots with centralized task allocation and optimization
4. **Predictive Maintenance:** Implement AI-powered monitoring for optimal uptime and performance
5. **Custom Tooling:** Develop application-specific end effectors for maximum efficiency

### ISAAC SIM ROBOT RECOMMENDATIONS

${recommendedRobots.map(robot => `
**${robot.name}** (${robot.manufacturer})
- Isaac Sim Path: ${robot.isaac_sim_path}
- Specifications: ${robot.specifications.payload_kg}kg payload, ${robot.specifications.reach_m}m reach, ${robot.specifications.dof} DOF
- Fit: ${robot.description}
- Isaac Sim Features: Real-time physics, collision detection, force feedback
`).join('\n')}

### SIMULATION CONFIGURATION
- Environment: Versatile workspace adaptable to your specific requirements
- Camera Angles: Multi-angle visualization for comprehensive analysis
- Physics: Full collision detection and constraint validation
- End Effector: Configurable based on application needs

**Next Steps:** Selected Isaac Sim robots provide complete physics simulation for requirements validation and optimization.`;
}

export const analyzeRequirements = async (userInput: string, catalogText?: string) => {
  try {
    if (!hasOpenAIKey) {
      console.log('📺 OpenAI API key not available - using mock analysis for demo');
      // Simulate API delay for realistic demo experience
      await new Promise(resolve => setTimeout(resolve, 2000));
      return generateMockAnalysis(userInput, catalogText);
    }

    // Include Isaac Sim robot catalog in LLM analysis
    const isaacSimCatalog = generateIsaacSimRobotCatalog();

    const response = await openai!.chat.completions.create({
      model: "gpt-4-turbo-preview",
      messages: [
        {
          role: "system",
          content: `You are an AI robotics expert with access to NVIDIA Isaac Sim's professional robot library.
          
          Your goal is to analyze automation requirements and recommend SPECIFIC ISAAC SIM ROBOT MODELS that will be rendered in real-time 3D physics simulation.

          IMPORTANT: You have access to real, industry-standard robot models from the Isaac Sim asset library. 
          Recommend specific models by their exact names and Isaac Sim paths.

          Structure your response in these sections using markdown:

          ### ANALYSIS & QUESTIONS
          Start with a brief overview of the request, then list specific questions about:

          **Safety Requirements:**
          [List 2-3 key safety questions]

          **Technical Specifications:**
          [List 2-3 key technical questions]

          **Environmental Considerations:**
          [List 2-3 key environmental questions]

          **Integration Requirements:**
          [List 2-3 key integration questions]

          **Maintenance Needs:**
          [List 2-3 key maintenance questions]

          ### ISAAC SIM ROBOT RECOMMENDATIONS
          Recommend 1-3 specific robots from the Isaac Sim library that match the requirements.
          For each robot, include:
          - Robot name and manufacturer
          - Isaac Sim asset path
          - Key specifications (payload, reach, DOF)
          - Why it fits the requirements
          - Expected performance in Isaac Sim simulation

          ### SIMULATION CONFIGURATION
          Specify how the selected robots should be configured in Isaac Sim:
          - Environment setting (warehouse, factory, lab, outdoor)
          - Camera angles for optimal visualization  
          - Physics simulation parameters
          - End effector configuration

          ### POTENTIAL REFINEMENTS
          List 3-5 specific ways the Isaac Sim simulation could be enhanced, each with technical details.

          End with a brief statement about next steps for Isaac Sim implementation.`
        },
        { role: "user", content: `USER REQUIREMENTS: ${userInput}` },
        { role: "user", content: `ISAAC SIM ROBOT CATALOG:\n${isaacSimCatalog}` },
        { role: "user", content: `ADDITIONAL CATALOG (for reference):\n${catalogText || 'N/A'}` }
      ],
      temperature: 0.7,
      max_tokens: 2000,
    });

    return response.choices[0].message.content;
  } catch (error) {
    console.error('OpenAI analysis failed, falling back to mock analysis:', error);
    // Fallback to mock analysis if OpenAI fails
    return generateMockAnalysis(userInput, catalogText);
  }
};

/**
 * Extract Isaac Sim robot recommendations from analysis text
 */
export function extractRobotRecommendations(analysisText: string): IsaacSimRobot[] {
  const robots: IsaacSimRobot[] = [];
  
  // Look for robot names mentioned in the analysis
  for (const robot of ISAAC_SIM_ROBOTS) {
    if (analysisText.toLowerCase().includes(robot.name.toLowerCase()) ||
        analysisText.includes(robot.isaac_sim_path)) {
      robots.push(robot);
    }
  }
  
  return robots;
}

/**
 * Generate robot configuration for Isaac Sim service
 */
export function generateIsaacSimConfiguration(robots: IsaacSimRobot[], requirements: string) {
  return {
    robots: robots.map(robot => ({
      id: robot.id,
      name: robot.name,
      isaac_sim_path: robot.isaac_sim_path,
      urdf_path: robot.urdf_path,
      specifications: robot.specifications
    })),
    environment: requirements.toLowerCase().includes('warehouse') ? 'warehouse' :
                requirements.toLowerCase().includes('factory') ? 'factory' :
                requirements.toLowerCase().includes('outdoor') ? 'outdoor' : 'lab',
    physics_config: {
      collision_detection: true,
      force_feedback: true,
      gravity_enabled: true,
      physics_hz: 240
    }
  };
} 