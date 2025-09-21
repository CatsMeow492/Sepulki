import OpenAI from 'openai';

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
  const isWarehouse = userInput.toLowerCase().includes('warehouse') || userInput.toLowerCase().includes('pick');
  const isAssembly = userInput.toLowerCase().includes('assembly') || userInput.toLowerCase().includes('manufacturing');
  const isInspection = userInput.toLowerCase().includes('inspection') || userInput.toLowerCase().includes('quality');
  
  if (isWarehouse) {
    return `### ANALYSIS & QUESTIONS

**Overview:** Your warehouse automation requirements indicate a need for a high-payload, precision picking system with advanced vision capabilities. This is an excellent fit for our Industrial Arm platform with specialized end effectors.

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

**Next Steps:** This configuration provides an excellent foundation for your warehouse automation needs. The suggested components offer 2x your payload requirements and precision suitable for varied inventory handling. Ready to proceed with detailed configuration and 3D simulation.`;
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

**Next Steps:** This precision assembly configuration offers sub-millimeter accuracy with gentle force control perfect for electronics manufacturing. Ready for detailed simulation and cycle time optimization.`;
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

**Next Steps:** This inspection configuration provides industry-leading defect detection with AI-powered analysis. The system offers sub-millimeter accuracy with real-time decision making perfect for production quality control.`;
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

**Next Steps:** This configuration provides a solid foundation for your automation needs. Ready to proceed with detailed 3D configuration and simulation to optimize the solution for your specific requirements.`;
}

export const analyzeRequirements = async (userInput: string, catalogText?: string) => {
  try {
    if (!hasOpenAIKey) {
      console.log('📺 OpenAI API key not available - using mock analysis for demo');
      // Simulate API delay for realistic demo experience
      await new Promise(resolve => setTimeout(resolve, 2000));
      return generateMockAnalysis(userInput, catalogText);
    }

    const response = await openai!.chat.completions.create({
      model: "gpt-4-turbo-preview",
      messages: [
        {
          role: "system",
          content: `You are an AI robotics expert helping to refine and analyze automation requirements.
          When recommending parts, prefer items present in the provided CATALOG.
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

          ### SUGGESTED CONFIGURATION (Catalog-aware)
          Recommend parts from the provided CATALOG only and use exact paths when possible.
          - Primary platform
          - Controllers / Sensors / End Effector

          ### POTENTIAL REFINEMENTS
          List 3-5 specific ways the solution could be enhanced, each with a brief explanation of the benefits.

          End with a brief concluding statement about next steps.`
        },
        { role: "user", content: `${userInput}` },
        { role: "user", content: `CATALOG (for reference):\n${catalogText || 'N/A'}` }
      ],
      temperature: 0.7,
      max_tokens: 1500,
    });

    return response.choices[0].message.content;
  } catch (error) {
    console.error('OpenAI analysis failed, falling back to mock analysis:', error);
    // Fallback to mock analysis if OpenAI fails
    return generateMockAnalysis(userInput, catalogText);
  }
}; 