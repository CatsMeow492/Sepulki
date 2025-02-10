import OpenAI from 'openai';

if (!process.env.OPENAI_API_KEY) {
  throw new Error('Missing OPENAI_API_KEY environment variable');
}

export const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY,
});

export const analyzeRequirements = async (userInput: string) => {
  const response = await openai.chat.completions.create({
    model: "gpt-4-turbo-preview",
    messages: [
      {
        role: "system",
        content: `You are an AI robotics expert helping to refine and analyze automation requirements. 
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

        ### SUGGESTED CONFIGURATION
        Based on typical restaurant requirements, here's our recommended configuration:

        **Recommended Robot Model:**
        - Universal Robots UR5e Collaborative Robot Arm
        - Payload: 5 kg (11 lbs)
        - Reach: 850 mm
        - Degrees of freedom: 6

        **Key Components:**
        - Robotiq 2F-85 Adaptive Gripper
        - Intel RealSense D435 3D Camera
        - Custom end-effector with food-grade coating
        - Safety-rated force/torque sensor
        - IP65-rated protective covering

        **Safety Features:**
        - Force and speed limiting
        - Advanced collision detection
        - Emergency stop buttons
        - Safety light curtains
        - Protective barriers where needed

        **Integration Points:**
        - Standard 110/220V power connection
        - Ethernet/IP communication protocol
        - Digital I/O for dishwasher signals
        - Built-in TCP/IP server

        **Estimated Performance Metrics:**
        - Cycle time: 5-7 seconds per dish
        - Operating speed: Up to 400 dishes per hour
        - Uptime: >99% with proper maintenance
        - Power consumption: 350W typical

        ### POTENTIAL REFINEMENTS
        List 3-5 specific ways the solution could be enhanced, each with a brief explanation of the benefits.

        End with a brief concluding statement about next steps.`
      },
      {
        role: "user",
        content: userInput
      }
    ],
    temperature: 0.7,
    max_tokens: 1500,
  });

  return response.choices[0].message.content;
}; 