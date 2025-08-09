import OpenAI from 'openai';

if (!process.env.OPENAI_API_KEY) {
  throw new Error('Missing OPENAI_API_KEY environment variable');
}

export const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY,
});

export const analyzeRequirements = async (userInput: string, catalogText?: string) => {
  const response = await openai.chat.completions.create({
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
}; 