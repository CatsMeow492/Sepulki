export type ModelPreset = 'unitree-dog' | 'industrial-arm' | 'sample-arm'

export function suggestPresetFromAnalysis(userInput: string, analysis: string): ModelPreset {
  const text = `${userInput}\n${analysis}`.toLowerCase()
  const dogHints = ['robot dog', 'quadruped', 'patrol', 'unitree', 'spot']
  const armHints = ['robot arm', 'industrial arm', 'manipulator', 'pick and place', 'welding', 'factory']

  if (dogHints.some((k) => text.includes(k))) return 'unitree-dog'
  if (armHints.some((k) => text.includes(k))) return 'industrial-arm'
  return 'sample-arm'
}


