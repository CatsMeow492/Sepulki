'use client';

import { Suspense, useEffect, useRef, useState } from 'react';
import { useSearchParams, useRouter } from 'next/navigation';
import Link from 'next/link';
import dynamic from 'next/dynamic';
import { SuggestedComponents } from './SuggestedComponents'
import type { RobotSpec } from '@/types/robot';
import { JointControls, StaticModel } from '@/components';
import { suggestPresetFromAnalysis } from '@/lib/presets';
import { fetchCatalog, selectFromCatalog } from '@/lib/catalog';
import { SaveDesignModal } from '@/components/SaveDesignModal';
import { useAuth } from '@/components/AuthProvider';
import { extractRobotRecommendations, generateIsaacSimConfiguration, type IsaacSimRobot } from '@/lib/openai';

// Dynamically import the Isaac Sim Display with SSR disabled
const IsaacSimDisplay = dynamic(
  () => import('@/components/IsaacSimDisplay').then((mod) => mod.IsaacSimDisplay),
  { ssr: false }
);

const steps = [
  { id: 1, name: 'Use Case' },
  { id: 2, name: 'Components' },
  { id: 3, name: 'Review' },
  { id: 4, name: 'Quote' },
];

const useCaseExamples = [
  {
    type: 'warehouse',
    icon: '🏭',
    title: 'Warehouse Automation',
    example: 'I need a robot to pick and pack items from shelves in my warehouse'
  },
  {
    type: 'assembly',
    icon: '⚙️',
    title: 'Assembly Line',
    example: 'Looking for a robot to assist with circuit board assembly and soldering'
  },
  {
    type: 'quality',
    icon: '🔍',
    title: 'Quality Control',
    example: 'Need automated visual inspection for product defects on our production line'
  }
];

function ConfigureContent() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const { smith } = useAuth();
  const [currentStep, setCurrentStep] = useState<number>(2);
  const [analysis, setAnalysis] = useState<string>('');
  const [userInput, setUserInput] = useState<string>('');
  const [sourceTab, setSourceTab] = useState<'spec' | 'urdf'>('spec');
  const [specText, setSpecText] = useState<string>('');
  const [urdfText, setUrdfText] = useState<string>('');
  const [builtSpec, setBuiltSpec] = useState<RobotSpec | undefined>(undefined);
  const [builtUrdf, setBuiltUrdf] = useState<string | undefined>(undefined);
  const [preset, setPreset] = useState<string>('sample-arm')
  const [jointSnapshot, setJointSnapshot] = useState<{ name: string; value: number; min?: number; max?: number }[]>([]);
  const [recommendedRobots, setRecommendedRobots] = useState<IsaacSimRobot[]>([]);
  const [selectedRobot, setSelectedRobot] = useState<IsaacSimRobot | null>(null);
  const [isaacSimConfig, setIsaacSimConfig] = useState<any>(null);
  const [playing, setPlaying] = useState(false)
  const [rate, setRate] = useState(1)
  const [seek, setSeek] = useState<number | undefined>(undefined)

  // Save design state
  const [showSaveModal, setShowSaveModal] = useState(false);
  const [selectedAlloys, setSelectedAlloys] = useState<string[]>([
    'e604e430-70bd-4614-ba64-4c36c219621d', // ServoMax Pro 3000
    'd86ab673-b375-4b46-823e-c198bc10b6f5'  // GripForce Elite
  ]);
  const [patternId, setPatternId] = useState<string>('3080083a-2de1-4200-bc5c-d92f6c459f05'); // Industrial Arm - 6DOF
  const robotApiRef = useRef<{
    getJoint: (name: string) => any
    setJointValue: (name: string, value: number) => void
    getJointValue: (name: string) => number
    listJoints: () => { name: string; limit?: { lower: number; upper: number } }[]
  } | null>(null)

  useEffect(() => {
    const step = Number(searchParams.get('step')) || 2;
    setCurrentStep(step);
    
    const storedAnalysis = localStorage.getItem('requirementAnalysis');
    const storedInput = localStorage.getItem('userInput');
    
    if (!storedAnalysis || !storedInput) {
      router.push('/');
      return;
    }

    setAnalysis(storedAnalysis);
    setUserInput(storedInput);
    
    // Extract Isaac Sim robot recommendations from analysis
    const robots = extractRobotRecommendations(storedAnalysis);
    console.log('🤖 Extracted robots from analysis:', robots);
    console.log('📄 Analysis text preview:', storedAnalysis.substring(0, 200));

    // TEMP: Force some robots for testing video functionality
    const testRobots = [
      {
        id: 'franka_panda',
        name: 'Franka Emika Panda',
        category: 'collaborative' as const,
        manufacturer: 'Franka Emika',
        description: 'Collaborative 7-DOF robot arm designed for human-robot interaction',
        isaac_sim_path: '/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Franka/franka.usd',
        urdf_path: '/robots/franka_panda/panda.urdf',
        usd_path: '/robots/franka_panda/panda.usd',
        specifications: {
          payload_kg: 3,
          reach_m: 0.855,
          repeatability_mm: 0.1,
          dof: 7,
          workspace_description: 'Spherical workspace with full orientation'
        },
        use_cases: ['assembly', 'packaging', 'quality inspection'],
        environments: ['warehouse', 'factory', 'lab'] as const,
        isaac_sim_features: {
          physics_simulation: true,
          collision_detection: true,
          force_feedback: true,
          path_planning: true,
          vision_integration: true
        }
      }
    ];

    // Select the first recommended robot as default
    const finalRobots = testRobots.length > 0 ? testRobots : robots;
    if (finalRobots.length > 0) {
      setRecommendedRobots(finalRobots);
      setSelectedRobot(finalRobots[0]);
      
      // Generate Isaac Sim configuration
      const config = generateIsaacSimConfiguration(finalRobots, storedInput);
      setIsaacSimConfig(config);
      
      console.log('🤖 Isaac Sim robot recommendations:', finalRobots.map(r => r.name));
      console.log('🏭 Isaac Sim configuration:', config);
    }
  }, [searchParams, router]);

  const loadSample = () => {
    // MVP: use primitives-only URDF to avoid external mesh loaders
    const sampleUrl = '/robots/sample-arm-01/urdf/sample.urdf'
    setBuiltUrdf(sampleUrl)
    setBuiltSpec(undefined)
    setUrdfText(sampleUrl)
    setSourceTab('urdf')
  }

  // Auto-load a sample model once analysis is present (MVP: AI-prepopulated view)
  useEffect(() => {
    const init = async () => {
      if (analysis && !builtUrdf && !builtSpec) {
        const p = suggestPresetFromAnalysis(userInput, analysis)
        setPreset(p)
        const catalog = await fetchCatalog()
        const choice = selectFromCatalog(catalog, p as any)
        if (choice.heroCandidates && choice.heroCandidates.length > 0) {
          // Render hero GLB statically
          setBuiltUrdf(undefined)
          setSourceTab('urdf')
          setUrdfText('')
          // store hero URL in a data attribute for future use if needed
          ;(window as any).__heroUrl = choice.heroCandidates[0]
        } else if (choice.urdf) {
          setBuiltUrdf(choice.urdf)
          setBuiltSpec(undefined)
          setUrdfText(choice.urdf)
          setSourceTab('urdf')
        } else {
          loadSample()
        }
      }
    }
    init()
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [analysis])

  const buildModel = () => {
    try {
      if (sourceTab === 'spec' && specText.trim()) {
        const parsed = JSON.parse(specText) as RobotSpec
        setBuiltSpec(parsed)
        setBuiltUrdf(undefined)
      } else if (sourceTab === 'urdf' && urdfText.trim()) {
        setBuiltUrdf(urdfText)
        setBuiltSpec(undefined)
      }
    } catch (e) {
      // minimal MVP: ignore detailed error surfacing
      console.error(e)
    }
  }

  return (
    <div className="max-w-7xl mx-auto px-4 py-8">
      {/* Progress Steps */}
      <div className="flex justify-center mb-12">
        <div className="flex items-center space-x-4">
          <div className="flex items-center">
            <div className="w-8 h-8 bg-gray-200 text-gray-500 rounded-full flex items-center justify-center">
              1
            </div>
            <span className="ml-2 text-gray-500">Use Case</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className={`w-8 h-8 ${currentStep >= 2 ? 'bg-blue-600 text-white' : 'bg-gray-200 text-gray-500'} rounded-full flex items-center justify-center`}>
              2
            </div>
            <span className={`ml-2 ${currentStep >= 2 ? 'text-blue-600 font-medium' : 'text-gray-500'}`}>Components</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className={`w-8 h-8 ${currentStep >= 3 ? 'bg-blue-600 text-white' : 'bg-gray-200 text-gray-500'} rounded-full flex items-center justify-center`}>
              3
            </div>
            <span className={`ml-2 ${currentStep >= 3 ? 'text-blue-600 font-medium' : 'text-gray-500'}`}>Review</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className={`w-8 h-8 ${currentStep >= 4 ? 'bg-blue-600 text-white' : 'bg-gray-200 text-gray-500'} rounded-full flex items-center justify-center`}>
              4
            </div>
            <span className={`ml-2 ${currentStep >= 4 ? 'text-blue-600 font-medium' : 'text-gray-500'}`}>Quote</span>
          </div>
        </div>
      </div>

      {/* Main Content - Two Column Layout */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
        {/* Left Column - Analysis and Components */}
        <div className="space-y-6">
          {/* AI Analysis Results */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">AI Analysis Results</h2>
            <div className="space-y-4">
              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-gray-700">Weight Capacity</span>
                  <span className="text-gray-900">150kg</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '80%' }}></div>
                </div>
              </div>

              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-gray-700">Reach Distance</span>
                  <span className="text-gray-900">2.5m</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '70%' }}></div>
                </div>
              </div>

              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-gray-700">Speed</span>
                  <span className="text-gray-900">1.2m/s</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '60%' }}></div>
                </div>
              </div>

              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-gray-700">Precision</span>
                  <span className="text-gray-900">±0.1mm</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '90%' }}></div>
                </div>
              </div>
            </div>
          </div>

          {/* Real-time Processing */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">Real-time Processing</h2>
            <div className="space-y-4">
              <div className="flex items-center space-x-3">
                <div className="animate-spin rounded-full h-4 w-4 border-2 border-blue-600 border-t-transparent"></div>
                <span className="text-gray-700">Analyzing workspace requirements...</span>
              </div>
              <div className="flex items-center space-x-3">
                <svg className="h-5 w-5 text-green-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                </svg>
                <span className="text-gray-700">Load capacity calculation complete</span>
              </div>
              <div className="flex items-center space-x-3">
                <svg className="h-5 w-5 text-green-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                </svg>
                <span className="text-gray-700">Movement patterns optimized</span>
              </div>
            </div>
          </div>

          {/* Isaac Sim Robot Recommendations */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">Isaac Sim Robot Recommendations</h2>
            
            {recommendedRobots.length > 0 ? (
              <div className="space-y-4">
                {recommendedRobots.map((robot) => (
                  <div 
                    key={robot.id}
                    className={`border rounded-lg p-4 cursor-pointer transition-all ${
                      selectedRobot?.id === robot.id 
                        ? 'border-blue-500 bg-blue-50' 
                        : 'border-gray-200 hover:border-gray-300'
                    }`}
                    onClick={() => {
                      setSelectedRobot(robot)
                      
                      // Trigger robot change in Isaac Sim
                      const changeRobotInIsaacSim = async () => {
                        try {
                          const sessionId = localStorage.getItem('isaac_sim_session_id')
                          if (!sessionId) {
                            console.log('🤖 No Isaac Sim session ID available for robot change')
                            return
                          }
                          
                          const response = await fetch('http://localhost:8002/change_robot', {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/json' },
                            body: JSON.stringify({
                              session_id: sessionId,
                              isaac_sim_robot: robot
                            })
                          })
                          
                          const result = await response.json()
                          if (result.success) {
                            console.log('🎉 Robot changed in Isaac Sim successfully:', result.robot_name)
                          } else {
                            console.error('❌ Failed to change robot in Isaac Sim:', result.error)
                          }
                        } catch (error) {
                          console.error('❌ Robot change API call failed:', error)
                        }
                      }
                      
                      changeRobotInIsaacSim()
                    }}
                  >
                    <div className="flex items-start space-x-3">
                      <div className="text-2xl">🤖</div>
                      <div className="flex-1">
                        <h3 className="font-semibold text-gray-900">{robot.name}</h3>
                        <p className="text-sm text-gray-600 mb-2">{robot.manufacturer} • {robot.category}</p>
                        <p className="text-sm text-gray-700 mb-3">{robot.description}</p>
                        
                        <div className="grid grid-cols-2 gap-4 text-xs">
                          <div>
                            <span className="font-medium text-gray-500">Payload:</span>
                            <span className="ml-1 text-gray-900">{robot.specifications.payload_kg}kg</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-500">Reach:</span>
                            <span className="ml-1 text-gray-900">{robot.specifications.reach_m}m</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-500">DOF:</span>
                            <span className="ml-1 text-gray-900">{robot.specifications.dof}</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-500">Precision:</span>
                            <span className="ml-1 text-gray-900">±{robot.specifications.repeatability_mm}mm</span>
                          </div>
                        </div>
                        
                        <div className="mt-3 flex flex-wrap gap-1">
                          {robot.use_cases.slice(0, 3).map((useCase) => (
                            <span key={useCase} className="px-2 py-1 bg-gray-100 text-gray-600 text-xs rounded">
                              {useCase.replace('_', ' ')}
                            </span>
                          ))}
                        </div>
                        
                        {selectedRobot?.id === robot.id && (
                          <div className="mt-3 p-3 bg-green-50 border border-green-200 rounded">
                            <div className="text-sm font-medium text-green-800">✅ Selected for Isaac Sim</div>
                            <div className="text-xs text-green-600 mt-1">
                              Will be loaded: {robot.isaac_sim_path}
                            </div>
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <div className="text-center py-8 text-gray-500">
                <div className="text-4xl mb-2">🤖</div>
                <div>No robot recommendations found</div>
                <div className="text-sm">Try analyzing requirements first</div>
              </div>
            )}
          </div>
        </div>

        {/* Right Column - 3D Visualization */}
        <div className="bg-gray-50 rounded-lg p-6">
          <div className="aspect-square w-full bg-gray-700 rounded-lg overflow-hidden relative">
            <IsaacSimDisplay
              spec={builtSpec}
              urdf={selectedRobot?.urdf_path || builtUrdf}
              environment={isaacSimConfig?.environment || "warehouse"}
              qualityProfile="engineering"
              enablePhysics={true}
              userId={smith?.email || 'anonymous'}
              robotConfig={{
                selectedRobot: selectedRobot,
                isaacSimPath: selectedRobot?.isaac_sim_path,
                robotName: selectedRobot?.name,
                physicsConfig: isaacSimConfig?.physics_config
              }}
              onJointControl={(jointStates) => {
                // Update joint display when Isaac Sim controls change
                const snapshot = Object.entries(jointStates).map(([name, value]) => ({
                  name,
                  value,
                  min: -1.57,
                  max: 1.57,
                }))
                setJointSnapshot(snapshot)
              }}
              onError={(error) => {
                console.error('Isaac Sim error:', error)
              }}
              className="w-full h-full"
            />
          </div>
          {/* Source selector + inputs */}
          <div className="mt-4">
            <div className="flex gap-2 mb-2">
              <button className={`px-3 py-1 rounded ${sourceTab === 'spec' ? 'bg-blue-600 text-white' : 'bg-gray-200'}`} onClick={() => setSourceTab('spec')}>Spec JSON</button>
              <button className={`px-3 py-1 rounded ${sourceTab === 'urdf' ? 'bg-blue-600 text-white' : 'bg-gray-200'}`} onClick={() => setSourceTab('urdf')}>URDF</button>
              <button data-testid="load-sample" className="px-3 py-1 rounded bg-gray-900 text-white" onClick={loadSample}>Load Sample</button>
              <button data-testid="build-model" className="ml-auto px-3 py-1 rounded bg-blue-600 text-white" onClick={buildModel}>Build Model</button>
            </div>
            {sourceTab === 'spec' ? (
              <textarea className="w-full h-40 text-sm font-mono p-2 rounded border" placeholder="Paste RobotSpec JSON here" value={specText} onChange={(e) => setSpecText(e.target.value)} />
            ) : (
              <textarea className="w-full h-40 text-sm font-mono p-2 rounded border" placeholder="Paste URDF XML here" value={urdfText} onChange={(e) => setUrdfText(e.target.value)} />
            )}
          </div>
          {/* Playback controls */}
          <div className="mt-4 flex items-center gap-3">
            <button className={`px-3 py-1 rounded ${playing ? 'bg-gray-200' : 'bg-blue-600 text-white'}`} onClick={() => setPlaying(true)}>Play</button>
            <button className={`px-3 py-1 rounded ${!playing ? 'bg-gray-200' : 'bg-blue-600 text-white'}`} onClick={() => setPlaying(false)}>Pause</button>
            <label className="ml-4 text-sm text-gray-700">Rate</label>
            <select className="border rounded p-1 text-sm" value={rate} onChange={(e) => setRate(Number(e.target.value))}>
              <option value={0.5}>0.5×</option>
              <option value={1}>1×</option>
              <option value={2}>2×</option>
            </select>
            <label className="ml-4 text-sm text-gray-700">Seek</label>
            <input type="range" min={0} max={4} step={0.01} className="flex-1" onChange={(e) => setSeek(Number(e.target.value))} />
          </div>

          {/* Joint controls */}
          {jointSnapshot.length > 0 && (
            <div className="mt-4">
              <h3 className="text-sm font-medium text-gray-800 mb-2">Joint Controls</h3>
              <JointControls joints={jointSnapshot} onChange={(name, value) => {
                robotApiRef.current?.setJointValue(name, value)
                setJointSnapshot((prev) => prev.map((j) => (j.name === name ? { ...j, value } : j)))
              }} />
            </div>
          )}
          <div className="mt-4 flex justify-center space-x-4">
            <button className="p-2 text-gray-500 hover:text-gray-700">
              <svg className="w-6 h-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
              </svg>
            </button>
            <button className="p-2 text-gray-500 hover:text-gray-700">
              <svg className="w-6 h-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4" />
              </svg>
            </button>
            <button className="p-2 text-gray-500 hover:text-gray-700">
              <svg className="w-6 h-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
              </svg>
            </button>
          </div>
          <p className="text-center text-sm text-gray-500 mt-2">Use mouse to rotate and zoom</p>
        </div>
      </div>

      {/* Navigation Buttons */}
      <div className="flex justify-between space-x-4 mt-8">
        <button
          onClick={() => router.push('/')}
          className="px-6 py-3 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
        >
          Start Over
        </button>
        
        <div className="flex space-x-3">
          {smith ? (
            <button
              onClick={() => setShowSaveModal(true)}
              className="px-6 py-3 bg-orange-600 text-white rounded-lg hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500 flex items-center"
            >
              🔥 Save Design
            </button>
          ) : (
            <Link
              href="/auth/signin"
              className="px-6 py-3 bg-gray-600 text-white rounded-lg hover:bg-gray-700 flex items-center"
            >
              🔐 Sign In to Save
            </Link>
          )}
          <button
            onClick={() => router.push('/review')}
            className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
          >
            Continue to Review
          </button>
        </div>
      </div>

      {/* Save Design Modal */}
      <SaveDesignModal
        isOpen={showSaveModal}
        onClose={() => setShowSaveModal(false)}
        onSaved={(sepulkaId) => {
          console.log('✅ Design saved with ID:', sepulkaId);
          // Navigate to My Designs page to show the new design
          router.push(`/designs?newDesign=${sepulkaId}`);
        }}
        robotSpec={builtSpec}
        selectedAlloys={selectedAlloys}
        patternId={patternId}
      />
    </div>
  );
} 

export default function ConfigurePage() {
  return (
    <Suspense fallback={<div className="max-w-7xl mx-auto px-4 py-8">Loading...</div>}>
      <ConfigureContent />
    </Suspense>
  );
}