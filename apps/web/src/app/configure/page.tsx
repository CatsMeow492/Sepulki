'use client';

import { Suspense, useEffect, useRef, useState } from 'react';
import { useSearchParams, useRouter } from 'next/navigation';
import dynamic from 'next/dynamic';
import type { RobotSpec } from '@/types/robot';
import { JointControls } from '@/components';

// Dynamically import the Scene3D component with SSR disabled
const Scene3D = dynamic(
  () => import('@/components/Scene3D').then((mod) => mod.Scene3D),
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
  const [currentStep, setCurrentStep] = useState<number>(2);
  const [analysis, setAnalysis] = useState<string>('');
  const [userInput, setUserInput] = useState<string>('');
  const [sourceTab, setSourceTab] = useState<'spec' | 'urdf'>('spec');
  const [specText, setSpecText] = useState<string>('');
  const [urdfText, setUrdfText] = useState<string>('');
  const [builtSpec, setBuiltSpec] = useState<RobotSpec | undefined>(undefined);
  const [builtUrdf, setBuiltUrdf] = useState<string | undefined>(undefined);
  const [jointSnapshot, setJointSnapshot] = useState<{ name: string; value: number; min?: number; max?: number }[]>([]);
  const [playing, setPlaying] = useState(false)
  const [rate, setRate] = useState(1)
  const [seek, setSeek] = useState<number | undefined>(undefined)
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
  }, [searchParams, router]);

  const loadSample = () => {
    // For MVP, load a static URDF with primitive shapes served from public/
    const sampleUrl = '/robots/sample-arm-01/urdf/sample.urdf'
    setBuiltUrdf(sampleUrl)
    setBuiltSpec(undefined)
    setUrdfText(sampleUrl)
    setSourceTab('urdf')
  }

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

          {/* Suggested Components */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">Suggested Components</h2>
            <div className="space-y-4">
              <div className="flex items-start space-x-4">
                <div className="flex-shrink-0">
                  <div className="w-10 h-10 bg-gray-100 rounded-lg flex items-center justify-center">
                    🤖
                  </div>
                </div>
                <div>
                  <h3 className="font-medium text-gray-900">RT-2000 Arm</h3>
                  <p className="text-sm text-gray-500">High precision, medium payload</p>
                </div>
              </div>
              <div className="flex items-start space-x-4">
                <div className="flex-shrink-0">
                  <div className="w-10 h-10 bg-gray-100 rounded-lg flex items-center justify-center">
                    🎮
                  </div>
                </div>
                <div>
                  <h3 className="font-medium text-gray-900">Smart Controller X1</h3>
                  <p className="text-sm text-gray-500">Advanced path planning</p>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Right Column - 3D Visualization */}
        <div className="bg-gray-50 rounded-lg p-6">
          <div className="aspect-square w-full bg-gray-700 rounded-lg overflow-hidden">
            <Scene3D
              spec={builtSpec}
              urdf={builtUrdf}
              assetBaseUrl="/robots/sample-arm-01"
              profile={builtSpec ? {
                name: 'demo',
                duration: 4,
                loop: true,
                frames: [
                  { t: 0, pose: { joint1: 0 } },
                  { t: 2, pose: { joint1: 1.2 } },
                  { t: 4, pose: { joint1: 0 } },
                ]
              } : undefined}
              playing={playing}
              playbackRate={rate}
              seekTime={seek}
              onRobotApi={(api) => {
                robotApiRef.current = api
                const joints = api.listJoints()
                const snapshot = joints.map((j) => ({
                  name: j.name,
                  value: api.getJointValue(j.name),
                  min: j.limit?.lower,
                  max: j.limit?.upper,
                }))
                setJointSnapshot(snapshot)
              }}
            />
          </div>
          {/* Source selector + inputs */}
          <div className="mt-4">
            <div className="flex gap-2 mb-2">
              <button className={`px-3 py-1 rounded ${sourceTab === 'spec' ? 'bg-blue-600 text-white' : 'bg-gray-200'}`} onClick={() => setSourceTab('spec')}>Spec JSON</button>
              <button className={`px-3 py-1 rounded ${sourceTab === 'urdf' ? 'bg-blue-600 text-white' : 'bg-gray-200'}`} onClick={() => setSourceTab('urdf')}>URDF</button>
              <button className="px-3 py-1 rounded bg-gray-900 text-white" onClick={loadSample}>Load Sample</button>
              <button className="ml-auto px-3 py-1 rounded bg-blue-600 text-white" onClick={buildModel}>Build Model</button>
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
      <div className="flex justify-end space-x-4 mt-8">
        <button
          onClick={() => router.push('/')}
          className="px-6 py-3 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
        >
          Start Over
        </button>
        <button
          onClick={() => router.push('/review')}
          className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
        >
          Continue
        </button>
      </div>
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