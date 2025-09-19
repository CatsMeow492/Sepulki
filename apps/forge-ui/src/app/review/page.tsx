'use client';

import { useEffect, useState } from 'react';
import { useRouter } from 'next/navigation';
import dynamic from 'next/dynamic';

const Scene3D = dynamic(
  () => import('@/components/Scene3D').then((mod) => mod.Scene3D),
  { ssr: false }
);

export default function ReviewPage() {
  const router = useRouter();
  const [userInput, setUserInput] = useState<string>('');
  const [analysis, setAnalysis] = useState<string>('');

  useEffect(() => {
    const storedInput = localStorage.getItem('userInput');
    const storedAnalysis = localStorage.getItem('requirementAnalysis');
    
    if (!storedInput || !storedAnalysis) {
      router.push('/');
      return;
    }

    setUserInput(storedInput);
    setAnalysis(storedAnalysis);
  }, [router]);

  return (
    <main className="max-w-7xl mx-auto px-4 py-8">
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
            <div className="w-8 h-8 bg-gray-200 text-gray-500 rounded-full flex items-center justify-center">
              2
            </div>
            <span className="ml-2 text-gray-500">Components</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className="w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center">
              3
            </div>
            <span className="ml-2 text-blue-600 font-medium">Review</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className="w-8 h-8 bg-gray-200 text-gray-500 rounded-full flex items-center justify-center">
              4
            </div>
            <span className="ml-2 text-gray-500">Quote</span>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Left Column - Performance Metrics */}
        <div className="lg:col-span-2 space-y-6">
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">Real-time Performance Metrics</h2>
            <div className="grid grid-cols-2 gap-6">
              <div className="space-y-2">
                <div className="flex justify-between">
                  <span className="text-gray-600">Total Cost</span>
                  <span className="font-medium text-gray-900">$24,599</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '75%' }}></div>
                </div>
              </div>
              <div className="space-y-2">
                <div className="flex justify-between">
                  <span className="text-gray-600">Efficiency Score</span>
                  <span className="font-medium text-gray-900">87%</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '87%' }}></div>
                </div>
              </div>
              <div className="space-y-2">
                <div className="flex justify-between">
                  <span className="text-gray-600">Power Usage</span>
                  <span className="font-medium text-gray-900">1.2 kW/h</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '60%' }}></div>
                </div>
              </div>
              <div className="space-y-2">
                <div className="flex justify-between">
                  <span className="text-gray-600">Speed Rating</span>
                  <span className="font-medium text-gray-900">95%</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '95%' }}></div>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">AI Suggestions</h2>
            <ul className="space-y-4">
              <li className="flex items-start space-x-3">
                <div className="flex-shrink-0">
                  <div className="w-6 h-6 bg-blue-100 rounded-full flex items-center justify-center">
                    <span className="text-blue-600">💡</span>
                  </div>
                </div>
                <span className="text-gray-700">Upgrade servo motors for 15% efficiency boost</span>
              </li>
              <li className="flex items-start space-x-3">
                <div className="flex-shrink-0">
                  <div className="w-6 h-6 bg-blue-100 rounded-full flex items-center justify-center">
                    <span className="text-blue-600">💡</span>
                  </div>
                </div>
                <span className="text-gray-700">Optimize arm configuration for reduced power consumption</span>
              </li>
              <li className="flex items-start space-x-3">
                <div className="flex-shrink-0">
                  <div className="w-6 h-6 bg-blue-100 rounded-full flex items-center justify-center">
                    <span className="text-blue-600">💡</span>
                  </div>
                </div>
                <span className="text-gray-700">Add precision sensors for improved accuracy</span>
              </li>
            </ul>
          </div>
        </div>

        {/* Right Column - Selected Components */}
        <div>
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-6">Selected Components</h2>
            
            <div className="space-y-6">
              <div>
                <div className="flex justify-between items-center mb-2">
                  <h3 className="font-medium text-gray-900">Base Unit</h3>
                  <span className="px-2 py-1 text-sm bg-gray-100 text-gray-600 rounded">Standard</span>
                </div>
                <div className="bg-gray-50 rounded-lg p-4 mb-2">
                  <div className="w-full h-32 bg-gray-200 rounded flex items-center justify-center">
                    <span className="text-3xl">📦</span>
                  </div>
                </div>
                <div className="grid grid-cols-2 gap-2 text-sm">
                  <div>
                    <span className="text-gray-600">Weight:</span>
                    <span className="ml-1 text-gray-900">75kg</span>
                  </div>
                  <div>
                    <span className="text-gray-600">Power:</span>
                    <span className="ml-1 text-gray-900">800W</span>
                  </div>
                </div>
              </div>

              <div>
                <div className="flex justify-between items-center mb-2">
                  <h3 className="font-medium text-gray-900">Arm Assembly</h3>
                  <span className="px-2 py-1 text-sm bg-gray-900 text-white rounded">Pro</span>
                </div>
                <div className="bg-gray-50 rounded-lg p-4 mb-2">
                  <div className="w-full h-32 bg-gray-200 rounded flex items-center justify-center">
                    <span className="text-3xl">🦾</span>
                  </div>
                </div>
                <div className="grid grid-cols-2 gap-2 text-sm">
                  <div>
                    <span className="text-gray-600">Reach:</span>
                    <span className="ml-1 text-gray-900">1.8m</span>
                  </div>
                  <div>
                    <span className="text-gray-600">Payload:</span>
                    <span className="ml-1 text-gray-900">25kg</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Navigation Buttons */}
      <div className="flex justify-end space-x-4 mt-8">
        <button
          onClick={() => router.push('/configure')}
          className="px-6 py-3 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
        >
          Back to Components
        </button>
        <button
          onClick={() => router.push('/quote')}
          className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
        >
          Get Quote
        </button>
      </div>
    </main>
  );
} 