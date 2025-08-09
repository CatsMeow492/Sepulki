'use client';

import { useEffect, useState } from 'react';
import { useRouter } from 'next/navigation';
import dynamic from 'next/dynamic';

const Scene3D = dynamic(
  () => import('@/components/Scene3D').then((mod) => mod.Scene3D),
  { ssr: false }
);

export default function QuotePage() {
  const router = useRouter();
  const [builtUrdf, setBuiltUrdf] = useState<string | undefined>(undefined)

  useEffect(() => {
    // Auto-load the same sample model for the quote preview (MVP)
    const sampleUrl = '/robots/sample-arm-01/urdf/sample.urdf'
    setBuiltUrdf(sampleUrl)
  }, [])

  return (
    <main className="max-w-7xl mx-auto px-4 py-8">
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
        {/* Left Column - 3D Preview and Specifications */}
        <div className="space-y-8">
          {/* 3D Preview */}
          <div className="bg-gray-100 rounded-lg p-4 aspect-square">
            <Scene3D urdf={builtUrdf} assetBaseUrl="/robots/sample-arm-01" />
          </div>

          {/* Specifications Summary */}
          <div>
            <h2 className="text-xl font-semibold text-gray-900 mb-6">Specifications Summary</h2>
            <div className="grid grid-cols-2 gap-6">
              <div className="bg-white rounded-lg p-4 shadow-sm">
                <div className="text-gray-600">Weight Capacity</div>
                <div className="text-2xl font-semibold text-gray-900">250 kg</div>
              </div>
              <div className="bg-white rounded-lg p-4 shadow-sm">
                <div className="text-gray-600">Maximum Reach</div>
                <div className="text-2xl font-semibold text-gray-900">2.5 m</div>
              </div>
              <div className="bg-white rounded-lg p-4 shadow-sm">
                <div className="text-gray-600">Precision</div>
                <div className="text-2xl font-semibold text-gray-900">±0.1 mm</div>
              </div>
              <div className="bg-white rounded-lg p-4 shadow-sm">
                <div className="text-gray-600">Speed</div>
                <div className="text-2xl font-semibold text-gray-900">2 m/s</div>
              </div>
            </div>
          </div>

          {/* Performance Metrics */}
          <div>
            <h2 className="text-xl font-semibold text-gray-900 mb-6">Performance Metrics</h2>
            <div className="space-y-4">
              <div>
                <div className="flex justify-between mb-2">
                  <span className="text-gray-600">Efficiency</span>
                  <span className="text-gray-900">85%</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '85%' }}></div>
                </div>
              </div>
              <div>
                <div className="flex justify-between mb-2">
                  <span className="text-gray-600">Reliability</span>
                  <span className="text-gray-900">92%</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '92%' }}></div>
                </div>
              </div>
              <div>
                <div className="flex justify-between mb-2">
                  <span className="text-gray-600">Uptime</span>
                  <span className="text-gray-900">98%</span>
                </div>
                <div className="w-full bg-gray-100 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{ width: '98%' }}></div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Right Column - ROI Calculator and Subscription */}
        <div className="space-y-8">
          {/* ROI Calculator */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-semibold text-gray-900 mb-6">ROI Calculator</h2>
            <div className="space-y-6">
              <div>
                <div className="text-gray-600 mb-1">Initial Investment</div>
                <div className="text-3xl font-semibold text-gray-900">$125,000</div>
              </div>
              <div>
                <div className="text-gray-600 mb-1">Annual Savings</div>
                <div className="text-3xl font-semibold text-gray-900">$48,000</div>
              </div>
              <div>
                <div className="text-gray-600 mb-1">Payback Period</div>
                <div className="text-3xl font-semibold text-gray-900">2.6 years</div>
              </div>
              <div>
                <div className="text-gray-600 mb-1">5-Year ROI</div>
                <div className="text-3xl font-semibold text-gray-900">192%</div>
              </div>
            </div>
          </div>

          {/* Monthly Subscription */}
          <div className="bg-white rounded-lg shadow-sm p-6">
            <h2 className="text-xl font-semibold text-gray-900 mb-6">Monthly Subscription</h2>
            <div className="space-y-4">
              <div className="flex justify-between items-center py-2">
                <span className="text-gray-600">Hardware Lease</span>
                <span className="text-gray-900 font-medium">$2,500</span>
              </div>
              <div className="flex justify-between items-center py-2">
                <span className="text-gray-600">Maintenance</span>
                <span className="text-gray-900 font-medium">$800</span>
              </div>
              <div className="flex justify-between items-center py-2">
                <span className="text-gray-600">Software License</span>
                <span className="text-gray-900 font-medium">$400</span>
              </div>
              <div className="border-t pt-4 mt-4">
                <div className="flex justify-between items-center">
                  <span className="text-gray-900 font-semibold">Total Monthly</span>
                  <span className="text-gray-900 font-semibold">$3,700</span>
                </div>
              </div>
            </div>
          </div>

          {/* Action Buttons */}
          <div className="space-y-4">
            <button className="w-full bg-gray-900 text-white py-3 px-4 rounded-lg hover:bg-gray-800 transition-colors">
              Request Quote
            </button>
            <button className="w-full bg-white border border-gray-300 text-gray-700 py-3 px-4 rounded-lg hover:bg-gray-50 transition-colors">
              Schedule Demo
            </button>
          </div>
        </div>
      </div>
    </main>
  );
} 