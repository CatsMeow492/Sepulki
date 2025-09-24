'use client'

import dynamic from 'next/dynamic'

// Dynamically import the video component to avoid SSR issues
const IsaacSimVideoOnly = dynamic(
  () => import('@/components/IsaacSimVideoOnly'),
  { 
    ssr: false,
    loading: () => (
      <div className="flex items-center justify-center min-h-screen">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-500 mx-auto mb-4"></div>
          <div className="text-lg font-medium">Loading Isaac Sim Video...</div>
          <div className="text-sm text-gray-600 mt-2">Connecting to Isaac Sim container</div>
        </div>
      </div>
    )
  }
)

export default function IsaacVideoPage() {
  return (
    <div className="min-h-screen bg-gray-50">
      <div className="container mx-auto px-4 py-8">
        <div className="max-w-4xl mx-auto">
          <div className="text-center mb-8">
            <h1 className="text-3xl font-bold text-gray-900 mb-2">
              Isaac Sim Video Stream
            </h1>
            <p className="text-gray-600">
              Minimal implementation for testing Isaac Sim video streaming
            </p>
          </div>

          <IsaacSimVideoOnly className="bg-white rounded-lg shadow-lg p-6" />
          
          <div className="mt-8 text-center text-sm text-gray-500">
            <div>This page connects directly to Isaac Sim WebSocket at ws://localhost:8765</div>
            <div>Make sure Isaac Sim is running in the Brev container</div>
          </div>
        </div>
      </div>
    </div>
  )
}
