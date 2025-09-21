'use client'

import { useState, useEffect } from 'react'
import dynamic from 'next/dynamic'
import { IsaacSimControls } from '@/components/IsaacSimControls'
import { SimulationMetrics } from '@/components/SimulationMetrics'
import { useIsaacSimConnection } from '@/hooks/useIsaacSimConnection'
import type { MotionProfile, RobotSpec } from '@/types/robot'

// Dynamically import the EnhancedScene3D component
const EnhancedScene3D = dynamic(
  () => import('@/components/EnhancedScene3D').then((mod) => mod.EnhancedScene3D),
  { ssr: false }
)

interface Enhanced3DSectionProps {
  builtSpec?: RobotSpec
  builtUrdf?: string
  playing: boolean
  rate: number
  seek: number
  onRobotApi?: (api: any) => void
  smith?: { email: string } | null
}

export function Enhanced3DSection({
  builtSpec,
  builtUrdf,
  playing,
  rate,
  seek,
  onRobotApi,
  smith
}: Enhanced3DSectionProps) {
  const { connectionStatus, connectToIsaacSim, updateMetrics } = useIsaacSimConnection()
  const [sessionId, setSessionId] = useState<string | null>(null)
  const [physicsEnabled, setPhysicsEnabled] = useState(true)
  const [collaborationEnabled, setCollaborationEnabled] = useState(false)
  const [quality, setQuality] = useState<'demo' | 'engineering' | 'certification'>('engineering')

  // Generate session ID when robot spec changes
  useEffect(() => {
    if (builtSpec) {
      const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
      setSessionId(newSessionId)
      connectToIsaacSim(newSessionId)
    }
  }, [builtSpec, connectToIsaacSim])

  const handleQualityChange = (newQuality: string) => {
    setQuality(newQuality as 'demo' | 'engineering' | 'certification')
  }

  const handleTogglePhysics = () => {
    setPhysicsEnabled(!physicsEnabled)
  }

  const handleResetCamera = () => {
    // This would be handled by the EnhancedScene3D component
    console.log('Reset camera view')
  }

  const handleToggleCollaboration = () => {
    setCollaborationEnabled(!collaborationEnabled)
  }

  return (
    <div className="space-y-4">
      {/* Main 3D Viewer */}
      <div className="aspect-square w-full bg-gray-700 rounded-lg overflow-hidden relative">
        <EnhancedScene3D
          spec={builtSpec}
          urdf={builtUrdf}
          assetBaseUrl="/robots/sample-arm-01"
          profile={builtSpec ? {
            name: 'demo',
            duration: 4,
            loop: true,
            frames: [
              { t: 0, pose: { joint1: 0, joint2: 0 } },
              { t: 2, pose: { joint1: 1.0, joint2: 0.6 } },
              { t: 4, pose: { joint1: 0, joint2: 0 } },
            ]
          } : undefined}
          playing={playing}
          playbackRate={rate}
          seekTime={seek}
          onRobotApi={onRobotApi}
          renderMode="auto"
          qualityProfile={quality}
          enablePhysics={physicsEnabled}
          environment="warehouse"
          userId={smith?.email || 'anonymous'}
          enableCollaboration={collaborationEnabled}
          onError={(error) => {
            console.error('3D Rendering error:', error)
          }}
        />
      </div>

      {/* Controls Row */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
        {/* Isaac Sim Controls */}
        <IsaacSimControls
          isConnected={connectionStatus.status === 'connected'}
          service={connectionStatus.service}
          quality={quality}
          onQualityChange={handleQualityChange}
          onTogglePhysics={handleTogglePhysics}
          onResetCamera={handleResetCamera}
          onToggleCollaboration={handleToggleCollaboration}
          physicsEnabled={physicsEnabled}
          collaborationEnabled={collaborationEnabled}
        />

        {/* Performance Metrics */}
        <SimulationMetrics
          service={connectionStatus.service}
          isConnected={connectionStatus.status === 'connected'}
          sessionId={sessionId || undefined}
        />
      </div>

      {/* Feature Highlights */}
      <div className="bg-gradient-to-r from-blue-50 to-indigo-50 rounded-lg p-4">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 text-sm">
          <div className="flex items-center space-x-2">
            <div className="w-2 h-2 bg-green-500 rounded-full"></div>
            <span className="text-gray-700">
              {connectionStatus.service === 'isaac_sim' ? 
                'Real-time Physics Simulation' : 
                'High-Performance 3D Rendering'}
            </span>
          </div>
          
          <div className="flex items-center space-x-2">
            <div className="w-2 h-2 bg-blue-500 rounded-full"></div>
            <span className="text-gray-700">
              {connectionStatus.service === 'isaac_sim' ? 
                'Photorealistic Materials' : 
                'Interactive Joint Controls'}
            </span>
          </div>
          
          <div className="flex items-center space-x-2">
            <div className="w-2 h-2 bg-purple-500 rounded-full"></div>
            <span className="text-gray-700">
              {connectionStatus.service === 'isaac_sim' ? 
                'WebRTC Streaming' : 
                'Lightweight & Fast'}
            </span>
          </div>
        </div>
      </div>
    </div>
  )
}
