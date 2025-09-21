'use client'

import React, { useState, useEffect, useRef } from 'react'
import { Loader2, Zap, AlertCircle, Settings, Camera, RotateCcw, Play, Pause } from 'lucide-react'
import { IsaacSimViewer } from './IsaacSimViewer'

interface IsaacSimDirectClientProps {
  spec?: any
  urdf?: string | URL
  environment?: 'warehouse' | 'factory' | 'lab' | 'outdoor'
  qualityProfile?: 'demo' | 'engineering' | 'certification'
  enablePhysics?: boolean
  userId?: string
  onJointControl?: (jointStates: Record<string, number>) => void
  onError?: (error: Error | string) => void
  className?: string
}

export function IsaacSimDirectClient({
  spec,
  urdf,
  environment = 'warehouse',
  qualityProfile = 'engineering',
  enablePhysics = true,
  userId = 'anonymous',
  onJointControl,
  onError,
  className = ''
}: IsaacSimDirectClientProps) {
  const [connectionState, setConnectionState] = useState<'checking' | 'connecting' | 'connected' | 'error'>('checking')
  const [sessionId, setSessionId] = useState<string | null>(null)
  const [metrics, setMetrics] = useState({ fps: 0, latency: 0, physics_fps: 0 })
  const [isSimulating, setIsSimulating] = useState(false)
  const [jointStates, setJointStates] = useState<Record<string, number>>({ joint1: 0, joint2: 0 })

  // Initialize Isaac Sim connection
  useEffect(() => {
    const initialize = async () => {
      try {
        setConnectionState('connecting')
        
        // Check if Isaac Sim service is available
        const healthResponse = await fetch('http://localhost:8002/health')
        
        if (!healthResponse.ok) {
          throw new Error('Isaac Sim service not available')
        }
        
        const healthData = await healthResponse.json()
        
        // Create session
        const sessionResponse = await fetch('http://localhost:8002/create_scene', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: userId,
            sepulka_id: spec?.id || 'demo-robot',
            environment,
            quality_profile: qualityProfile,
            urdf_content: typeof urdf === 'string' ? urdf : urdf?.toString()
          })
        })
        
        if (!sessionResponse.ok) {
          throw new Error('Failed to create Isaac Sim session')
        }
        
        const sessionData = await sessionResponse.json()
        setSessionId(sessionData.session_id)
        setConnectionState('connected')
        
        console.log('✅ Isaac Sim session created:', sessionData.session_id)
        
        
      } catch (error) {
        console.error('❌ Isaac Sim connection failed:', error)
        setConnectionState('error')
        onError?.(error as Error)
      }
    }

    initialize()
  }, [spec, environment, qualityProfile, userId, urdf, onError])

  // Error state
  if (connectionState === 'error') {
    return (
      <div className={`bg-gray-900 rounded-lg flex items-center justify-center ${className}`}>
        <div className="text-center text-white max-w-md mx-auto p-6">
          <AlertCircle className="w-16 h-16 text-red-500 mx-auto mb-4" />
          <div className="text-xl font-medium mb-2">Isaac Sim Service Offline</div>
          <div className="text-sm text-gray-300 mb-6">
            The Isaac Sim service is not responding. Please ensure it's running.
          </div>
          
          <div className="text-left bg-gray-800 rounded-lg p-4 mb-4">
            <div className="text-sm font-medium mb-2">Start Isaac Sim service:</div>
            <div className="font-mono text-xs bg-black rounded px-2 py-1">
              cd services/anvil-sim && ./scripts/dev-start.sh
            </div>
          </div>
          
          <button
            onClick={() => window.location.reload()}
            className="px-6 py-2 bg-blue-600 hover:bg-blue-700 rounded-md text-sm font-medium transition-colors"
          >
            Retry Connection
          </button>
        </div>
      </div>
    )
  }

  // Loading state
  if (connectionState === 'checking' || connectionState === 'connecting') {
    return (
      <div className={`bg-gray-900 rounded-lg flex items-center justify-center ${className}`}>
        <div className="text-center text-white">
          <div className="relative mb-4">
            <Zap className="w-12 h-12 text-yellow-500 mx-auto animate-pulse" />
            <Loader2 className="w-6 h-6 animate-spin absolute -top-1 -right-1" />
          </div>
          <div className="text-lg font-medium mb-2">
            {connectionState === 'checking' ? 'Checking Isaac Sim' : 'Connecting to Isaac Sim'}
          </div>
          <div className="text-sm text-gray-400">
            {connectionState === 'checking' ? 'Verifying service...' : 'Initializing physics simulation...'}
          </div>
        </div>
      </div>
    )
  }

  // Connected state - show enhanced Isaac Sim visualization
  return (
    <IsaacSimViewer
      spec={spec}
      urdf={urdf}
      environment={environment}
      qualityProfile={qualityProfile}
      enablePhysics={enablePhysics}
      sessionId={sessionId}
      onJointControl={(jointStates) => {
        setJointStates(jointStates)
        onJointControl?.(jointStates)
      }}
      className={className}
    />
  )

}
