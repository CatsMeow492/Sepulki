'use client'

import React, { useState, useEffect, useRef, useCallback } from 'react'
import { Loader2, Zap, AlertCircle, Wifi, WifiOff } from 'lucide-react'
import { IsaacSimClient } from './IsaacSimClient'
import type { MotionProfile, RobotSpec } from '@/types/robot'

interface IsaacSimScene3DProps {
  spec?: RobotSpec
  urdf?: string | URL
  assetBaseUrl?: string
  showAxes?: boolean
  profile?: MotionProfile
  onRobotApi?: (api: {
    getJoint: (name: string) => any
    setJointValue: (name: string, value: number) => void
    getJointValue: (name: string) => number
    listJoints: () => { name: string; limit?: { lower: number; upper: number } }[]
  }) => void
  onError?: (error: Error | string) => void
  playing?: boolean
  playbackRate?: number
  seekTime?: number
  heroUrl?: string
  
  // Isaac Sim specific props
  qualityProfile?: 'demo' | 'engineering' | 'certification'
  enablePhysics?: boolean
  environment?: 'warehouse' | 'factory' | 'lab' | 'outdoor'
  userId?: string
  enableCollaboration?: boolean
}

export function IsaacSimScene3D({
  spec,
  urdf,
  assetBaseUrl,
  showAxes = false,
  profile,
  onRobotApi,
  onError,
  playing = false,
  playbackRate = 1,
  seekTime,
  heroUrl,
  qualityProfile = 'engineering',
  enablePhysics = true,
  environment = 'warehouse',
  userId = 'anonymous',
  enableCollaboration = false,
  ...props
}: IsaacSimScene3DProps) {
  const [connectionState, setConnectionState] = useState<'checking' | 'connecting' | 'connected' | 'error'>('checking')
  const [sessionId, setSessionId] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [isaacSimAvailable, setIsaacSimAvailable] = useState(false)
  
  const robotApiRef = useRef<any>(null)

  // Check Isaac Sim service availability
  const checkServiceAvailability = useCallback(async (): Promise<boolean> => {
    try {
      const response = await fetch('http://localhost:8002/health', {
        method: 'GET',
        timeout: 5000
      } as any)
      
      if (!response.ok) return false
      
      const data = await response.json()
      return data.status === 'healthy'
    } catch (error) {
      console.error('Isaac Sim service check failed:', error)
      return false
    }
  }, [])

  // Create Isaac Sim session
  const createSession = useCallback(async (): Promise<string | null> => {
    try {
      setConnectionState('connecting')
      
      const response = await fetch('http://localhost:8002/create_scene', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: userId,
          sepulka_id: spec?.id || 'demo-robot',
          environment,
          quality_profile: qualityProfile,
          urdf_content: typeof urdf === 'string' ? urdf : urdf?.toString()
        })
      })
      
      if (!response.ok) {
        throw new Error(`Session creation failed: ${response.status}`)
      }
      
      const data = await response.json()
      
      if (!data.success) {
        throw new Error(data.message || 'Session creation failed')
      }
      
      console.log('✅ Isaac Sim session created:', data.session_id)
      return data.session_id
      
    } catch (error) {
      console.error('❌ Failed to create Isaac Sim session:', error)
      setError(`Session creation failed: ${error}`)
      setConnectionState('error')
      onError?.(error as Error)
      return null
    }
  }, [userId, spec?.id, environment, qualityProfile, urdf, onError])

  // Initialize Isaac Sim connection
  useEffect(() => {
    let isMounted = true

    const initialize = async () => {
      try {
        setConnectionState('checking')
        setError(null)
        
        // Check if Isaac Sim service is available
        const available = await checkServiceAvailability()
        
        if (!isMounted) return
        
        setIsaacSimAvailable(available)
        
        if (!available) {
          setError('Isaac Sim service is not available. Please ensure the Anvil Sim service is running.')
          setConnectionState('error')
          return
        }
        
        // Create session
        const newSessionId = await createSession()
        
        if (!isMounted) return
        
        if (newSessionId) {
          setSessionId(newSessionId)
          setConnectionState('connected')
        } else {
          setConnectionState('error')
        }
        
      } catch (error) {
        if (!isMounted) return
        console.error('Isaac Sim initialization failed:', error)
        setError(`Initialization failed: ${error}`)
        setConnectionState('error')
      }
    }

    initialize()

    return () => {
      isMounted = false
    }
  }, [spec, environment, qualityProfile, userId, checkServiceAvailability, createSession])

  // Handle robot API from Isaac Sim client
  const handleRobotApi = useCallback((api: any) => {
    robotApiRef.current = api
    onRobotApi?.(api)
  }, [onRobotApi])

  // Handle connection state changes
  const handleConnectionChange = useCallback((connected: boolean) => {
    if (connected && connectionState !== 'connected') {
      setConnectionState('connected')
    } else if (!connected && connectionState === 'connected') {
      setConnectionState('error')
      setError('Connection lost')
    }
  }, [connectionState])

  // Handle errors from Isaac Sim client
  const handleError = useCallback((error: Error | string) => {
    console.error('Isaac Sim client error:', error)
    setError(typeof error === 'string' ? error : error.message)
    setConnectionState('error')
    onError?.(error)
  }, [onError])

  // Render loading state
  if (connectionState === 'checking') {
    return (
      <div className="w-full h-full bg-gray-900 rounded-lg flex items-center justify-center relative">
        <div className="text-center text-white">
          <Loader2 className="w-12 h-12 animate-spin mx-auto mb-4" />
          <div className="text-lg font-medium mb-2">Checking Isaac Sim Service</div>
          <div className="text-sm text-gray-400">Verifying connection...</div>
        </div>
      </div>
    )
  }

  // Render error state
  if (connectionState === 'error') {
    return (
      <div className="w-full h-full bg-gray-900 rounded-lg flex items-center justify-center relative">
        <div className="text-center text-white max-w-md mx-auto p-6">
          <AlertCircle className="w-16 h-16 text-red-500 mx-auto mb-4" />
          <div className="text-xl font-medium mb-2">Isaac Sim Unavailable</div>
          <div className="text-sm text-gray-300 mb-6">{error}</div>
          
          <div className="text-left bg-gray-800 rounded-lg p-4 mb-4">
            <div className="text-sm font-medium mb-2">To start Isaac Sim service:</div>
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

  // Render connecting state
  if (connectionState === 'connecting') {
    return (
      <div className="w-full h-full bg-gray-900 rounded-lg flex items-center justify-center relative">
        <div className="text-center text-white">
          <div className="relative mb-4">
            <Zap className="w-12 h-12 text-yellow-500 mx-auto animate-pulse" />
            <div className="absolute -top-1 -right-1">
              <Loader2 className="w-6 h-6 animate-spin" />
            </div>
          </div>
          <div className="text-lg font-medium mb-2">Connecting to Isaac Sim</div>
          <div className="text-sm text-gray-400">Initializing physics simulation...</div>
        </div>
      </div>
    )
  }

  // Render Isaac Sim client
  if (connectionState === 'connected' && sessionId) {
    return (
      <div className="relative w-full h-full">
        <IsaacSimClient
          sessionId={sessionId}
          userId={userId}
          qualityProfile={qualityProfile}
          onJointControl={(jointStates) => {
            // Forward joint control to robot API if available
            if (robotApiRef.current) {
              Object.entries(jointStates).forEach(([jointName, value]) => {
                robotApiRef.current.setJointValue(jointName, value)
              })
            }
          }}
          onCameraControl={(camera) => {
            console.log('Camera control:', camera)
          }}
          onError={handleError}
          onConnectionChange={handleConnectionChange}
          className="w-full h-full"
        />
        
        {/* Isaac Sim Status Indicator */}
        <div className="absolute top-4 left-4">
          <div className="bg-black/70 backdrop-blur-sm rounded-lg p-3">
            <div className="flex items-center space-x-2">
              <div className="flex items-center space-x-1">
                <Zap className="w-4 h-4 text-yellow-500" />
                <span className="text-white font-medium">Isaac Sim</span>
                <span className="px-2 py-1 bg-yellow-600 rounded text-xs text-white font-medium">
                  PHYSICS
                </span>
              </div>
            </div>
            <div className="mt-2 text-xs text-gray-300">
              Environment: {environment} • Quality: {qualityProfile}
            </div>
            {enablePhysics && (
              <div className="flex items-center mt-1">
                <div className="w-2 h-2 bg-green-500 rounded-full mr-2"></div>
                <span className="text-xs text-gray-300">Real-time physics simulation</span>
              </div>
            )}
          </div>
        </div>
      </div>
    )
  }

  // Fallback - should not reach here
  return (
    <div className="w-full h-full bg-gray-900 rounded-lg flex items-center justify-center">
      <div className="text-center text-white">
        <AlertCircle className="w-12 h-12 text-red-500 mx-auto mb-4" />
        <div className="text-lg font-medium">Unknown State</div>
        <div className="text-sm text-gray-400">Please reload the page</div>
      </div>
    </div>
  )
}
