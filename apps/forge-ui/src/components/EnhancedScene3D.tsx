'use client'

import React, { useState, useEffect, useRef, useCallback } from 'react'
import { Monitor, Cpu, Zap, AlertCircle, Settings } from 'lucide-react'
import { Scene3D } from './Scene3D'
import { IsaacSimClient } from './IsaacSimClient'
import type { MotionProfile, RobotSpec } from '@/types/robot'

interface EnhancedScene3DProps {
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
  renderMode?: 'auto' | 'threejs' | 'isaac_sim'
  qualityProfile?: 'demo' | 'engineering' | 'certification'
  enablePhysics?: boolean
  environment?: 'warehouse' | 'factory' | 'lab' | 'outdoor'
  userId?: string
  enableCollaboration?: boolean
}

interface RenderingDecision {
  useIsaacSim: boolean
  reason: string
  confidence: number
}

interface DeviceCapabilities {
  gpu: 'high' | 'medium' | 'low' | 'none'
  bandwidth: number // Mbps
  isDesktop: boolean
  supportsWebRTC: boolean
}

export function EnhancedScene3D({
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
  renderMode = 'auto',
  qualityProfile = 'engineering',
  enablePhysics = true,
  environment = 'warehouse',
  userId = 'anonymous',
  enableCollaboration = false,
  ...props
}: EnhancedScene3DProps) {
  const [currentRenderer, setCurrentRenderer] = useState<'threejs' | 'isaac_sim' | 'loading'>('loading')
  const [renderingDecision, setRenderingDecision] = useState<RenderingDecision | null>(null)
  const [isaacSimSession, setIsaacSimSession] = useState<string | null>(null)
  const [isIsaacSimAvailable, setIsIsaacSimAvailable] = useState(false)
  const [deviceCapabilities, setDeviceCapabilities] = useState<DeviceCapabilities | null>(null)
  const [showRenderingInfo, setShowRenderingInfo] = useState(false)
  const [error, setError] = useState<string | null>(null)
  
  const robotApiRef = useRef<any>(null)
  const isaacSimClientRef = useRef<any>(null)

  // Detect device capabilities
  const detectDeviceCapabilities = useCallback(async (): Promise<DeviceCapabilities> => {
    // Detect GPU capabilities
    const canvas = document.createElement('canvas')
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl')
    let gpu: DeviceCapabilities['gpu'] = 'none'
    
    if (gl) {
      const debugInfo = gl.getExtension('WEBGL_debug_renderer_info')
      const renderer = debugInfo ? gl.getParameter(debugInfo.UNMASKED_RENDERER_WEBGL) : ''
      
      if (renderer.toLowerCase().includes('nvidia') || renderer.toLowerCase().includes('amd')) {
        gpu = 'high'
      } else if (renderer.toLowerCase().includes('intel')) {
        gpu = 'medium'
      } else {
        gpu = 'low'
      }
    }

    // Estimate bandwidth (simplified)
    const connection = (navigator as any).connection || (navigator as any).mozConnection || (navigator as any).webkitConnection
    const bandwidth = connection?.downlink || 10 // Default to 10 Mbps

    // Detect device type
    const isDesktop = window.innerWidth > 1024 && !/Mobi|Android/i.test(navigator.userAgent)

    // Check WebRTC support
    const supportsWebRTC = !!(window.RTCPeerConnection || (window as any).webkitRTCPeerConnection || (window as any).mozRTCPeerConnection)

    return {
      gpu,
      bandwidth,
      isDesktop,
      supportsWebRTC
    }
  }, [])

  // Calculate robot complexity score
  const calculateComplexity = useCallback((spec?: RobotSpec): number => {
    if (!spec) return 0.3 // Simple default
    
    let complexity = 0
    
    // Base complexity from alloys
    if (spec.alloys) {
      complexity += spec.alloys.length * 0.1
    }
    
    // Add complexity for physics simulation needs
    if (spec.parameters?.payload && spec.parameters.payload > 10) {
      complexity += 0.2 // Heavy payload needs better physics
    }
    
    if (spec.parameters?.reach && spec.parameters.reach > 2) {
      complexity += 0.2 // Long reach needs better visualization
    }
    
    if (spec.parameters?.precision && spec.parameters.precision < 1) {
      complexity += 0.3 // High precision needs better simulation
    }
    
    return Math.min(complexity, 1.0) // Cap at 1.0
  }, [])

  // Intelligent rendering decision engine
  const makeRenderingDecision = useCallback((
    capabilities: DeviceCapabilities,
    complexity: number,
    renderMode: string
  ): RenderingDecision => {
    
    // Force specific renderer
    if (renderMode === 'threejs') {
      return {
        useIsaacSim: false,
        reason: 'User forced Three.js rendering',
        confidence: 1.0
      }
    }
    
    if (renderMode === 'isaac_sim') {
      if (capabilities.supportsWebRTC && capabilities.bandwidth > 5) {
        return {
          useIsaacSim: true,
          reason: 'User forced Isaac Sim rendering',
          confidence: 1.0
        }
      } else {
        return {
          useIsaacSim: false,
          reason: 'Isaac Sim forced but requirements not met, falling back to Three.js',
          confidence: 0.8
        }
      }
    }
    
    // Auto-decision logic
    let score = 0
    let reasons: string[] = []
    
    // Device capability factors
    if (capabilities.supportsWebRTC) {
      score += 0.3
    } else {
      reasons.push('No WebRTC support')
      return {
        useIsaacSim: false,
        reason: reasons.join(', '),
        confidence: 0.9
      }
    }
    
    if (capabilities.bandwidth >= 10) {
      score += 0.3
      reasons.push('High bandwidth')
    } else if (capabilities.bandwidth >= 5) {
      score += 0.1
      reasons.push('Medium bandwidth')
    } else {
      reasons.push('Low bandwidth')
    }
    
    if (capabilities.isDesktop) {
      score += 0.2
      reasons.push('Desktop device')
    }
    
    if (capabilities.gpu === 'high') {
      score += 0.2
    } else if (capabilities.gpu === 'medium') {
      score += 0.1
    }
    
    // Physics simulation needs
    if (enablePhysics && complexity > 0.5) {
      score += 0.3
      reasons.push('Complex physics simulation needed')
    }
    
    // Collaboration features
    if (enableCollaboration) {
      score += 0.2
      reasons.push('Collaboration features needed')
    }
    
    // Professional quality requirements
    if (qualityProfile === 'certification') {
      score += 0.4
      reasons.push('Certification quality required')
    } else if (qualityProfile === 'engineering') {
      score += 0.2
      reasons.push('Engineering quality preferred')
    }
    
    const useIsaacSim = score >= 0.6
    const confidence = Math.min(Math.abs(score - 0.6) + 0.6, 1.0)
    
    return {
      useIsaacSim,
      reason: useIsaacSim 
        ? `Isaac Sim selected: ${reasons.join(', ')}` 
        : `Three.js selected: insufficient score (${score.toFixed(2)}/1.0)`,
      confidence
    }
  }, [enablePhysics, enableCollaboration, qualityProfile])

  // Check Isaac Sim availability
  const checkIsaacSimAvailability = useCallback(async (): Promise<boolean> => {
    try {
      const response = await fetch('http://localhost:8002/health', {
        method: 'GET',
        timeout: 3000
      } as any)
      return response.ok
    } catch (error) {
      console.log('Isaac Sim service not available:', error)
      return false
    }
  }, [])

  // Create Isaac Sim session
  const createIsaacSimSession = useCallback(async (): Promise<string | null> => {
    try {
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
      
      if (!response.ok) throw new Error('Failed to create Isaac Sim session')
      
      const data = await response.json()
      return data.session_id
    } catch (error) {
      console.error('Failed to create Isaac Sim session:', error)
      return null
    }
  }, [userId, spec?.id, environment, qualityProfile, urdf])

  // Initialize rendering system
  useEffect(() => {
    let isMounted = true

    const initialize = async () => {
      try {
        // Detect device capabilities
        const capabilities = await detectDeviceCapabilities()
        if (!isMounted) return
        setDeviceCapabilities(capabilities)

        // Calculate robot complexity
        const complexity = calculateComplexity(spec)

        // Check Isaac Sim availability
        const isaacSimAvailable = await checkIsaacSimAvailability()
        if (!isMounted) return
        setIsIsaacSimAvailable(isaacSimAvailable)

        // Make rendering decision
        const decision = makeRenderingDecision(capabilities, complexity, renderMode)
        setRenderingDecision(decision)

        // Apply decision
        if (decision.useIsaacSim && isaacSimAvailable) {
          // Try to create Isaac Sim session
          const sessionId = await createIsaacSimSession()
          if (!isMounted) return
          
          if (sessionId) {
            setIsaacSimSession(sessionId)
            setCurrentRenderer('isaac_sim')
          } else {
            // Fallback to Three.js
            setCurrentRenderer('threejs')
            setError('Isaac Sim session creation failed, using Three.js fallback')
          }
        } else {
          setCurrentRenderer('threejs')
          if (decision.useIsaacSim && !isaacSimAvailable) {
            setError('Isaac Sim preferred but service unavailable, using Three.js fallback')
          }
        }
      } catch (error) {
        if (!isMounted) return
        setError(`Initialization failed: ${error}`)
        setCurrentRenderer('threejs')
      }
    }

    initialize()

    return () => {
      isMounted = false
    }
  }, [
    spec, renderMode, qualityProfile, environment, userId,
    detectDeviceCapabilities, calculateComplexity, checkIsaacSimAvailability,
    makeRenderingDecision, createIsaacSimSession
  ])

  // Handle robot API from either renderer
  const handleRobotApi = useCallback((api: any) => {
    robotApiRef.current = api
    onRobotApi?.(api)
  }, [onRobotApi])

  // Handle joint control (forward to appropriate renderer)
  const handleJointControl = useCallback((jointStates: Record<string, number>) => {
    if (currentRenderer === 'isaac_sim' && isaacSimClientRef.current) {
      isaacSimClientRef.current.updateJointStates?.(jointStates)
    }
    // For Three.js, joint control is handled directly
  }, [currentRenderer])

  // Handle errors from either renderer
  const handleError = useCallback((error: Error | string) => {
    console.error('Renderer error:', error)
    setError(typeof error === 'string' ? error : error.message)
    onError?.(error)
  }, [onError])

  // Renderer selection UI
  const renderModeSelector = (
    <div className="absolute top-4 left-4 z-10">
      <div className={`bg-black/70 backdrop-blur-sm rounded-lg p-3 transition-opacity ${showRenderingInfo ? 'opacity-100' : 'opacity-0 hover:opacity-100'}`}>
        <div className="flex items-center space-x-2 text-white text-sm mb-2">
          {currentRenderer === 'isaac_sim' ? (
            <Zap className="w-4 h-4 text-yellow-500" />
          ) : (
            <Monitor className="w-4 h-4 text-blue-500" />
          )}
          <span className="font-medium">
            {currentRenderer === 'isaac_sim' ? 'Isaac Sim' : 'Three.js'}
          </span>
          {currentRenderer === 'isaac_sim' && (
            <span className="px-2 py-1 bg-yellow-600 rounded text-xs">PHYSICS</span>
          )}
        </div>
        
        {renderingDecision && (
          <div className="text-xs text-gray-300 mb-2">
            {renderingDecision.reason}
          </div>
        )}
        
        {deviceCapabilities && (
          <div className="text-xs text-gray-400 space-y-1">
            <div>GPU: {deviceCapabilities.gpu}</div>
            <div>Bandwidth: {deviceCapabilities.bandwidth.toFixed(1)} Mbps</div>
            <div>WebRTC: {deviceCapabilities.supportsWebRTC ? '✓' : '✗'}</div>
          </div>
        )}
        
        <button
          onClick={() => setShowRenderingInfo(!showRenderingInfo)}
          className="mt-2 p-1 bg-gray-700 hover:bg-gray-600 rounded text-white transition-colors"
          title="Rendering info"
        >
          <Settings className="w-3 h-3" />
        </button>
      </div>
    </div>
  )

  // Error display
  if (error && currentRenderer === 'loading') {
    return (
      <div className="w-full h-96 bg-gray-900 rounded-lg flex items-center justify-center relative">
        <div className="text-center text-white">
          <AlertCircle className="w-12 h-12 text-red-500 mx-auto mb-4" />
          <div className="text-lg font-medium mb-2">Rendering Error</div>
          <div className="text-sm text-gray-400 max-w-md">{error}</div>
        </div>
        {renderModeSelector}
      </div>
    )
  }

  // Loading state
  if (currentRenderer === 'loading') {
    return (
      <div className="w-full h-96 bg-gray-900 rounded-lg flex items-center justify-center relative">
        <div className="text-center text-white">
          <Cpu className="w-12 h-12 animate-pulse mx-auto mb-4" />
          <div className="text-lg font-medium mb-2">Initializing Renderer</div>
          <div className="text-sm text-gray-400">Analyzing requirements...</div>
        </div>
      </div>
    )
  }

  return (
    <div className="relative w-full h-full">
      {/* Render based on decision */}
      {currentRenderer === 'isaac_sim' && isaacSimSession ? (
        <IsaacSimClient
          ref={isaacSimClientRef}
          sessionId={isaacSimSession}
          userId={userId}
          qualityProfile={qualityProfile}
          onJointControl={handleJointControl}
          onError={handleError}
          className="w-full h-full"
        />
      ) : (
        <Scene3D
          spec={spec}
          urdf={urdf}
          assetBaseUrl={assetBaseUrl}
          showAxes={showAxes}
          profile={profile}
          onRobotApi={handleRobotApi}
          onError={handleError}
          playing={playing}
          playbackRate={playbackRate}
          seekTime={seekTime}
          heroUrl={heroUrl}
          {...props}
        />
      )}

      {/* Renderer info overlay */}
      {renderModeSelector}

      {/* Error notification */}
      {error && (
        <div className="absolute bottom-4 right-4 max-w-sm">
          <div className="bg-orange-600/90 backdrop-blur-sm text-white p-3 rounded-lg text-sm">
            <div className="font-medium mb-1">Notice</div>
            {error}
          </div>
        </div>
      )}
    </div>
  )
}
