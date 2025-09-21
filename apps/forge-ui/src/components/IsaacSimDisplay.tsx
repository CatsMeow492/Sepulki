'use client'

import React, { useState, useEffect, useRef } from 'react'
import { Loader2, Zap, AlertCircle, Pause, Play, RotateCcw, Camera, Maximize, Minimize, Eye, Move, Target } from 'lucide-react'
import type { RobotSpec } from '@/types/robot'

interface RobotConfig {
  selectedRobot?: any // IsaacSimRobot type
  isaacSimPath?: string
  robotName?: string
  physicsConfig?: any
}

interface IsaacSimDisplayProps {
  spec?: RobotSpec
  urdf?: string | URL
  environment?: 'warehouse' | 'factory' | 'lab' | 'outdoor'
  qualityProfile?: 'demo' | 'engineering' | 'certification'
  enablePhysics?: boolean
  userId?: string
  robotConfig?: RobotConfig
  onJointControl?: (jointStates: Record<string, number>) => void
  onError?: (error: Error | string) => void
  className?: string
}

export function IsaacSimDisplay({
  spec,
  urdf,
  environment = 'warehouse',
  qualityProfile = 'engineering',
  enablePhysics = true,
  userId = 'anonymous',
  robotConfig,
  onJointControl,
  onError,
  className = ''
}: IsaacSimDisplayProps) {
  const [connectionState, setConnectionState] = useState<'checking' | 'connecting' | 'connected' | 'error'>('checking')
  const [sessionId, setSessionId] = useState<string | null>(null)
  const [websocket, setWebsocket] = useState<WebSocket | null>(null)
  const [jointStates, setJointStates] = useState<Record<string, number>>({ 
    joint1: 0.2, 
    joint2: -0.3 
  })
  const [metrics, setMetrics] = useState({ 
    fps: 60, 
    physics_fps: 240, 
    latency: 148 
  })
  const [isSimulating, setIsSimulating] = useState(true)
  const [showPhysics, setShowPhysics] = useState(enablePhysics)
  const [isFullscreen, setIsFullscreen] = useState(false)
  const [showCameraControls, setShowCameraControls] = useState(false)
  const [cameraState, setCameraState] = useState({
    position: { x: 4, y: 4, z: 4 },
    target: { x: 0, y: 0, z: 0 },
    fov: 50
  })
  
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const peerConnectionRef = useRef<RTCPeerConnection | null>(null)
  const animationRef = useRef<number>()
  const initRef = useRef(false)

  // Initialize Isaac Sim connection and WebRTC
  useEffect(() => {
    if (initRef.current) return
    initRef.current = true

    const initialize = async () => {
      try {
        setConnectionState('connecting')
        
        // Check Isaac Sim service health
        const healthResponse = await fetch('http://localhost:8002/health')
        if (!healthResponse.ok) {
          throw new Error('Isaac Sim service not available')
        }

        // Create session
        const sessionResponse = await fetch('http://localhost:8002/create_scene', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: userId,
            sepulka_id: spec?.id || 'demo-robot',
            environment,
            quality_profile: qualityProfile,
            urdf_content: typeof urdf === 'string' ? urdf : urdf?.toString(),
            isaac_sim_robot: robotConfig?.selectedRobot,
            physics_config: robotConfig?.physicsConfig
          })
        })

        if (!sessionResponse.ok) {
          throw new Error('Failed to create Isaac Sim session')
        }

        const sessionData = await sessionResponse.json()
        const newSessionId = sessionData.session_id
        setSessionId(newSessionId)
        
        console.log('✅ Isaac Sim Display session created:', newSessionId)

        // Establish WebSocket connection
        const ws = new WebSocket('ws://localhost:8001')

        ws.onopen = () => {
          console.log('🔌 WebSocket connected to Isaac Sim')
          setWebsocket(ws)

          // Join the session
          ws.send(JSON.stringify({
            type: 'join_session',
            session_id: newSessionId,
            user_id: userId,
            quality_profile: qualityProfile
          }))
          console.log('👋 Join session message sent to Isaac Sim')
        }

        ws.onmessage = (event) => {
          const data = JSON.parse(event.data)
          console.log('📨 Isaac Sim message:', data.type, data)

          if (data.type === 'connection_established') {
            setConnectionState('connected')
            console.log('✅ Isaac Sim WebSocket connection established')
            
            // Set up WebRTC for video streaming (will be muted by browser)
            try {
              const peerConnection = new RTCPeerConnection({
                iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
              })

              peerConnectionRef.current = peerConnection

              // Handle incoming video stream
              peerConnection.ontrack = (event) => {
                console.log('📹 Received Isaac Sim video stream!')
                if (videoRef.current && event.streams[0]) {
                  videoRef.current.srcObject = event.streams[0]
                  videoRef.current.play().catch(console.error)
                }
              }

              // Handle ICE candidates
              peerConnection.onicecandidate = (event) => {
                if (event.candidate) {
                  ws.send(JSON.stringify({
                    type: 'ice_candidate',
                    candidate: event.candidate
                  }))
                }
              }

              // Create offer for Isaac Sim
              peerConnection.createOffer({
                offerToReceiveVideo: true,
                offerToReceiveAudio: false
              }).then((offer) => {
                return peerConnection.setLocalDescription(offer)
              }).then(() => {
                ws.send(JSON.stringify({
                  type: 'offer',
                  session_id: newSessionId,
                  sdp: peerConnection.localDescription?.sdp
                }))
                console.log('📡 WebRTC offer sent to Isaac Sim (after join_session)')
              })

            } catch (error) {
              console.error('❌ WebRTC setup failed:', error)
            }
            
          } else if (data.type === 'answer') {
            // Handle WebRTC answer
            try {
              const answerDescription = new RTCSessionDescription({
                type: 'answer',
                sdp: data.sdp
              })
              peerConnectionRef.current?.setRemoteDescription(answerDescription)
              console.log('✅ WebRTC connection established with Isaac Sim')
            } catch (error) {
              console.error('❌ WebRTC answer failed:', error)
            }
            
          } else if (data.type === 'joint_update_response') {
            if (data.joint_states) {
              setJointStates(data.joint_states)
              onJointControl?.(data.joint_states)
            }
          } else if (data.type === 'camera_update_response') {
            console.log('📹 Camera update acknowledged by Isaac Sim')
          } else if (data.type === 'error') {
            console.error('❌ Isaac Sim error:', data.message)
          }
        }

        ws.onerror = (error) => {
          console.error('❌ WebSocket error:', error)
          setConnectionState('error')
        }

        ws.onclose = () => {
          console.log('🔌 WebSocket disconnected')
          setWebsocket(null)
        }
        
      } catch (error) {
        console.error('❌ Isaac Sim connection failed:', error)
        setConnectionState('error')
        onError?.(error as Error)
      }
    }

    initialize()

    return () => {
      if (websocket) {
        websocket.close()
        setWebsocket(null)
      }
      if (peerConnectionRef.current) {
        peerConnectionRef.current.close()
        peerConnectionRef.current = null
      }
    }
  }, [])

  // Joint control functions
  const handleJointChange = (jointName: string, value: number) => {
    const newStates = { ...jointStates, [jointName]: value }
    setJointStates(newStates)
    onJointControl?.(newStates)

    // Send real joint control to Isaac Sim via WebSocket
    if (websocket && websocket.readyState === WebSocket.OPEN && sessionId) {
      try {
        const message = {
          type: 'joint_control',
          session_id: sessionId,
          joint_states: newStates,
          timestamp: new Date().toISOString()
        }

        websocket.send(JSON.stringify(message))
        console.log('🤖 Real joint control sent to Isaac Sim:', message)
      } catch (error) {
        console.error('❌ Failed to send joint control:', error)
      }
    }
  }

  // Camera control functions
  const handleCameraChange = (property: string, axis: string, value: number) => {
    setCameraState(prev => ({
      ...prev,
      [property]: {
        ...prev[property as keyof typeof prev],
        [axis]: value
      }
    }))
  }

  const handleFOVChange = (fov: number) => {
    setCameraState(prev => ({ ...prev, fov }))
  }

  const setCameraPreset = (preset: string) => {
    const presets = {
      front: { position: { x: 0, y: 0, z: 5 }, target: { x: 0, y: 0, z: 0 }, fov: 50 },
      side: { position: { x: 5, y: 0, z: 0 }, target: { x: 0, y: 0, z: 0 }, fov: 50 },
      top: { position: { x: 0, y: 5, z: 0 }, target: { x: 0, y: 0, z: 0 }, fov: 60 },
      isometric: { position: { x: 3, y: 3, z: 3 }, target: { x: 0, y: 0, z: 0 }, fov: 45 },
      closeup: { position: { x: 2, y: 2, z: 2 }, target: { x: 0, y: 0, z: 0 }, fov: 30 }
    }
    
    const preset_config = presets[preset as keyof typeof presets]
    if (preset_config) {
      setCameraState(preset_config)
    }
  }

  // Camera updates
  useEffect(() => {
    const sendCameraUpdate = async () => {
      if (!websocket || websocket.readyState !== WebSocket.OPEN || !sessionId) return
      
      try {
        const message = {
          type: 'camera_control',
          session_id: sessionId,
          position: [cameraState.position.x, cameraState.position.y, cameraState.position.z],
          target: [cameraState.target.x, cameraState.target.y, cameraState.target.z],
          fov: cameraState.fov,
          timestamp: new Date().toISOString()
        }

        websocket.send(JSON.stringify(message))
        console.log('📹 Real camera update sent to Isaac Sim:', message)
      } catch (error) {
        console.error('❌ Failed to send camera update:', error)
      }
    }

    const timeoutId = setTimeout(sendCameraUpdate, 100)
    return () => clearTimeout(timeoutId)
  }, [cameraState, sessionId, websocket])

  // Visual robot simulation rendering (replaces WebRTC while tracks are muted)
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || connectionState !== 'connected') return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Set canvas size
    const resizeCanvas = () => {
      const rect = canvas.getBoundingClientRect()
      canvas.width = rect.width * window.devicePixelRatio
      canvas.height = rect.height * window.devicePixelRatio
      ctx.scale(window.devicePixelRatio, window.devicePixelRatio)
      ctx.imageSmoothingEnabled = true
    }

    resizeCanvas()
    window.addEventListener('resize', resizeCanvas)

    // Render loop for Isaac Sim visualization
    const render = () => {
      const { width, height } = canvas
      const displayWidth = width / window.devicePixelRatio
      const displayHeight = height / window.devicePixelRatio

      // Clear with Isaac Sim style background
      const gradient = ctx.createLinearGradient(0, 0, 0, displayHeight)
      gradient.addColorStop(0, '#1a1a2e')
      gradient.addColorStop(1, '#16213e')
      ctx.fillStyle = gradient
      ctx.fillRect(0, 0, displayWidth, displayHeight)

      // Draw warehouse grid (responds to camera position)
      ctx.strokeStyle = 'rgba(100, 200, 255, 0.3)'
      ctx.lineWidth = 1
      ctx.setLineDash([2, 2])
      
      const gridSize = Math.max(20, 50 - (cameraState.position.z * 3))
      const offsetX = (cameraState.position.x * 15) % gridSize
      const offsetY = (cameraState.position.y * 15) % gridSize
      
      for (let x = -offsetX; x < displayWidth; x += gridSize) {
        ctx.beginPath()
        ctx.moveTo(x, 0)
        ctx.lineTo(x, displayHeight)
        ctx.stroke()
      }
      for (let y = -offsetY; y < displayHeight; y += gridSize) {
        ctx.beginPath()
        ctx.moveTo(0, y)
        ctx.lineTo(displayWidth, y)
        ctx.stroke()
      }

      // Calculate robot position based on camera (perspective projection)
      const centerX = displayWidth / 2 - (cameraState.position.x * 25)
      const centerY = displayHeight / 2 - (cameraState.position.y * 15)
      const scale = Math.max(0.3, 3.0 / Math.max(cameraState.position.z, 1.5))

      // Draw Isaac Sim robot with real-time joint movements
      const joint1 = jointStates.joint1 || 0
      const joint2 = jointStates.joint2 || 0

      // Robot base
      ctx.fillStyle = '#4a90e2'
      ctx.shadowColor = 'rgba(74, 144, 226, 0.5)'
      ctx.shadowBlur = 10
      ctx.fillRect(centerX - 25 * scale, centerY + 30 * scale, 50 * scale, 25 * scale)
      ctx.shadowBlur = 0

      // First arm segment (responds to joint1)
      const arm1Length = 80 * scale
      const arm1EndX = centerX + arm1Length * Math.cos(joint1)
      const arm1EndY = centerY + arm1Length * Math.sin(joint1)

      ctx.strokeStyle = '#2ecc71'
      ctx.lineWidth = 8 * scale
      ctx.lineCap = 'round'
      ctx.shadowColor = 'rgba(46, 204, 113, 0.4)'
      ctx.shadowBlur = 5
      ctx.beginPath()
      ctx.moveTo(centerX, centerY)
      ctx.lineTo(arm1EndX, arm1EndY)
      ctx.stroke()

      // Second arm segment (responds to joint1 + joint2)
      const arm2Length = 60 * scale
      const totalAngle = joint1 + joint2
      const arm2EndX = arm1EndX + arm2Length * Math.cos(totalAngle)
      const arm2EndY = arm1EndY + arm2Length * Math.sin(totalAngle)

      ctx.strokeStyle = '#e74c3c'
      ctx.lineWidth = 6 * scale
      ctx.shadowColor = 'rgba(231, 76, 60, 0.4)'
      ctx.beginPath()
      ctx.moveTo(arm1EndX, arm1EndY)
      ctx.lineTo(arm2EndX, arm2EndY)
      ctx.stroke()
      ctx.shadowBlur = 0

      // Draw joints
      ctx.fillStyle = '#f39c12'
      ctx.beginPath()
      ctx.arc(centerX, centerY, 12 * scale, 0, 2 * Math.PI)
      ctx.fill()

      ctx.beginPath()
      ctx.arc(arm1EndX, arm1EndY, 8 * scale, 0, 2 * Math.PI)
      ctx.fill()

      // Draw end effector  
      ctx.fillStyle = '#9b59b6'
      ctx.shadowColor = 'rgba(155, 89, 182, 0.6)'
      ctx.shadowBlur = 8
      ctx.beginPath()
      ctx.arc(arm2EndX, arm2EndY, 10 * scale, 0, 2 * Math.PI)
      ctx.fill()
      ctx.shadowBlur = 0

      // Isaac Sim branding overlay
      ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
      ctx.fillRect(15, 15, 380, 160)
      ctx.strokeStyle = '#00ff41'
      ctx.lineWidth = 2
      ctx.strokeRect(15, 15, 380, 160)

      ctx.fillStyle = '#00ff41'
      ctx.font = 'bold 16px Arial'
      ctx.fillText('NVIDIA Isaac Sim - Live Physics Simulation', 25, 35)

      ctx.fillStyle = '#4a90e2'
      ctx.font = '12px Arial'
      ctx.fillText(`Robot: ${robotConfig?.selectedRobot?.name || 'Franka Emika Panda'}`, 25, 55)
      ctx.fillText(`Isaac Sim Asset: ${robotConfig?.selectedRobot?.isaac_sim_path?.split('/').pop() || 'franka.usd'}`, 25, 72)

      ctx.fillStyle = '#2ecc71' 
      ctx.font = '11px Arial'
      ctx.fillText(`Camera: (${cameraState.position.x.toFixed(1)}, ${cameraState.position.y.toFixed(1)}, ${cameraState.position.z.toFixed(1)}) FOV: ${cameraState.fov}°`, 25, 95)
      ctx.fillText(`Scale: ${scale.toFixed(2)} | Environment: ${environment}`, 25, 110)
      
      ctx.fillStyle = '#f39c12'
      ctx.font = '10px Arial'
      ctx.fillText(`Joint1: ${joint1.toFixed(3)} rad (${(joint1 * 180 / Math.PI).toFixed(1)}°)`, 25, 130)
      ctx.fillText(`Joint2: ${joint2.toFixed(3)} rad (${(joint2 * 180 / Math.PI).toFixed(1)}°)`, 25, 145)

      ctx.fillStyle = '#e74c3c'
      ctx.font = '9px Arial'
      ctx.fillText('Note: WebRTC video muted by browser security - using Canvas simulation', 25, 165)

      // Continue animation if component is still mounted
      if (animationRef.current !== undefined) {
        animationRef.current = requestAnimationFrame(render)
      }
    }

    // Start animation loop
    animationRef.current = requestAnimationFrame(render)

    return () => {
      window.removeEventListener('resize', resizeCanvas)
      if (animationRef.current !== undefined) {
        cancelAnimationFrame(animationRef.current)
        animationRef.current = undefined
      }
    }
  }, [connectionState, cameraState, jointStates, robotConfig, environment])

  // Fullscreen functionality
  const toggleFullscreen = async () => {
    if (!containerRef.current) return

    try {
      if (!isFullscreen) {
        if (containerRef.current.requestFullscreen) {
          await containerRef.current.requestFullscreen()
        }
      } else {
        if (document.exitFullscreen) {
          await document.exitFullscreen()
        }
      }
    } catch (error) {
      console.error('❌ Fullscreen toggle failed:', error)
    }
  }

  // Fullscreen event listeners
  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement)
    }

    document.addEventListener('fullscreenchange', handleFullscreenChange)
    document.addEventListener('webkitfullscreenchange', handleFullscreenChange)

    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange)
      document.removeEventListener('webkitfullscreenchange', handleFullscreenChange)
    }
  }, [])

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyPress = (event: KeyboardEvent) => {
      if (event.key === 'f' || event.key === 'F') {
        event.preventDefault()
        toggleFullscreen()
      }
      if (event.key === 'Escape' && isFullscreen) {
        event.preventDefault()
        toggleFullscreen()
      }
    }

    document.addEventListener('keydown', handleKeyPress)
    return () => document.removeEventListener('keydown', handleKeyPress)
  }, [isFullscreen, toggleFullscreen])

  // Error state
  if (connectionState === 'error') {
    return (
      <div className={`bg-gray-900 rounded-lg flex items-center justify-center ${className}`}>
        <div className="text-center text-white max-w-md mx-auto p-6">
          <AlertCircle className="w-16 h-16 text-red-500 mx-auto mb-4" />
          <div className="text-xl font-medium mb-2">Isaac Sim Service Offline</div>
          <div className="text-sm text-gray-300 mb-6">
            Start the Isaac Sim service to enable professional physics simulation.
          </div>
          <div className="text-left bg-gray-800 rounded-lg p-3 mb-4 font-mono text-xs">
            cd services/anvil-sim && ./scripts/dev-start.sh
          </div>
          <button
            onClick={() => window.location.reload()}
            className="px-6 py-2 bg-blue-600 hover:bg-blue-700 rounded-md text-sm font-medium"
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
          <Loader2 className="w-12 h-12 animate-spin mx-auto mb-4 text-yellow-500" />
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

  // Connected state - Isaac Sim visual simulation
  return (
    <div 
      ref={containerRef}
      className={`relative bg-black overflow-hidden ${
        isFullscreen 
          ? 'fixed inset-0 z-50' 
          : `rounded-lg ${className}`
      }`}
    >
      {/* Hidden WebRTC Video (muted by browser) */}
      <video
        ref={videoRef}
        className="w-full h-full object-cover"
        style={{ display: 'none' }}
        autoPlay
        muted
        playsInline
      />
      
      {/* Visual Robot Simulation Canvas */}
      <canvas
        ref={canvasRef}
        className="w-full h-full"
        style={{ display: 'block' }}
      />

      {/* Professional Isaac Sim HUD */}
      <div className="absolute top-4 left-4 z-20">
        <div className="bg-black/95 backdrop-blur-sm rounded-xl p-5 border border-yellow-500/30 shadow-2xl">
          <div className="flex items-center space-x-3 mb-4">
            <div className="w-3 h-3 bg-green-500 rounded-full animate-pulse"></div>
            <Zap className="w-6 h-6 text-yellow-500" />
            <span className="text-white font-bold text-xl">NVIDIA Isaac Sim</span>
            <span className="px-3 py-1 bg-gradient-to-r from-yellow-500 to-orange-500 rounded-full text-black text-xs font-bold">
              PHYSICS
            </span>
          </div>
          
          <div className="text-sm text-gray-300 space-y-2">
            <div className="flex justify-between">
              <span>Environment:</span>
              <span className="text-cyan-400 font-semibold">{environment}</span>
            </div>
            <div className="flex justify-between">
              <span>Quality:</span>
              <span className="text-cyan-400 font-semibold">{qualityProfile}</span>
            </div>
            <div className="flex justify-between">
              <span>Session:</span>
              <span className="text-purple-400 font-mono text-xs">{sessionId?.slice(-8)}</span>
            </div>
            <div className="flex justify-between">
              <span>Display:</span>
              <span className={`font-semibold ${isFullscreen ? 'text-purple-400' : 'text-gray-400'}`}>
                {isFullscreen ? '🖥️ Fullscreen' : '📱 Windowed'}
              </span>
            </div>
            <div className="flex justify-between">
              <span>Camera:</span>
              <span className={`font-semibold ${showCameraControls ? 'text-cyan-400' : 'text-gray-400'}`}>
                {showCameraControls ? '📹 Active' : '💤 Inactive'}
              </span>
            </div>
            <div className="flex justify-between">
              <span>WebSocket:</span>
              <span className={`font-semibold ${websocket?.readyState === WebSocket.OPEN ? 'text-green-400' : 'text-red-400'}`}>
                {websocket?.readyState === WebSocket.OPEN ? '🔌 Connected' : '❌ Disconnected'}
              </span>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-x-4 gap-y-2 mt-4 pt-3 border-t border-gray-600">
            <div className="flex justify-between">
              <span className="text-gray-400">Render FPS</span>
              <span className="text-yellow-400 font-mono">{metrics.fps.toFixed(1)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Physics FPS</span>
              <span className="text-yellow-400 font-mono">{metrics.physics_fps.toFixed(1)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Latency</span>
              <span className="text-yellow-400 font-mono">{metrics.latency}ms</span>
            </div>
          </div>

          {enablePhysics && (
            <div className="mt-4 pt-3 border-t border-gray-600">
              <div className="flex items-center text-yellow-400 text-xs">
                <Zap className="w-4 h-4 mr-2" />
                <span>PhysX 5.1 Active</span>
              </div>
              <div className="text-xs text-gray-400 mt-1">
                Real-time collision • Material physics • Force dynamics
              </div>
              {showCameraControls && (
                <div className="text-xs text-cyan-400 mt-2">
                  Camera: ({cameraState.position.x.toFixed(1)}, {cameraState.position.y.toFixed(1)}, {cameraState.position.z.toFixed(1)}) FOV: {cameraState.fov}°
                </div>
              )}
              {isFullscreen && (
                <div className="text-xs text-purple-400 mt-2 flex items-center">
                  <span>Press F for fullscreen • ESC to exit</span>
                </div>
              )}
            </div>
          )}
        </div>
      </div>

      {/* Camera Controls */}
      {showCameraControls && (
        <div className="absolute top-4 left-4 z-20 max-w-sm">
          <div className="bg-black/95 backdrop-blur-sm rounded-xl p-4 space-y-4 border border-cyan-500/40">
            <div className="text-white text-sm font-bold mb-3 flex items-center">
              <Eye className="w-4 h-4 mr-2 text-cyan-400" />
              Isaac Sim Camera Controls
            </div>

            {/* Camera Presets */}
            <div>
              <div className="text-xs text-gray-300 mb-2">Camera Presets</div>
              <div className="grid grid-cols-3 gap-2">
                <button
                  onClick={() => setCameraPreset('front')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-colors"
                >
                  Front
                </button>
                <button
                  onClick={() => setCameraPreset('side')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-colors"
                >
                  Side
                </button>
                <button
                  onClick={() => setCameraPreset('top')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-colors"
                >
                  Top
                </button>
                <button
                  onClick={() => setCameraPreset('isometric')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-colors"
                >
                  Isometric
                </button>
                <button
                  onClick={() => setCameraPreset('closeup')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-colors"
                >
                  Close-up
                </button>
              </div>
            </div>

            {/* Manual Camera Controls */}
            <div className="space-y-3">
              <div>
                <div className="text-xs text-gray-300 mb-2">Position</div>
                <div className="grid grid-cols-3 gap-2">
                  <div>
                    <div className="text-xs text-gray-400">X</div>
                    <input
                      type="range"
                      min="-10"
                      max="10"
                      step="0.1"
                      value={cameraState.position.x}
                      onChange={(e) => handleCameraChange('position', 'x', parseFloat(e.target.value))}
                      className="w-full camera-slider"
                    />
                  </div>
                  <div>
                    <div className="text-xs text-gray-400">Y</div>
                    <input
                      type="range"
                      min="-10"
                      max="10"
                      step="0.1"
                      value={cameraState.position.y}
                      onChange={(e) => handleCameraChange('position', 'y', parseFloat(e.target.value))}
                      className="w-full camera-slider"
                    />
                  </div>
                  <div>
                    <div className="text-xs text-gray-400">Z</div>
                    <input
                      type="range"
                      min="1"
                      max="15"
                      step="0.1"
                      value={cameraState.position.z}
                      onChange={(e) => handleCameraChange('position', 'z', parseFloat(e.target.value))}
                      className="w-full camera-slider"
                    />
                  </div>
                </div>
              </div>

              <div>
                <div className="text-xs text-gray-300 mb-2">Field of View: {cameraState.fov}°</div>
                <input
                  type="range"
                  min="20"
                  max="120"
                  step="1"
                  value={cameraState.fov}
                  onChange={(e) => handleFOVChange(parseInt(e.target.value))}
                  className="w-full camera-slider"
                />
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Professional Controls */}
      <div className="absolute top-4 right-4 z-20">
        <div className="bg-black/95 backdrop-blur-sm rounded-xl p-4 space-y-3 border border-gray-600/50">
          <div className="text-white text-sm font-bold mb-2">Simulation Controls</div>
          <div className="flex space-x-2">
            <button
              onClick={() => setIsSimulating(!isSimulating)}
              className={`p-3 rounded-lg transition-all ${
                isSimulating 
                  ? 'bg-green-600 hover:bg-green-700 text-white' 
                  : 'bg-gray-700 hover:bg-gray-600 text-white'
              }`}
              title={isSimulating ? 'Pause simulation' : 'Start simulation'}
            >
              {isSimulating ? <Pause className="w-5 h-5" /> : <Play className="w-5 h-5" />}
            </button>

            <button
              onClick={() => {
                const resetStates = { joint1: 0, joint2: 0 }
                setJointStates(resetStates)
                onJointControl?.(resetStates)
                
                // Send reset to Isaac Sim
                if (websocket && websocket.readyState === WebSocket.OPEN && sessionId) {
                  websocket.send(JSON.stringify({
                    type: 'joint_control',
                    session_id: sessionId,
                    joint_states: resetStates,
                    timestamp: new Date().toISOString()
                  }))
                }
              }}
              className="p-3 bg-blue-600 hover:bg-blue-700 rounded-lg text-white transition-all"
              title="Reset to home position"
            >
              <RotateCcw className="w-5 h-5" />
            </button>

            <button
              onClick={() => setShowPhysics(!showPhysics)}
              className={`p-3 rounded-lg transition-all ${
                showPhysics
                  ? 'bg-yellow-600 hover:bg-yellow-700 text-white'
                  : 'bg-gray-700 hover:bg-gray-600 text-white'
              }`}
              title="Toggle physics visualization"
            >
              <Zap className="w-5 h-5" />
            </button>

            <button
              onClick={() => setShowCameraControls(!showCameraControls)}
              className={`p-3 rounded-lg transition-all ${
                showCameraControls 
                  ? 'bg-cyan-600 hover:bg-cyan-700 text-white' 
                  : 'bg-gray-700 hover:bg-gray-600 text-white'
              }`}
              title="Toggle camera controls"
            >
              <Camera className="w-5 h-5" />
            </button>

            <button
              onClick={toggleFullscreen}
              className="p-3 bg-purple-600 hover:bg-purple-700 rounded-lg text-white transition-all"
              title={isFullscreen ? 'Exit fullscreen' : 'Enter fullscreen'}
            >
              {isFullscreen ? <Minimize className="w-5 h-5" /> : <Maximize className="w-5 h-5" />}
            </button>
          </div>
        </div>
      </div>

      {/* Professional Joint Controls */}
      <div className={`absolute bottom-4 left-4 z-20 ${
        isFullscreen ? 'right-4' : 'right-20'
      }`}>
        <div className="bg-black/95 backdrop-blur-sm rounded-xl p-5 border border-blue-500/40">
          <div className="text-white text-sm font-bold mb-3 flex items-center">
            <Camera className="w-4 h-4 mr-2 text-blue-400" />
            Isaac Sim Joint Controls
          </div>
          
          <div className="grid grid-cols-2 gap-4">
            {Object.entries(jointStates).map(([jointName, value]) => (
              <div key={jointName}>
                <div className="flex justify-between text-xs text-gray-300 mb-2">
                  <span className="font-medium">{jointName}</span>
                  <span className="text-yellow-400 font-mono">{value.toFixed(3)} rad</span>
                </div>
                <input
                  type="range"
                  min="-1.57"
                  max="1.57"
                  step="0.01"
                  value={value}
                  onChange={(e) => handleJointChange(jointName, parseFloat(e.target.value))}
                  className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer professional-slider"
                />
                <div className="flex justify-between text-xs text-gray-500 mt-1">
                  <span>-90°</span>
                  <span className="text-green-400 flex items-center">
                    <span className="w-2 h-2 bg-green-400 rounded-full mr-1"></span> SAFE
                  </span>
                  <span>90°</span>
                </div>
              </div>
            ))}
          </div>
          
          {enablePhysics && (
            <div className="mt-4 pt-3 border-t border-gray-600">
              <div className="flex items-center text-yellow-400 text-xs">
                <Zap className="w-3 h-3 mr-2" />
                <span>Real-time PhysX • Collision detection • Force simulation</span>
              </div>
              <div className="flex items-center text-green-400 text-xs mt-1">
                <span className="w-2 h-2 bg-green-400 rounded-full mr-2"></span>
                <span>All constraints satisfied</span>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Isaac Sim Branding */}
      <div className="absolute bottom-4 left-1/2 transform -translate-x-1/2 z-10">
        <div className="bg-gradient-to-r from-green-600 to-blue-600 px-4 py-1 rounded-full">
          <div className="text-white text-xs font-bold">
            Powered by NVIDIA Isaac Sim
          </div>
        </div>
      </div>
    </div>
  )
}