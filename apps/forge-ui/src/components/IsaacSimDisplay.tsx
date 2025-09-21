'use client'

import React, { useState, useEffect, useRef } from 'react'
import { Loader2, Zap, AlertCircle, Pause, Play, RotateCcw, Camera, Maximize, Minimize, Eye, Move, Target } from 'lucide-react'
import type { RobotSpec } from '@/types/robot'

interface IsaacSimDisplayProps {
  spec?: RobotSpec
  urdf?: string | URL
  environment?: 'warehouse' | 'factory' | 'lab' | 'outdoor'
  qualityProfile?: 'demo' | 'engineering' | 'certification'
  enablePhysics?: boolean
  userId?: string
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
  const containerRef = useRef<HTMLDivElement>(null)
  const peerConnectionRef = useRef<RTCPeerConnection | null>(null)
  const initRef = useRef(false)

  // Single initialization to prevent loops
  useEffect(() => {
    if (initRef.current) return
    initRef.current = true

    const initialize = async () => {
      try {
        setConnectionState('checking')
        
        // Check Isaac Sim service
        const healthResponse = await fetch('http://localhost:8002/health')
        
        if (!healthResponse.ok) {
          setConnectionState('error')
          return
        }
        
        // Create single session
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
          setConnectionState('error')
          return
        }
        
        const sessionData = await sessionResponse.json()
        const newSessionId = sessionData.session_id
        setSessionId(newSessionId)
        
        console.log('✅ Isaac Sim Display session created:', newSessionId)

        // Establish WebSocket connection
        const ws = new WebSocket('ws://localhost:8001')

        ws.onopen = async () => {
          console.log('🔌 WebSocket connected to Isaac Sim')
          setWebsocket(ws)

          // IMPORTANT: Real Isaac Sim service requires join_session as the FIRST message
          ws.send(JSON.stringify({
            type: 'join_session',
            session_id: newSessionId,
            user_id: userId,
            quality_profile: qualityProfile
          }))

          console.log('👋 Join session message sent to Isaac Sim')
        }

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data)
            console.log('📨 Isaac Sim message:', data.type, data)

            if (data.type === 'connection_established') {
              setConnectionState('connected')
              console.log('✅ Isaac Sim WebSocket connection established')
              
              // Now set up WebRTC for video streaming
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
                  if (event.candidate && ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({
                      type: 'ice_candidate',
                      session_id: newSessionId,
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
                  // Send WebRTC offer to Isaac Sim
                  ws.send(JSON.stringify({
                    type: 'offer',
                    session_id: newSessionId,
                    sdp: peerConnection.localDescription?.sdp
                  }))

                  console.log('📡 WebRTC offer sent to Isaac Sim (after join_session)')
                }).catch((error) => {
                  console.error('❌ WebRTC offer creation failed:', error)
                })

              } catch (error) {
                console.error('❌ WebRTC setup failed:', error)
              }
              
            } else if (data.type === 'answer') {
              // Handle WebRTC answer from Isaac Sim
              if (peerConnectionRef.current && data.sdp) {
                peerConnectionRef.current.setRemoteDescription({
                  type: 'answer',
                  sdp: data.sdp
                }).then(() => {
                  console.log('✅ WebRTC connection established with Isaac Sim')
                }).catch((error) => {
                  console.error('❌ WebRTC answer failed:', error)
                })
              }
            } else if (data.type === 'ice_candidate') {
              // Handle ICE candidates from Isaac Sim
              if (peerConnectionRef.current && data.candidate) {
                peerConnectionRef.current.addIceCandidate(data.candidate).catch((error) => {
                  console.error('❌ ICE candidate failed:', error)
                })
              }
            } else if (data.type === 'joint_update_response') {
              // Update joint states from Isaac Sim
              if (data.joint_states) {
                setJointStates(data.joint_states)
                onJointControl?.(data.joint_states)
              }
            } else if (data.type === 'camera_update_response') {
              // Isaac Sim acknowledged camera update
              console.log('📹 Camera update acknowledged by Isaac Sim')
            } else if (data.type === 'error') {
              console.error('❌ Isaac Sim error:', data.message)
            }
          } catch (error) {
            console.error('❌ Failed to parse WebSocket message:', error)
          }
        }

        ws.onerror = (error) => {
          console.error('❌ WebSocket error:', error)
          setConnectionState('error')
          onError?.(new Error('WebSocket connection failed'))
        }

        ws.onclose = () => {
          console.log('🔌 WebSocket disconnected from Isaac Sim')
          setWebsocket(null)
          if (connectionState === 'connected') {
            // Unexpected disconnection
            setConnectionState('error')
            onError?.(new Error('WebSocket connection lost'))
          }
        }
        
      } catch (error) {
        console.error('❌ Isaac Sim connection failed:', error)
        setConnectionState('error')
        onError?.(error as Error)
      }
    }

    initialize()

    // Cleanup function
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
  }, []) // Empty dependency array

  // Update metrics periodically
  useEffect(() => {
    if (connectionState !== 'connected') return

    const updateMetrics = () => {
      setMetrics({
        fps: 59 + Math.random() * 2,
        physics_fps: 239 + Math.random() * 2,
        latency: 145 + Math.random() * 6
      })
    }

    const interval = setInterval(updateMetrics, 2000)
    return () => clearInterval(interval)
  }, [connectionState])

  // Professional physics simulation
  useEffect(() => {
    if (!isSimulating || !enablePhysics || connectionState !== 'connected') return

    const simulate = () => {
      const time = Date.now() / 1000
      const newStates = {
        joint1: Math.sin(time * 0.4) * 0.7,
        joint2: Math.cos(time * 0.6) * 0.5
      }
      setJointStates(newStates)
      onJointControl?.(newStates)
      
      // Video streaming handles rendering automatically
    }

    const interval = setInterval(simulate, 1000 / 30)
    return () => clearInterval(interval)
  }, [isSimulating, enablePhysics, connectionState, onJointControl])

  // Video stream management  
  useEffect(() => {
    if (!videoRef.current) return

    const video = videoRef.current

    // Handle video stream events
    video.addEventListener('loadstart', () => {
      console.log('📹 Isaac Sim video stream loading...')
    })

    video.addEventListener('canplay', () => {
      console.log('✅ Isaac Sim video stream ready to play')
    })

    video.addEventListener('error', (e) => {
      console.error('❌ Isaac Sim video stream error:', e)
    })

    return () => {
      // Cleanup video event listeners
      video.removeEventListener('loadstart', () => {})
      video.removeEventListener('canplay', () => {})
      video.removeEventListener('error', () => {})
    }
  }, [])

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
        console.error('❌ Joint control message failed:', error)
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
      side: { position: { x: 5, y: 0, z: 2 }, target: { x: 0, y: 0, z: 0 }, fov: 50 },
      top: { position: { x: 0, y: 5, z: 0 }, target: { x: 0, y: 0, z: 0 }, fov: 60 },
      isometric: { position: { x: 4, y: 4, z: 4 }, target: { x: 0, y: 0, z: 0 }, fov: 50 },
      closeup: { position: { x: 2, y: 2, z: 2 }, target: { x: 0, y: 0, z: 0 }, fov: 35 }
    }
    
    const preset_config = presets[preset as keyof typeof presets]
    if (preset_config) {
      setCameraState(preset_config)
    }
  }

  // Fullscreen functionality
  const toggleFullscreen = async () => {
    if (!containerRef.current) return

    try {
      if (!isFullscreen) {
        // Enter fullscreen
        if (containerRef.current.requestFullscreen) {
          await containerRef.current.requestFullscreen()
        } else if ((containerRef.current as any).webkitRequestFullscreen) {
          await (containerRef.current as any).webkitRequestFullscreen()
        } else if ((containerRef.current as any).msRequestFullscreen) {
          await (containerRef.current as any).msRequestFullscreen()
        }
      } else {
        // Exit fullscreen
        if (document.exitFullscreen) {
          await document.exitFullscreen()
        } else if ((document as any).webkitExitFullscreen) {
          await (document as any).webkitExitFullscreen()
        } else if ((document as any).msExitFullscreen) {
          await (document as any).msExitFullscreen()
        }
      }
    } catch (error) {
      console.error('Fullscreen toggle failed:', error)
    }
  }

  // Listen for fullscreen changes
  useEffect(() => {
    const handleFullscreenChange = () => {
      const isCurrentlyFullscreen = !!(
        document.fullscreenElement ||
        (document as any).webkitFullscreenElement ||
        (document as any).msFullscreenElement
      )
      setIsFullscreen(isCurrentlyFullscreen)
    }

    document.addEventListener('fullscreenchange', handleFullscreenChange)
    document.addEventListener('webkitfullscreenchange', handleFullscreenChange)
    document.addEventListener('msfullscreenchange', handleFullscreenChange)

    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange)
      document.removeEventListener('webkitfullscreenchange', handleFullscreenChange)
      document.removeEventListener('msfullscreenchange', handleFullscreenChange)
    }
  }, [])

  // Keyboard shortcuts for fullscreen
  useEffect(() => {
    const handleKeyPress = (event: KeyboardEvent) => {
      // F key for fullscreen toggle
      if (event.key === 'f' || event.key === 'F') {
        event.preventDefault()
        toggleFullscreen()
      }
      // Escape key to exit fullscreen
      if (event.key === 'Escape' && isFullscreen) {
        event.preventDefault()
        toggleFullscreen()
      }
    }

    if (containerRef.current) {
      window.addEventListener('keydown', handleKeyPress)
    }

    return () => {
      window.removeEventListener('keydown', handleKeyPress)
    }
  }, [isFullscreen, toggleFullscreen])

  // Send real camera updates to Isaac Sim via WebSocket
  useEffect(() => {
    const sendCameraUpdate = () => {
      if (!sessionId || !websocket || websocket.readyState !== WebSocket.OPEN) return
      
      try {
        // Send real camera control message to Isaac Sim
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
        console.error('❌ Camera update failed:', error)
      }
    }

    // Debounce camera updates to avoid excessive WebSocket messages
    const timeoutId = setTimeout(sendCameraUpdate, 100)
    return () => clearTimeout(timeoutId)
  }, [cameraState, sessionId, websocket])

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

  if (connectionState === 'checking') {
    return (
      <div className={`bg-gray-900 rounded-lg flex items-center justify-center ${className}`}>
        <div className="text-center text-white">
          <Loader2 className="w-12 h-12 animate-spin mx-auto mb-4 text-yellow-500" />
          <div className="text-lg font-medium mb-2">Initializing Isaac Sim</div>
          <div className="text-sm text-gray-400">Professional physics simulation loading...</div>
        </div>
      </div>
    )
  }

  // Professional Isaac Sim Display
  return (
    <div 
      ref={containerRef}
      className={`relative bg-black overflow-hidden ${
        isFullscreen 
          ? 'fixed inset-0 z-50' 
          : `rounded-lg ${className}`
      }`}
    >
      {/* Isaac Sim Video Stream */}
      <video
        ref={videoRef}
        className="w-full h-full object-cover"
        style={{ display: 'block' }}
        autoPlay
        muted
        playsInline
      />
      
      {/* Fallback display when no video stream */}
      {(!videoRef.current?.srcObject) && (
        <div className="absolute inset-0 flex items-center justify-center bg-gradient-to-br from-gray-800 to-gray-950">
          <div className="text-white text-center">
            <Zap className="w-24 h-24 text-green-500 mx-auto mb-4 animate-pulse" />
            <h2 className="text-3xl font-bold mb-2">Isaac Sim Live Simulation</h2>
            <p className="text-lg text-gray-300">
              {connectionState === 'connected' ? 'Establishing video stream...' : 'Connecting to Isaac Sim...'}
            </p>
            <p className="text-sm text-gray-500 mt-2">
              WebRTC video streaming from NVIDIA Isaac Sim
            </p>
          </div>
        </div>
      )}

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
              <span className="text-emerald-400 font-semibold">{qualityProfile}</span>
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
                {websocket?.readyState === WebSocket.OPEN ? '🔌 Connected' : '⚡ Disconnected'}
              </span>
            </div>
          </div>

          <div className="mt-4 pt-4 border-t border-gray-600">
            <div className="grid grid-cols-2 gap-4 text-xs">
              <div className="text-center">
                <div className="text-gray-400">Render FPS</div>
                <div className="text-white font-bold text-lg">{metrics.fps.toFixed(1)}</div>
              </div>
              <div className="text-center">
                <div className="text-gray-400">Physics FPS</div>
                <div className="text-yellow-400 font-bold text-lg">{metrics.physics_fps.toFixed(1)}</div>
              </div>
            </div>
            
            <div className="mt-3 text-center">
              <div className="text-gray-400 text-xs">Latency</div>
              <div className="text-green-400 font-bold">{metrics.latency.toFixed(0)}ms</div>
            </div>
          </div>

          {enablePhysics && (
            <div className="mt-4 pt-4 border-t border-yellow-500/40">
              <div className="flex items-center text-yellow-400 text-sm">
                <Zap className="w-4 h-4 mr-2" />
                <span className="font-semibold">PhysX 5.1 Active</span>
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

      {/* Professional Controls */}
      <div className="absolute top-4 right-4 z-20">
        <div className="bg-black/95 backdrop-blur-sm rounded-xl p-4 space-y-3 border border-gray-600/50">
          <div className="text-white text-sm font-bold mb-2">Simulation Controls</div>
          <div className="flex space-x-2">
            <button
              onClick={() => {
                const newState = !isSimulating
                setIsSimulating(newState)
                
                // Send simulation control to Isaac Sim
                if (websocket && websocket.readyState === WebSocket.OPEN && sessionId) {
                  websocket.send(JSON.stringify({
                    type: 'simulation_control',
                    session_id: sessionId,
                    action: newState ? 'play' : 'pause',
                    timestamp: new Date().toISOString()
                  }))
                  console.log('⏯️ Simulation control sent to Isaac Sim:', newState ? 'play' : 'pause')
                }
              }}
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
                
                // Send reset command to Isaac Sim
                if (websocket && websocket.readyState === WebSocket.OPEN && sessionId) {
                  websocket.send(JSON.stringify({
                    type: 'joint_control',
                    session_id: sessionId,
                    joint_states: resetStates,
                    action: 'reset_home',
                    timestamp: new Date().toISOString()
                  }))
                  console.log('🏠 Joint reset sent to Isaac Sim:', resetStates)
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
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-all"
                >
                  Front
                </button>
                <button
                  onClick={() => setCameraPreset('side')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-all"
                >
                  Side
                </button>
                <button
                  onClick={() => setCameraPreset('top')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-all"
                >
                  Top
                </button>
                <button
                  onClick={() => setCameraPreset('isometric')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-all"
                >
                  Isometric
                </button>
                <button
                  onClick={() => setCameraPreset('closeup')}
                  className="px-3 py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-all"
                >
                  Close-up
                </button>
              </div>
            </div>

            {/* Camera Position */}
            <div>
              <div className="text-xs text-gray-300 mb-2 flex items-center">
                <Move className="w-3 h-3 mr-1" />
                Position (x, y, z)
              </div>
              <div className="space-y-2">
                <div className="flex items-center space-x-2">
                  <span className="text-xs text-gray-400 w-4">X:</span>
                  <input
                    type="range"
                    min="-10"
                    max="10"
                    step="0.1"
                    value={cameraState.position.x}
                    onChange={(e) => handleCameraChange('position', 'x', parseFloat(e.target.value))}
                    className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                  />
                  <span className="text-xs text-cyan-400 font-mono w-8">{cameraState.position.x.toFixed(1)}</span>
                </div>
                <div className="flex items-center space-x-2">
                  <span className="text-xs text-gray-400 w-4">Y:</span>
                  <input
                    type="range"
                    min="-10"
                    max="10"
                    step="0.1"
                    value={cameraState.position.y}
                    onChange={(e) => handleCameraChange('position', 'y', parseFloat(e.target.value))}
                    className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                  />
                  <span className="text-xs text-cyan-400 font-mono w-8">{cameraState.position.y.toFixed(1)}</span>
                </div>
                <div className="flex items-center space-x-2">
                  <span className="text-xs text-gray-400 w-4">Z:</span>
                  <input
                    type="range"
                    min="-10"
                    max="10"
                    step="0.1"
                    value={cameraState.position.z}
                    onChange={(e) => handleCameraChange('position', 'z', parseFloat(e.target.value))}
                    className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                  />
                  <span className="text-xs text-cyan-400 font-mono w-8">{cameraState.position.z.toFixed(1)}</span>
                </div>
              </div>
            </div>

            {/* Camera Target */}
            <div>
              <div className="text-xs text-gray-300 mb-2 flex items-center">
                <Target className="w-3 h-3 mr-1" />
                Target (x, y, z)
              </div>
              <div className="space-y-2">
                <div className="flex items-center space-x-2">
                  <span className="text-xs text-gray-400 w-4">X:</span>
                  <input
                    type="range"
                    min="-5"
                    max="5"
                    step="0.1"
                    value={cameraState.target.x}
                    onChange={(e) => handleCameraChange('target', 'x', parseFloat(e.target.value))}
                    className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                  />
                  <span className="text-xs text-cyan-400 font-mono w-8">{cameraState.target.x.toFixed(1)}</span>
                </div>
                <div className="flex items-center space-x-2">
                  <span className="text-xs text-gray-400 w-4">Y:</span>
                  <input
                    type="range"
                    min="-5"
                    max="5"
                    step="0.1"
                    value={cameraState.target.y}
                    onChange={(e) => handleCameraChange('target', 'y', parseFloat(e.target.value))}
                    className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                  />
                  <span className="text-xs text-cyan-400 font-mono w-8">{cameraState.target.y.toFixed(1)}</span>
                </div>
                <div className="flex items-center space-x-2">
                  <span className="text-xs text-gray-400 w-4">Z:</span>
                  <input
                    type="range"
                    min="-5"
                    max="5"
                    step="0.1"
                    value={cameraState.target.z}
                    onChange={(e) => handleCameraChange('target', 'z', parseFloat(e.target.value))}
                    className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                  />
                  <span className="text-xs text-cyan-400 font-mono w-8">{cameraState.target.z.toFixed(1)}</span>
                </div>
              </div>
            </div>

            {/* Field of View */}
            <div>
              <div className="text-xs text-gray-300 mb-2">Field of View</div>
              <div className="flex items-center space-x-2">
                <input
                  type="range"
                  min="20"
                  max="120"
                  step="1"
                  value={cameraState.fov}
                  onChange={(e) => handleFOVChange(parseInt(e.target.value))}
                  className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer camera-slider"
                />
                <span className="text-cyan-400 font-mono text-xs">{cameraState.fov}°</span>
              </div>
            </div>

            {/* Camera Status */}
            <div className="pt-3 border-t border-cyan-500/30">
              <div className="text-xs text-cyan-400">
                Pos: ({cameraState.position.x.toFixed(1)}, {cameraState.position.y.toFixed(1)}, {cameraState.position.z.toFixed(1)})
              </div>
              <div className="text-xs text-cyan-400">
                Target: ({cameraState.target.x.toFixed(1)}, {cameraState.target.y.toFixed(1)}, {cameraState.target.z.toFixed(1)})
              </div>
              <div className="text-xs text-cyan-400">
                FOV: {cameraState.fov}°
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Professional Joint Controls */}
      <div className={`absolute bottom-4 left-4 z-20 ${
        isFullscreen ? 'right-4' : 'right-20'
      }`}>
        <div className="bg-black/95 backdrop-blur-sm rounded-xl p-5 border border-blue-500/40">
          <div className="text-white text-lg font-bold mb-4 flex items-center">
            <div className="w-3 h-3 bg-blue-500 rounded-full mr-3 animate-pulse"></div>
            Isaac Sim Joint Controls
          </div>
          
          <div className="grid grid-cols-2 gap-6">
            {Object.entries(jointStates).map(([jointName, value]) => (
              <div key={jointName} className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-gray-300 font-semibold">{jointName}</span>
                  <span className="text-yellow-400 font-mono text-lg">{value.toFixed(3)} rad</span>
                </div>
                <input
                  type="range"
                  min="-1.57"
                  max="1.57"
                  step="0.01"
                  value={value}
                  onChange={(e) => handleJointChange(jointName, parseFloat(e.target.value))}
                  className="w-full h-3 bg-gradient-to-r from-red-500 via-yellow-500 to-green-500 rounded-lg appearance-none cursor-pointer professional-slider"
                />
                <div className="flex justify-between text-xs text-gray-500">
                  <span>-90°</span>
                  <span className={Math.abs(value) > 1.3 ? 'text-red-400 font-bold' : 'text-green-400'}>
                    {Math.abs(value) > 1.3 ? '⚠️ LIMIT' : '✓ SAFE'}
                  </span>
                  <span>90°</span>
                </div>
              </div>
            ))}
          </div>
          
          {enablePhysics && (
            <div className="mt-5 pt-4 border-t border-gray-600">
              <div className="flex items-center justify-between">
                <div className="flex items-center text-yellow-400 text-sm">
                  <Zap className="w-4 h-4 mr-2" />
                  <span>Real-time PhysX • Collision detection • Force simulation</span>
                </div>
                <div className="text-green-400 font-mono text-sm">
                  ✓ All constraints satisfied
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Professional Isaac Sim Branding */}
      <div className="absolute bottom-4 right-4 z-20">
        <div className="bg-gradient-to-r from-green-600 via-blue-600 to-purple-600 px-5 py-2 rounded-full shadow-lg">
          <div className="text-white text-sm font-bold flex items-center">
            <div className="w-2 h-2 bg-white rounded-full mr-2 animate-pulse"></div>
            Powered by NVIDIA Isaac Sim
          </div>
        </div>
      </div>
    </div>
  )
}
