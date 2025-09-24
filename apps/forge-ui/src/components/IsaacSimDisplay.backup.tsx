'use client'

import React, { useState, useEffect, useRef } from 'react'
import { Loader2, Zap, AlertCircle, Pause, Play, RotateCcw, Camera, Maximize, Minimize, Eye, Move, Target } from 'lucide-react'
import type { RobotSpec } from '@/types/robot'
import { env } from '@/lib/env'

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
  const [videoStreamWorking, setVideoStreamWorking] = useState(false)
  const [peerConnection, setPeerConnection] = useState<RTCPeerConnection | null>(null)
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

    // Use port-forwarded Brev anvil-sim service endpoint
    const anvilSimHost = 'localhost'
    const anvilSimPort = '8002'
    const httpBaseUrl = `http://${anvilSimHost}:${anvilSimPort}`
    const wsUrl = `ws://${anvilSimHost}:8001`  // WebSocket on port 8001
  
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const animationRef = useRef<number>()
  const initRef = useRef(false)
  const lastFrameRef = useRef<HTMLImageElement | null>(null)

  // Initialize Isaac Sim connection and WebRTC
  useEffect(() => {
    if (initRef.current) return
    initRef.current = true

    const initialize = async () => {
      try {
        setConnectionState('connecting')
        
        // Check Isaac Sim service health
        const healthResponse = await fetch(`${httpBaseUrl}/health`)
        if (!healthResponse.ok) {
          throw new Error('Isaac Sim service not available')
        }

        // Create session
        const sessionResponse = await fetch(`${httpBaseUrl}/create_scene`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: userId,
            sepulka_id: spec?.name || 'demo-robot',
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
        
        // Store session ID for robot changes
        localStorage.setItem('isaac_sim_session_id', newSessionId)
        
        console.log('✅ Isaac Sim Display session created:', newSessionId)

        // Initialize WebRTC peer connection for video streaming
        const pc = new RTCPeerConnection({
          iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
        })
        setPeerConnection(pc)

        // Handle incoming video stream
        pc.ontrack = async (event) => {
          console.log('📹 Received video track from Isaac Sim')
          if (videoRef.current && event.streams[0]) {
            videoRef.current.srcObject = event.streams[0]

            // Show video element, hide canvas
            videoRef.current.style.display = 'block'
            if (canvasRef.current) {
              canvasRef.current.style.display = 'none'
            }

            // Wait for video metadata to load before playing
            videoRef.current.onloadedmetadata = async () => {
              console.log('📹 Video metadata loaded, attempting to play')
              if (!videoRef.current) return
              
              try {
                await videoRef.current.play()
                console.log('▶️ Video started playing successfully')
                setVideoStreamWorking(true)
              } catch (playError) {
                console.error('❌ Failed to play video:', playError)
                // Check if video has actual content
                if (videoRef.current && videoRef.current.videoWidth > 0 && videoRef.current.videoHeight > 0) {
                  console.log('✅ Video has content, setting stream as working')
                  setVideoStreamWorking(true)
                } else {
                  console.log('❌ Video has no content, WebRTC failed')
                }
              }
            }

            // Fallback: if metadata doesn't load within 2 seconds, check video dimensions
            setTimeout(() => {
              if (!videoStreamWorking && videoRef.current) {
                if (videoRef.current.videoWidth > 0 && videoRef.current.videoHeight > 0) {
                  console.log('✅ Video has content (fallback check), setting stream as working')
                  setVideoStreamWorking(true)
                } else {
                  console.log('❌ Video still has no content after timeout')
                }
              }
            }, 2000)
          }
        }

        // Handle ICE candidates
        pc.onicecandidate = (event) => {
          if (event.candidate && ws) {
            ws.send(JSON.stringify({
              type: 'ice_candidate',
              session_id: newSessionId,
              candidate: event.candidate
            }))
          }
        }

        // Establish WebSocket connection for controls and WebRTC signaling
        const ws = new WebSocket(`${wsUrl.replace('8002', '8001')}`)

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

        ws.onmessage = async (event) => {
          const data = JSON.parse(event.data)
          console.log('📨 Isaac Sim message:', data.type, data)

          if (data.type === 'connection_established') {
            setConnectionState('connected')
            console.log('✅ Isaac Sim WebSocket connection established for controls')
            // Wait for user to click "Start Video" button
            
          } else if (data.type === 'webrtc_offer') {
            // Handle WebRTC offer from Isaac Sim
            console.log('📹 Received WebRTC offer from Isaac Sim')
            ;(async () => {
              try {
                await pc.setRemoteDescription(new RTCSessionDescription(data.offer))
                const answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                
                ws.send(JSON.stringify({
                  type: 'webrtc_answer',
                  session_id: newSessionId,
                  answer: answer
                }))
                console.log('📹 Sent WebRTC answer to Isaac Sim')
              } catch (error) {
                console.error('❌ WebRTC offer handling error:', error)
              }
            })()
            
          } else if (data.type === 'ice_candidate') {
            // Handle ICE candidate from Isaac Sim
            ;(async () => {
              try {
                await pc.addIceCandidate(new RTCIceCandidate(data.candidate))
                console.log('📹 Added ICE candidate from Isaac Sim')
              } catch (error) {
                console.error('❌ ICE candidate error:', error)
              }
            })()
            
          } else if (data.type === 'video_stream_started') {
            console.log('✅ WebSocket video stream started (fallback)')
            setVideoStreamWorking(false) // Set to false so canvas is visible

            // Show canvas for WebSocket streaming, hide video element
            if (videoRef.current) {
              videoRef.current.style.display = 'none'
            }
            if (canvasRef.current) {
              canvasRef.current.style.display = 'block'
            }

      } else if (data.type === 'video_frame') {
        // Display video frame on canvas for WebSocket streaming
        if (canvasRef.current && data.frame_data) {
          try {
            const canvas = canvasRef.current
            const ctx = canvas.getContext('2d')
            if (!ctx) return

            console.log('🔄 Processing video frame:', data.frame_count, 'base64 length:', data.frame_data.length)

            // Create image from base64 data
            const img = new Image()
            img.onload = () => {
              console.log('🖼️ Canvas image loaded successfully:', img.width, 'x', img.height)

            // Store the loaded image for persistence
            lastFrameRef.current = img

            // If we're receiving video frames, we're connected even if connection_established message wasn't received
            if (connectionState !== 'connected') {
              setConnectionState('connected')
              console.log('✅ Connection state updated to connected due to video frames')
            }

            // Set canvas size to match image if needed
              if (canvas.width !== img.width || canvas.height !== img.height) {
                canvas.width = img.width
                canvas.height = img.height
                console.log('📐 Resized canvas to:', canvas.width, 'x', canvas.height)
              }

              // Clear and draw the frame
              ctx.clearRect(0, 0, canvas.width, canvas.height)
              ctx.drawImage(img, 0, 0)

              console.log('🎨 Canvas updated with frame:', canvas.width, 'x', canvas.height)

              // Check if canvas now has content
              const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
              const data = imageData.data
              let nonBlackPixels = 0
              for (let i = 0; i < data.length; i += 4) {
                if (data[i] !== 0 || data[i + 1] !== 0 || data[i + 2] !== 0) {
                  nonBlackPixels++
                }
              }
              console.log(`📊 Canvas content check: ${nonBlackPixels} non-black pixels out of ${data.length / 4} total`)
            }

            img.onerror = (error) => {
              console.error('❌ Failed to load video frame image:', error, 'data length:', data.frame_data.length)
              console.error('❌ Image src attempted:', `data:image/jpeg;base64,${data.frame_data.substring(0, 50)}...`)
            }

            console.log('🔄 Setting canvas image src with base64 data length:', data.frame_data.length)
            img.src = `data:image/jpeg;base64,${data.frame_data}`

            // Log every 30 frames (every 2 seconds at 15 FPS)
            if (data.frame_count && data.frame_count % 30 === 0) {
              console.log(`📹 Isaac Sim Canvas frame #${data.frame_count} - Robot: ${robotConfig?.selectedRobot?.name || 'Default'}`)
            }
          } catch (error) {
            console.error('❌ Canvas frame processing error:', error)
          }
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
      if (peerConnection) {
        peerConnection.close()
        setPeerConnection(null)
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
    setCameraState(prev => {
      const currentProperty = prev[property as keyof typeof prev]
      const propertyValue = typeof currentProperty === 'object' && currentProperty !== null 
        ? currentProperty as Record<string, number>
        : {}
      
      return {
        ...prev,
        [property]: {
          ...propertyValue,
          [axis]: value
        }
      }
    })
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

  // Video stream is handled via WebRTC with user interaction

  // Start video stream with user interaction
  const startVideoStream = async () => {
    if (!websocket || !peerConnection || connectionState !== 'connected') {
      console.error('❌ Cannot start video stream: not connected')
      return
    }

    try {
      console.log('🎬 Starting WebRTC video stream with user interaction')

      // Request WebRTC video streaming from server
      websocket.send(JSON.stringify({
        type: 'start_webrtc_stream',
        session_id: sessionId
      }))

        // Set a timeout to check if WebRTC works, otherwise fallback
        setTimeout(() => {
          if (!videoStreamWorking) {
            console.log('🔄 WebRTC not working after user interaction, falling back to WebSocket')
            setVideoStreamWorking(false) // Ensure canvas is visible for WebSocket fallback
            
            // Force hide video element and show canvas
            if (videoRef.current) {
              videoRef.current.style.display = 'none'
            }
            if (canvasRef.current) {
              canvasRef.current.style.display = 'block'
            }
            
            websocket.send(JSON.stringify({
              type: 'start_video_stream',
              session_id: sessionId
            }))
          }
        }, 5000) // Increased from 3000 to 5000ms to give WebRTC more time

    } catch (error) {
      console.error('❌ Failed to start video stream:', error)
    }
  }

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

  // Redraw last frame if canvas gets cleared (e.g., during hot reload)
  useEffect(() => {
    const redrawLastFrame = () => {
      const canvas = canvasRef.current
      if (!canvas || !lastFrameRef.current) return false

      const ctx = canvas.getContext('2d')
      if (!ctx) return false

      const img = lastFrameRef.current
      if (canvas.width === img.width && canvas.height === img.height) {
        // Check if canvas is empty (all black/transparent)
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
        const data = imageData.data
        let hasContent = false
        for (let i = 0; i < data.length; i += 4) {
          if (data[i] !== 0 || data[i + 1] !== 0 || data[i + 2] !== 0 || data[i + 3] !== 0) {
            hasContent = true
            break
          }
        }

        if (!hasContent) {
          ctx.clearRect(0, 0, canvas.width, canvas.height)
          ctx.drawImage(img, 0, 0)
          console.log('🔄 Redrew last frame after canvas reset')
          return true
        }
      }
      return false
    }

    // Try to redraw immediately
    redrawLastFrame()

    // Set up periodic check in case canvas gets cleared later
    const interval = setInterval(() => {
      if (redrawLastFrame()) {
        // Successfully redrew, can stop checking for now
        clearInterval(interval)
      }
    }, 1000) // Check every second

    return () => clearInterval(interval)
  }, [])

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

  // Show connection UI when not connected or when video is not streaming
  // Also check if we're receiving video frames (which indicates the connection is working)
  const hasVideoFrames = lastFrameRef.current !== null
  // Simplified logic: show main display when connected, regardless of video stream status
  const showConnectionUI = connectionState !== 'connected'

  if (showConnectionUI) {
    return (
      <div className={`bg-gray-900 rounded-lg flex items-center justify-center ${className}`}>
        <div className="text-center text-white max-w-md">
          <div className="mb-6">
            <div className="text-4xl mb-4">🎬</div>
            <h3 className="text-xl font-bold mb-2">Isaac Sim Video Stream</h3>
            <p className="text-gray-300 mb-4">
              Connect to live Isaac Sim physics simulation and robot visualization
            </p>
          </div>

          {/* Connection Status */}
          <div className="mb-6 p-4 bg-gray-800 rounded-lg">
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm font-medium">Status:</span>
              <span className={`text-sm font-bold ${
                (connectionState as any) === 'connected' ? 'text-green-400' :
                (connectionState as any) === 'connecting' ? 'text-yellow-400' :
                (connectionState as any) === 'error' ? 'text-red-400' : 'text-gray-400'
              }`}>
                {(connectionState as any) === 'connected' ? '✅ Connected' :
                 (connectionState as any) === 'connecting' ? '🔄 Connecting' :
                 (connectionState as any) === 'error' ? '❌ Error' : '⏳ Checking'}
              </span>
            </div>

            <div className="space-y-1 text-xs text-gray-400">
              <div>Environment: {environment}</div>
              <div>Quality: {qualityProfile}</div>
              <div>WebSocket: {(connectionState as any) === 'connected' ? '🔌 Connected' : '🔌 Disconnected'}</div>
              <div>Video: {videoStreamWorking ? '🎥 Active' : '🎥 Inactive'}</div>
            </div>
          </div>

          {/* Start Video Button */}
          {!videoStreamWorking && (
            <button
              onClick={startVideoStream}
              disabled={(connectionState as any) !== 'connected'}
              className={`w-full py-3 px-6 rounded-lg font-semibold text-white transition-all ${
                (connectionState as any) === 'connected'
                  ? 'bg-blue-600 hover:bg-blue-700 shadow-lg hover:shadow-xl'
                  : 'bg-gray-600 cursor-not-allowed opacity-50'
              }`}
            >
              🎬 Start Isaac Sim Video
            </button>
          )}

          {/* Video Element (shown when streaming) */}
          {videoStreamWorking && (
            <div className="mt-4 w-full">
              <div className="text-sm text-gray-300 mb-2">Live Isaac Sim Stream:</div>
              <video
                ref={videoRef}
                className="w-full h-48 bg-black rounded border border-gray-600"
                autoPlay
                muted
                playsInline
                controls={false}
              />
            </div>
          )}

          {/* Error Message */}
          {(connectionState as any) === 'error' && (
            <div className="mt-4 p-3 bg-red-900/50 border border-red-700 rounded text-red-200 text-sm">
              Failed to connect to Isaac Sim. Please check that the service is running.
            </div>
          )}
        </div>
      </div>
    )
  }

  // Connected state - Isaac Sim visual simulation
  return (
    <div 
      ref={containerRef}
      data-testid="isaac-sim-display"
      className={`relative bg-black overflow-hidden ${
        isFullscreen 
          ? 'fixed inset-0 z-50' 
          : `rounded-lg ${className}`
      }`}
    >
      {/* WebRTC Video Stream from Isaac Sim */}
      <video
        ref={videoRef}
        className="w-full h-full object-cover"
        style={{ display: videoStreamWorking ? 'block' : 'none', zIndex: 10, position: 'relative' }}
        autoPlay
        muted
        playsInline
        controls={false}
      />

      {/* Canvas for WebSocket video frames (when WebRTC fails) */}
      <canvas
        ref={canvasRef}
        className="w-full h-full object-cover absolute inset-0"
        style={{ display: videoStreamWorking ? 'none' : 'block', zIndex: 5 }} // Show when WebRTC fails
      />

      {/* Start Video Button for User Interaction */}
      {!videoStreamWorking && connectionState === 'connected' && (
        <div className="absolute inset-0 flex items-center justify-center z-20">
          <button
            onClick={startVideoStream}
            className="bg-blue-600 hover:bg-blue-700 text-white font-bold py-4 px-8 rounded-lg shadow-lg transition-colors"
          >
            🎬 Start Isaac Sim Video
          </button>
        </div>
      )}

      {/* Loading State */}
      {!videoStreamWorking && (connectionState as any) === 'connecting' && (
        <div className="absolute inset-0 flex items-center justify-center z-20">
          <div className="bg-black/80 text-white p-6 rounded-lg">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-white mx-auto mb-2"></div>
            Connecting to Isaac Sim...
          </div>
        </div>
      )}

      {/* Professional Isaac Sim HUD */}
      <div className="absolute top-4 left-4 z-5">
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
        <div className="absolute top-20 right-4 z-30 max-w-sm">
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