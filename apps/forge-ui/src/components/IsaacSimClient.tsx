'use client'

import React, { useEffect, useRef, useState, useCallback } from 'react'
import { Loader2, Wifi, WifiOff, Settings, Play, Pause, RotateCcw } from 'lucide-react'

interface IsaacSimClientProps {
  sessionId: string
  userId?: string
  qualityProfile?: 'demo' | 'engineering' | 'certification'
  onJointControl?: (jointStates: Record<string, number>) => void
  onCameraControl?: (camera: { position: [number, number, number]; target: [number, number, number]; fov: number }) => void
  onError?: (error: Error | string) => void
  onConnectionChange?: (connected: boolean) => void
  className?: string
}

interface StreamMetrics {
  fps: number
  bitrate: number
  latency: number
  frameDrops: number
  bandwidth: number
}

export function IsaacSimClient({
  sessionId,
  userId = 'anonymous',
  qualityProfile = 'engineering',
  onJointControl,
  onCameraControl,
  onError,
  onConnectionChange,
  className = ''
}: IsaacSimClientProps) {
  const videoRef = useRef<HTMLVideoElement>(null)
  const websocketRef = useRef<WebSocket | null>(null)
  const peerConnectionRef = useRef<RTCPeerConnection | null>(null)
  const [connectionState, setConnectionState] = useState<'disconnected' | 'connecting' | 'connected' | 'error'>('disconnected')
  const [streamMetrics, setStreamMetrics] = useState<StreamMetrics>({ fps: 0, bitrate: 0, latency: 0, frameDrops: 0, bandwidth: 0 })
  const [isPlaying, setIsPlaying] = useState(false)
  const [showControls, setShowControls] = useState(false)

  // WebRTC configuration
  const rtcConfig: RTCConfiguration = {
    iceServers: [
      { urls: 'stun:stun.l.google.com:19302' },
      { urls: 'stun:stun1.l.google.com:19302' }
    ]
  }

  const connect = useCallback(async () => {
    try {
      setConnectionState('connecting')
      
      // Connect to WebSocket signaling server
      const wsUrl = `ws://localhost:8001`
      const websocket = new WebSocket(wsUrl)
      websocketRef.current = websocket

      websocket.onopen = async () => {
        console.log('🔗 WebSocket connected')
        
        // Send join session message
        websocket.send(JSON.stringify({
          type: 'join_session',
          user_id: userId,
          session_id: sessionId,
          quality_profile: qualityProfile
        }))
      }

      websocket.onmessage = async (event) => {
        const data = JSON.parse(event.data)
        await handleWebSocketMessage(data)
      }

      websocket.onclose = () => {
        console.log('🔌 WebSocket disconnected')
        setConnectionState('disconnected')
        onConnectionChange?.(false)
      }

      websocket.onerror = (error) => {
        console.error('❌ WebSocket error:', error)
        setConnectionState('error')
        onError?.(new Error('WebSocket connection failed'))
      }

    } catch (error) {
      console.error('❌ Connection failed:', error)
      setConnectionState('error')
      onError?.(error as Error)
    }
  }, [sessionId, userId, qualityProfile, onError, onConnectionChange])

  const handleWebSocketMessage = async (data: any) => {
    switch (data.type) {
      case 'connection_established':
        console.log('✅ Connection established:', data)
        await initializeWebRTC()
        break

      case 'offer':
        await handleOffer(data.sdp)
        break

      case 'answer':
        await handleAnswer(data.sdp)
        break

      case 'ice_candidate':
        await handleIceCandidate(data.candidate)
        break

      case 'quality_updated':
        console.log('🎥 Quality updated:', data.quality_profile)
        break

      case 'metrics_update':
        setStreamMetrics(data.metrics)
        break

      default:
        console.log('📨 Unknown message:', data.type)
    }
  }

  const initializeWebRTC = async () => {
    try {
      // Create RTCPeerConnection
      const peerConnection = new RTCPeerConnection(rtcConfig)
      peerConnectionRef.current = peerConnection

      // Handle incoming stream
      peerConnection.ontrack = (event) => {
        console.log('🎥 Received video stream')
        if (videoRef.current) {
          videoRef.current.srcObject = event.streams[0]
          setIsPlaying(true)
          setConnectionState('connected')
          onConnectionChange?.(true)
        }
      }

      // Handle ICE candidates
      peerConnection.onicecandidate = (event) => {
        if (event.candidate && websocketRef.current) {
          websocketRef.current.send(JSON.stringify({
            type: 'ice_candidate',
            candidate: event.candidate
          }))
        }
      }

      // Create offer to start connection
      const offer = await peerConnection.createOffer()
      await peerConnection.setLocalDescription(offer)

      // Send offer to server
      if (websocketRef.current) {
        websocketRef.current.send(JSON.stringify({
          type: 'offer',
          sdp: offer.sdp
        }))
      }

    } catch (error) {
      console.error('❌ WebRTC initialization failed:', error)
      onError?.(error as Error)
    }
  }

  const handleOffer = async (sdp: string) => {
    try {
      const peerConnection = peerConnectionRef.current
      if (!peerConnection) return

      await peerConnection.setRemoteDescription({ sdp, type: 'offer' })
      const answer = await peerConnection.createAnswer()
      await peerConnection.setLocalDescription(answer)

      if (websocketRef.current) {
        websocketRef.current.send(JSON.stringify({
          type: 'answer',
          sdp: answer.sdp
        }))
      }
    } catch (error) {
      console.error('❌ Failed to handle offer:', error)
    }
  }

  const handleAnswer = async (sdp: string) => {
    try {
      const peerConnection = peerConnectionRef.current
      if (!peerConnection) return

      await peerConnection.setRemoteDescription({ sdp, type: 'answer' })
    } catch (error) {
      console.error('❌ Failed to handle answer:', error)
    }
  }

  const handleIceCandidate = async (candidate: RTCIceCandidateInit) => {
    try {
      const peerConnection = peerConnectionRef.current
      if (!peerConnection) return

      await peerConnection.addIceCandidate(candidate)
    } catch (error) {
      console.error('❌ Failed to add ICE candidate:', error)
    }
  }

  const disconnect = useCallback(() => {
    // Close WebRTC connection
    if (peerConnectionRef.current) {
      peerConnectionRef.current.close()
      peerConnectionRef.current = null
    }

    // Close WebSocket
    if (websocketRef.current) {
      websocketRef.current.close()
      websocketRef.current = null
    }

    setConnectionState('disconnected')
    setIsPlaying(false)
    onConnectionChange?.(false)
  }, [onConnectionChange])

  const updateJointStates = useCallback((jointStates: Record<string, number>) => {
    if (websocketRef.current && connectionState === 'connected') {
      websocketRef.current.send(JSON.stringify({
        type: 'joint_control',
        joint_states: jointStates
      }))
    }
    onJointControl?.(jointStates)
  }, [connectionState, onJointControl])

  const updateCamera = useCallback((camera: { position: [number, number, number]; target: [number, number, number]; fov: number }) => {
    if (websocketRef.current && connectionState === 'connected') {
      websocketRef.current.send(JSON.stringify({
        type: 'camera_control',
        position: camera.position,
        target: camera.target,
        fov: camera.fov
      }))
    }
    onCameraControl?.(camera)
  }, [connectionState, onCameraControl])

  const requestKeyframe = useCallback(() => {
    if (websocketRef.current && connectionState === 'connected') {
      websocketRef.current.send(JSON.stringify({
        type: 'request_keyframe'
      }))
    }
  }, [connectionState])

  // Auto-connect on mount
  useEffect(() => {
    connect()
    return () => disconnect()
  }, [connect, disconnect])

  // Connection state indicator
  const getConnectionIcon = () => {
    switch (connectionState) {
      case 'connected':
        return <Wifi className="w-4 h-4 text-green-500" />
      case 'connecting':
        return <Loader2 className="w-4 h-4 text-yellow-500 animate-spin" />
      case 'error':
        return <WifiOff className="w-4 h-4 text-red-500" />
      default:
        return <WifiOff className="w-4 h-4 text-gray-500" />
    }
  }

  const getConnectionStatus = () => {
    switch (connectionState) {
      case 'connected':
        return 'Connected to Isaac Sim'
      case 'connecting':
        return 'Connecting to Isaac Sim...'
      case 'error':
        return 'Connection failed'
      default:
        return 'Disconnected'
    }
  }

  return (
    <div className={`relative bg-black rounded-lg overflow-hidden ${className}`}>
      {/* Video Stream */}
      <video
        ref={videoRef}
        className="w-full h-full object-contain"
        autoPlay
        playsInline
        muted
        onPlay={() => setIsPlaying(true)}
        onPause={() => setIsPlaying(false)}
      />

      {/* Loading overlay */}
      {connectionState === 'connecting' && (
        <div className="absolute inset-0 bg-gray-900/80 flex items-center justify-center">
          <div className="text-center text-white">
            <Loader2 className="w-12 h-12 animate-spin mx-auto mb-4" />
            <div className="text-lg font-medium">Initializing Isaac Sim</div>
            <div className="text-sm text-gray-300">Connecting to simulation...</div>
          </div>
        </div>
      )}

      {/* Error overlay */}
      {connectionState === 'error' && (
        <div className="absolute inset-0 bg-red-900/80 flex items-center justify-center">
          <div className="text-center text-white">
            <WifiOff className="w-12 h-12 mx-auto mb-4" />
            <div className="text-lg font-medium">Connection Failed</div>
            <div className="text-sm text-gray-300 mb-4">Unable to connect to Isaac Sim</div>
            <button
              onClick={connect}
              className="px-4 py-2 bg-red-600 hover:bg-red-700 rounded-md text-sm font-medium transition-colors"
            >
              Retry Connection
            </button>
          </div>
        </div>
      )}

      {/* Controls overlay */}
      <div 
        className={`absolute top-4 right-4 transition-opacity ${showControls ? 'opacity-100' : 'opacity-0 hover:opacity-100'}`}
        onMouseEnter={() => setShowControls(true)}
        onMouseLeave={() => setShowControls(false)}
      >
        <div className="bg-black/70 backdrop-blur-sm rounded-lg p-3 space-y-2">
          {/* Connection status */}
          <div className="flex items-center space-x-2 text-white text-sm">
            {getConnectionIcon()}
            <span>{getConnectionStatus()}</span>
          </div>

          {/* Stream metrics */}
          {connectionState === 'connected' && (
            <div className="text-xs text-gray-300 space-y-1">
              <div>FPS: {streamMetrics.fps.toFixed(1)}</div>
              <div>Latency: {streamMetrics.latency.toFixed(0)}ms</div>
              <div>Quality: {qualityProfile}</div>
            </div>
          )}

          {/* Control buttons */}
          <div className="flex space-x-2">
            <button
              onClick={requestKeyframe}
              disabled={connectionState !== 'connected'}
              className="p-2 bg-gray-700 hover:bg-gray-600 disabled:opacity-50 rounded text-white transition-colors"
              title="Request keyframe"
            >
              <RotateCcw className="w-4 h-4" />
            </button>

            <button
              onClick={() => setShowControls(!showControls)}
              className="p-2 bg-gray-700 hover:bg-gray-600 rounded text-white transition-colors"
              title="Settings"
            >
              <Settings className="w-4 h-4" />
            </button>
          </div>
        </div>
      </div>

      {/* Quality indicator */}
      {connectionState === 'connected' && (
        <div className="absolute bottom-4 left-4">
          <div className="bg-black/70 backdrop-blur-sm rounded-md px-3 py-1 text-white text-sm">
            Isaac Sim • {qualityProfile} • {streamMetrics.fps.toFixed(1)} FPS
          </div>
        </div>
      )}
    </div>
  )

  // Expose methods for parent component
  React.useImperativeHandle(React.forwardRef(() => null), () => ({
    updateJointStates,
    updateCamera,
    requestKeyframe,
    connect,
    disconnect,
    isConnected: connectionState === 'connected'
  }))
}
