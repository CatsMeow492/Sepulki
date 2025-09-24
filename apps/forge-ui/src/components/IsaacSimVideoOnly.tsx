'use client'

import React, { useRef, useEffect, useState } from 'react'

interface IsaacSimVideoOnlyProps {
  className?: string
}

export default function IsaacSimVideoOnly({ className = '' }: IsaacSimVideoOnlyProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const websocketRef = useRef<WebSocket | null>(null)
  const [isConnected, setIsConnected] = useState(false)
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected' | 'error'>('disconnected')
  const [frameCount, setFrameCount] = useState(0)

  useEffect(() => {
    // Connect to Isaac Sim WebSocket
    const connectWebSocket = () => {
      try {
        const ws = new WebSocket('ws://localhost:8765')
        websocketRef.current = ws

        ws.onopen = () => {
          console.log('✅ WebSocket connected to Isaac Sim')
          setIsConnected(true)
          setConnectionStatus('connected')
          
          // Request video stream
          ws.send(JSON.stringify({
            type: 'start_video_stream',
            session_id: 'minimal-test'
          }))
        }

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data)
            
            if (data.type === 'video_frame' && data.frame_data) {
              // Draw frame to canvas
              drawFrameToCanvas(data.frame_data)
              setFrameCount(prev => prev + 1)
            }
          } catch (error) {
            console.error('❌ Error parsing WebSocket message:', error)
          }
        }

        ws.onclose = () => {
          console.log('🔌 WebSocket disconnected')
          setIsConnected(false)
          setConnectionStatus('disconnected')
        }

        ws.onerror = (error) => {
          console.error('❌ WebSocket error:', error)
          setConnectionStatus('error')
        }

      } catch (error) {
        console.error('❌ Failed to create WebSocket connection:', error)
        setConnectionStatus('error')
      }
    }

    // Connect on mount
    connectWebSocket()

    // Cleanup on unmount
    return () => {
      if (websocketRef.current) {
        websocketRef.current.close()
      }
    }
  }, [])

  const drawFrameToCanvas = (base64Data: string) => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const img = new Image()
    img.onload = () => {
      // Set canvas size to match image
      canvas.width = img.width
      canvas.height = img.height
      
      // Draw image to canvas
      ctx.drawImage(img, 0, 0)
      
      console.log(`📹 Frame ${frameCount + 1} drawn: ${img.width}x${img.height}`)
    }
    
    img.onerror = (error) => {
      console.error('❌ Error loading frame image:', error)
    }
    
    img.src = `data:image/jpeg;base64,${base64Data}`
  }

  const handleReconnect = () => {
    if (websocketRef.current) {
      websocketRef.current.close()
    }
    
    setConnectionStatus('connecting')
    setIsConnected(false)
    setFrameCount(0)
    
    // Reconnect after a short delay
    setTimeout(() => {
      const ws = new WebSocket('ws://localhost:8765')
      websocketRef.current = ws

      ws.onopen = () => {
        console.log('✅ WebSocket reconnected')
        setIsConnected(true)
        setConnectionStatus('connected')
        
        ws.send(JSON.stringify({
          type: 'start_video_stream',
          session_id: 'minimal-test'
        }))
      }

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data)
          if (data.type === 'video_frame' && data.frame_data) {
            drawFrameToCanvas(data.frame_data)
            setFrameCount(prev => prev + 1)
          }
        } catch (error) {
          console.error('❌ Error parsing WebSocket message:', error)
        }
      }

      ws.onclose = () => {
        setConnectionStatus('disconnected')
        setIsConnected(false)
      }

      ws.onerror = () => {
        setConnectionStatus('error')
      }
    }, 1000)
  }

  return (
    <div className={`flex flex-col items-center justify-center p-4 ${className}`}>
      <div className="mb-4 flex items-center gap-4">
        <div className="text-sm font-medium">
          Isaac Sim Video Stream (Minimal)
        </div>
        <div className={`px-2 py-1 rounded text-xs font-medium ${
          connectionStatus === 'connected' ? 'bg-green-100 text-green-800' :
          connectionStatus === 'connecting' ? 'bg-yellow-100 text-yellow-800' :
          connectionStatus === 'error' ? 'bg-red-100 text-red-800' :
          'bg-gray-100 text-gray-800'
        }`}>
          {connectionStatus === 'connected' ? '✅ Connected' :
           connectionStatus === 'connecting' ? '🔄 Connecting' :
           connectionStatus === 'error' ? '❌ Error' : '⏸️ Disconnected'}
        </div>
        <div className="text-xs text-gray-600">
          Frames: {frameCount}
        </div>
        {connectionStatus !== 'connected' && (
          <button
            onClick={handleReconnect}
            className="px-3 py-1 bg-blue-500 text-white rounded text-xs hover:bg-blue-600"
          >
            Reconnect
          </button>
        )}
      </div>

      <div className="relative bg-black border border-gray-300 rounded-lg overflow-hidden">
        <canvas
          ref={canvasRef}
          className="block max-w-full max-h-[600px]"
          style={{ minWidth: '400px', minHeight: '300px' }}
        />
        
        {!isConnected && (
          <div className="absolute inset-0 bg-gray-900 bg-opacity-75 flex items-center justify-center">
            <div className="text-center text-white">
              <div className="text-lg font-medium mb-2">
                {connectionStatus === 'connecting' ? 'Connecting to Isaac Sim...' :
                 connectionStatus === 'error' ? 'Connection Error' :
                 'Not Connected'}
              </div>
              <div className="text-sm text-gray-300">
                {connectionStatus === 'connecting' ? 'Please wait...' :
                 connectionStatus === 'error' ? 'Failed to connect to Isaac Sim WebSocket' :
                 'Click Reconnect to start'}
              </div>
            </div>
          </div>
        )}
      </div>

      <div className="mt-4 text-xs text-gray-500 text-center">
              <div>WebSocket: ws://localhost:8765 (via SSH tunnel)</div>
        <div>Streaming from Isaac Sim container</div>
      </div>
    </div>
  )
}
