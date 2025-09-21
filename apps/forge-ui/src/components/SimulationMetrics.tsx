'use client'

import React, { useEffect, useState } from 'react'
import { Activity, Zap, Wifi, Clock } from 'lucide-react'

interface SimulationMetricsProps {
  service: 'isaac_sim' | 'three_js'
  isConnected: boolean
  sessionId?: string
  className?: string
}

interface Metrics {
  fps: number
  latency: number
  bandwidth: number
  physics_fps: number
  gpu_utilization: number
  memory_usage: number
  uptime: number
}

export function SimulationMetrics({ 
  service, 
  isConnected, 
  sessionId, 
  className = '' 
}: SimulationMetricsProps) {
  const [metrics, setMetrics] = useState<Metrics>({
    fps: 0,
    latency: 0,
    bandwidth: 0,
    physics_fps: 0,
    gpu_utilization: 0,
    memory_usage: 0,
    uptime: 0
  })

  const [isExpanded, setIsExpanded] = useState(false)

  useEffect(() => {
    if (!isConnected || !sessionId) return

    const updateMetrics = () => {
      if (service === 'isaac_sim') {
        // Real Isaac Sim metrics would come from WebSocket
        setMetrics({
          fps: 58 + Math.random() * 4, // 58-62 fps
          latency: 150 + Math.random() * 50, // 150-200ms
          bandwidth: 4.5 + Math.random() * 1, // 4.5-5.5 Mbps
          physics_fps: 238 + Math.random() * 4, // 238-242 fps (240Hz target)
          gpu_utilization: 65 + Math.random() * 20, // 65-85%
          memory_usage: 2.1 + Math.random() * 0.8, // 2.1-2.9 GB
          uptime: Date.now() / 1000 // Seconds since epoch
        })
      } else {
        // Three.js metrics - simpler
        setMetrics({
          fps: 59 + Math.random() * 2, // 59-61 fps
          latency: 16, // ~16ms frame time
          bandwidth: 0.1, // Minimal bandwidth
          physics_fps: 0, // No physics in Three.js mode
          gpu_utilization: 25 + Math.random() * 15, // 25-40%
          memory_usage: 0.8 + Math.random() * 0.4, // 0.8-1.2 GB
          uptime: Date.now() / 1000
        })
      }
    }

    updateMetrics()
    const interval = setInterval(updateMetrics, 1000)

    return () => clearInterval(interval)
  }, [isConnected, sessionId, service])

  const formatUptime = (seconds: number) => {
    const mins = Math.floor((Date.now() / 1000 - seconds + 3600) / 60) // Mock start time
    if (mins < 60) return `${mins}m`
    const hours = Math.floor(mins / 60)
    return `${hours}h ${mins % 60}m`
  }

  if (!isConnected) {
    return (
      <div className={`bg-gray-50 rounded-lg p-4 ${className}`}>
        <div className="text-center text-gray-500">
          <Activity className="w-6 h-6 mx-auto mb-2 opacity-50" />
          <p className="text-sm">Not connected</p>
        </div>
      </div>
    )
  }

  return (
    <div className={`bg-white rounded-lg shadow-sm border ${className}`}>
      <div 
        className="p-4 cursor-pointer"
        onClick={() => setIsExpanded(!isExpanded)}
      >
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Activity className="w-4 h-4 text-green-500" />
            <span className="font-medium text-gray-900">Performance</span>
            <span className="px-2 py-1 bg-green-100 text-green-800 rounded text-xs">
              {metrics.fps.toFixed(1)} FPS
            </span>
          </div>
          <button className="text-gray-400 hover:text-gray-600">
            {isExpanded ? '−' : '+'}
          </button>
        </div>

        {/* Always visible key metrics */}
        <div className="grid grid-cols-3 gap-4 mt-3 text-sm">
          <div>
            <div className="text-gray-500">Latency</div>
            <div className="font-medium">
              {service === 'isaac_sim' ? `${metrics.latency.toFixed(0)}ms` : `${metrics.latency}ms`}
            </div>
          </div>
          <div>
            <div className="text-gray-500">Service</div>
            <div className="font-medium">
              {service === 'isaac_sim' ? 'Isaac Sim' : 'Three.js'}
            </div>
          </div>
          <div>
            <div className="text-gray-500">Uptime</div>
            <div className="font-medium">{formatUptime(metrics.uptime)}</div>
          </div>
        </div>
      </div>

      {/* Expanded detailed metrics */}
      {isExpanded && (
        <div className="px-4 pb-4 border-t border-gray-200">
          <div className="grid grid-cols-2 gap-4 mt-4">
            {service === 'isaac_sim' && (
              <>
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Zap className="w-4 h-4 text-yellow-500 mr-2" />
                    <span className="text-sm font-medium">Physics Engine</span>
                  </div>
                  <div className="text-sm text-gray-600">
                    <div>Physics FPS: {metrics.physics_fps.toFixed(1)}</div>
                    <div>GPU Usage: {metrics.gpu_utilization.toFixed(1)}%</div>
                  </div>
                </div>

                <div className="space-y-2">
                  <div className="flex items-center">
                    <Wifi className="w-4 h-4 text-blue-500 mr-2" />
                    <span className="text-sm font-medium">Streaming</span>
                  </div>
                  <div className="text-sm text-gray-600">
                    <div>Bandwidth: {metrics.bandwidth.toFixed(1)} Mbps</div>
                    <div>Memory: {metrics.memory_usage.toFixed(1)} GB</div>
                  </div>
                </div>
              </>
            )}

            {service === 'three_js' && (
              <>
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Activity className="w-4 h-4 text-blue-500 mr-2" />
                    <span className="text-sm font-medium">Rendering</span>
                  </div>
                  <div className="text-sm text-gray-600">
                    <div>Frame Time: ~{metrics.latency}ms</div>
                    <div>GPU Usage: {metrics.gpu_utilization.toFixed(1)}%</div>
                  </div>
                </div>

                <div className="space-y-2">
                  <div className="flex items-center">
                    <Clock className="w-4 h-4 text-green-500 mr-2" />
                    <span className="text-sm font-medium">Resources</span>
                  </div>
                  <div className="text-sm text-gray-600">
                    <div>Memory: {metrics.memory_usage.toFixed(1)} GB</div>
                    <div>Mode: Client-side</div>
                  </div>
                </div>
              </>
            )}
          </div>

          {/* Performance status bar */}
          <div className="mt-4">
            <div className="flex items-center justify-between text-sm mb-1">
              <span className="text-gray-500">Performance</span>
              <span className={`font-medium ${
                metrics.fps > 50 ? 'text-green-600' : 
                metrics.fps > 30 ? 'text-yellow-600' : 'text-red-600'
              }`}>
                {metrics.fps > 50 ? 'Excellent' : 
                 metrics.fps > 30 ? 'Good' : 'Poor'}
              </span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div 
                className={`h-2 rounded-full transition-all duration-300 ${
                  metrics.fps > 50 ? 'bg-green-500' : 
                  metrics.fps > 30 ? 'bg-yellow-500' : 'bg-red-500'
                }`}
                style={{ width: `${Math.min(metrics.fps / 60 * 100, 100)}%` }}
              />
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
