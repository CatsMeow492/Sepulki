'use client'

import React, { useState } from 'react'
import { Settings, Zap, Monitor, Wifi, WifiOff, Camera, Gamepad2 } from 'lucide-react'

interface IsaacSimControlsProps {
  isConnected: boolean
  service: 'isaac_sim' | 'three_js'
  quality: 'demo' | 'engineering' | 'certification'
  onQualityChange: (quality: string) => void
  onTogglePhysics: () => void
  onResetCamera: () => void
  onToggleCollaboration: () => void
  physicsEnabled: boolean
  collaborationEnabled: boolean
  className?: string
}

export function IsaacSimControls({
  isConnected,
  service,
  quality,
  onQualityChange,
  onTogglePhysics,
  onResetCamera,
  onToggleCollaboration,
  physicsEnabled,
  collaborationEnabled,
  className = ''
}: IsaacSimControlsProps) {
  const [showAdvanced, setShowAdvanced] = useState(false)

  const qualityOptions = [
    { value: 'demo', label: 'Demo (720p)', description: 'Fast preview quality' },
    { value: 'engineering', label: 'Engineering (1080p)', description: 'Professional quality' },
    { value: 'certification', label: 'Certification (4K)', description: 'Maximum fidelity' }
  ]

  const getServiceIcon = () => {
    if (!isConnected) return <WifiOff className="w-4 h-4 text-red-500" />
    return service === 'isaac_sim' ? 
      <Zap className="w-4 h-4 text-yellow-500" /> : 
      <Monitor className="w-4 h-4 text-blue-500" />
  }

  const getServiceStatus = () => {
    if (!isConnected) return 'Disconnected'
    return service === 'isaac_sim' ? 'Isaac Sim' : 'Three.js'
  }

  return (
    <div className={`bg-white rounded-lg shadow-sm border p-4 ${className}`}>
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center space-x-2">
          {getServiceIcon()}
          <span className="font-medium text-gray-900">{getServiceStatus()}</span>
          {service === 'isaac_sim' && physicsEnabled && (
            <span className="px-2 py-1 bg-yellow-100 text-yellow-800 rounded text-xs font-medium">
              PHYSICS
            </span>
          )}
        </div>
        
        <button
          onClick={() => setShowAdvanced(!showAdvanced)}
          className="p-1 rounded hover:bg-gray-100"
          title="Advanced settings"
        >
          <Settings className="w-4 h-4 text-gray-500" />
        </button>
      </div>

      {/* Quality Selector */}
      <div className="mb-4">
        <label className="block text-sm font-medium text-gray-700 mb-2">
          Rendering Quality
        </label>
        <select
          value={quality}
          onChange={(e) => onQualityChange(e.target.value)}
          className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
          disabled={service !== 'isaac_sim'}
        >
          {qualityOptions.map((option) => (
            <option key={option.value} value={option.value}>
              {option.label}
            </option>
          ))}
        </select>
        <p className="text-xs text-gray-500 mt-1">
          {qualityOptions.find(o => o.value === quality)?.description}
        </p>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-2 mb-4">
        <button
          onClick={onResetCamera}
          className="flex items-center justify-center px-3 py-2 bg-gray-100 hover:bg-gray-200 rounded-md text-sm font-medium transition-colors"
        >
          <Camera className="w-4 h-4 mr-1" />
          Reset View
        </button>
        
        <button
          onClick={onTogglePhysics}
          disabled={service !== 'isaac_sim'}
          className={`flex items-center justify-center px-3 py-2 rounded-md text-sm font-medium transition-colors ${
            physicsEnabled 
              ? 'bg-yellow-100 text-yellow-800 hover:bg-yellow-200' 
              : 'bg-gray-100 hover:bg-gray-200 text-gray-700'
          } disabled:opacity-50 disabled:cursor-not-allowed`}
        >
          <Zap className="w-4 h-4 mr-1" />
          Physics
        </button>
      </div>

      {/* Advanced Settings */}
      {showAdvanced && (
        <div className="pt-4 border-t border-gray-200">
          <h4 className="text-sm font-medium text-gray-900 mb-3">Advanced Settings</h4>
          
          <div className="space-y-3">
            <label className="flex items-center">
              <input
                type="checkbox"
                checked={collaborationEnabled}
                onChange={onToggleCollaboration}
                disabled={service !== 'isaac_sim'}
                className="rounded border-gray-300 text-blue-600 focus:ring-blue-500 disabled:opacity-50"
              />
              <span className="ml-2 text-sm text-gray-700">Enable Collaboration</span>
            </label>

            {service === 'isaac_sim' && (
              <div className="bg-green-50 p-3 rounded-md">
                <div className="flex items-center">
                  <Wifi className="w-4 h-4 text-green-600 mr-2" />
                  <span className="text-sm text-green-800 font-medium">
                    Isaac Sim Connected
                  </span>
                </div>
                <p className="text-xs text-green-700 mt-1">
                  Real-time physics simulation and photorealistic rendering active
                </p>
              </div>
            )}

            {service === 'three_js' && (
              <div className="bg-blue-50 p-3 rounded-md">
                <div className="flex items-center">
                  <Monitor className="w-4 h-4 text-blue-600 mr-2" />
                  <span className="text-sm text-blue-800 font-medium">
                    Three.js Fallback
                  </span>
                </div>
                <p className="text-xs text-blue-700 mt-1">
                  High-performance 3D rendering with basic physics simulation
                </p>
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  )
}
