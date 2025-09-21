'use client'

import React, { useState } from 'react'
import { Monitor, Zap, Settings, Info } from 'lucide-react'

interface RenderModeToggleProps {
  currentMode: 'auto' | 'threejs' | 'isaac_sim'
  onModeChange: (mode: 'auto' | 'threejs' | 'isaac_sim') => void
  isaacSimAvailable: boolean
  className?: string
}

export function RenderModeToggle({
  currentMode,
  onModeChange,
  isaacSimAvailable,
  className = ''
}: RenderModeToggleProps) {
  const [showTooltip, setShowTooltip] = useState(false)

  const modes = [
    {
      id: 'auto' as const,
      label: 'Auto',
      icon: Settings,
      description: 'Automatically choose best renderer',
      available: true
    },
    {
      id: 'threejs' as const,
      label: 'Three.js',
      icon: Monitor,
      description: 'High-performance browser rendering',
      available: true
    },
    {
      id: 'isaac_sim' as const,
      label: 'Isaac Sim',
      icon: Zap,
      description: 'Photorealistic physics simulation',
      available: isaacSimAvailable
    }
  ]

  return (
    <div className={`relative ${className}`}>
      {/* Mode Toggle Buttons */}
      <div className="bg-gray-100 rounded-lg p-1 flex">
        {modes.map((mode) => {
          const Icon = mode.icon
          const isActive = currentMode === mode.id
          const isDisabled = !mode.available

          return (
            <button
              key={mode.id}
              onClick={() => !isDisabled && onModeChange(mode.id)}
              disabled={isDisabled}
              className={`
                flex items-center px-3 py-2 rounded-md text-sm font-medium transition-all
                ${isActive 
                  ? 'bg-white text-gray-900 shadow-sm' 
                  : isDisabled 
                    ? 'text-gray-400 cursor-not-allowed'
                    : 'text-gray-600 hover:text-gray-900 hover:bg-gray-50'
                }
              `}
              title={mode.description}
            >
              <Icon className="w-4 h-4 mr-1" />
              {mode.label}
              {mode.id === 'isaac_sim' && !isDisabled && (
                <span className="ml-1 px-1.5 py-0.5 bg-yellow-100 text-yellow-800 rounded text-xs">
                  PHYSICS
                </span>
              )}
              {mode.id === 'isaac_sim' && isDisabled && (
                <span className="ml-1 px-1.5 py-0.5 bg-gray-200 text-gray-500 rounded text-xs">
                  OFFLINE
                </span>
              )}
            </button>
          )
        })}
      </div>

      {/* Info Button */}
      <button
        onClick={() => setShowTooltip(!showTooltip)}
        className="absolute -top-2 -right-2 w-6 h-6 bg-blue-500 text-white rounded-full flex items-center justify-center hover:bg-blue-600 transition-colors"
        title="Rendering mode info"
      >
        <Info className="w-3 h-3" />
      </button>

      {/* Tooltip/Info Panel */}
      {showTooltip && (
        <div className="absolute top-8 right-0 w-80 bg-white rounded-lg shadow-lg border p-4 z-50">
          <div className="space-y-3">
            <div className="flex items-center justify-between">
              <h4 className="font-medium text-gray-900">Rendering Modes</h4>
              <button
                onClick={() => setShowTooltip(false)}
                className="text-gray-400 hover:text-gray-600"
              >
                ×
              </button>
            </div>

            {modes.map((mode) => {
              const Icon = mode.icon
              return (
                <div key={mode.id} className="flex items-start space-x-3">
                  <div className={`
                    p-2 rounded-lg
                    ${mode.id === 'isaac_sim' 
                      ? 'bg-yellow-100 text-yellow-700' 
                      : mode.id === 'threejs' 
                        ? 'bg-blue-100 text-blue-700'
                        : 'bg-gray-100 text-gray-700'
                    }
                  `}>
                    <Icon className="w-4 h-4" />
                  </div>
                  <div className="flex-1">
                    <div className="flex items-center space-x-2">
                      <span className="font-medium text-gray-900">{mode.label}</span>
                      {!mode.available && (
                        <span className="px-2 py-1 bg-red-100 text-red-700 rounded text-xs">
                          Unavailable
                        </span>
                      )}
                    </div>
                    <p className="text-sm text-gray-600">{mode.description}</p>
                    
                    {mode.id === 'isaac_sim' && (
                      <div className="mt-2 text-xs text-gray-500">
                        {isaacSimAvailable ? (
                          <span className="text-green-600">✓ Service available</span>
                        ) : (
                          <span className="text-red-600">✗ Service offline</span>
                        )}
                      </div>
                    )}
                  </div>
                </div>
              )
            })}

            <div className="pt-3 border-t border-gray-200">
              <p className="text-xs text-gray-500">
                <strong>Auto mode</strong> intelligently selects the best renderer based on device capabilities, 
                network conditions, and Isaac Sim service availability.
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Click outside to close tooltip */}
      {showTooltip && (
        <div 
          className="fixed inset-0 z-40" 
          onClick={() => setShowTooltip(false)}
        />
      )}
    </div>
  )
}
