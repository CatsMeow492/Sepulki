import { useState, useEffect, useCallback } from 'react'

export interface ConnectionStatus {
  status: 'disconnected' | 'connecting' | 'connected' | 'error'
  service: 'isaac_sim' | 'three_js'
  quality: 'demo' | 'engineering' | 'certification'
  metrics: {
    fps: number
    latency: number
    bandwidth: number
  }
}

export function useIsaacSimConnection() {
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>({
    status: 'disconnected',
    service: 'three_js',
    quality: 'engineering',
    metrics: { fps: 0, latency: 0, bandwidth: 0 }
  })

  const checkServiceHealth = useCallback(async () => {
    try {
      const response = await fetch('http://localhost:8002/health', {
        method: 'GET',
        timeout: 3000
      } as any)
      return response.ok
    } catch (error) {
      return false
    }
  }, [])

  const connectToIsaacSim = useCallback(async (sessionId: string) => {
    setConnectionStatus(prev => ({ ...prev, status: 'connecting' }))
    
    const isHealthy = await checkServiceHealth()
    
    if (isHealthy) {
      // Isaac Sim available - attempt WebRTC connection
      setConnectionStatus(prev => ({ 
        ...prev, 
        status: 'connected',
        service: 'isaac_sim'
      }))
      return true
    } else {
      // Fallback to Three.js
      setConnectionStatus(prev => ({ 
        ...prev, 
        status: 'connected',
        service: 'three_js'
      }))
      return false
    }
  }, [checkServiceHealth])

  const updateMetrics = useCallback((metrics: Partial<ConnectionStatus['metrics']>) => {
    setConnectionStatus(prev => ({
      ...prev,
      metrics: { ...prev.metrics, ...metrics }
    }))
  }, [])

  return {
    connectionStatus,
    connectToIsaacSim,
    updateMetrics,
    checkServiceHealth
  }
}
