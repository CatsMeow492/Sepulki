import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Stage } from '@react-three/drei'
import { useMemo, useRef, useState } from 'react'
import type { MotionProfile, RobotSpec } from '@/types/robot'
import { specToUrdf } from '@/lib/specToUrdf'
import { RobotModel } from './RobotModel'
import { MotionPlayer } from '@/lib/motionPlayer'

type SceneProps = {
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
}

export function Scene3D({ spec, urdf, assetBaseUrl, showAxes = false, onRobotApi, onError, profile }: SceneProps) {
  const builtUrdf = useMemo(() => {
    if (urdf) return typeof urdf === 'string' ? urdf : urdf.toString()
    if (spec) return specToUrdf(spec, assetBaseUrl)
    return ''
  }, [spec, urdf, assetBaseUrl])

  // Minimal playback: sample profile on each rAF and set joints via API
  const playerRef = useRef<MotionPlayer | null>(null)
  const robotApiRef = useRef<{
    getJoint: (name: string) => any
    setJointValue: (name: string, value: number) => void
    getJointValue: (name: string) => number
    listJoints: () => { name: string; limit?: { lower: number; upper: number } }[]
  } | null>(null)

  if (!playerRef.current) playerRef.current = new MotionPlayer()
  if (profile) playerRef.current.loadProfile(profile)

  return (
    <Canvas shadows>
      <color attach="background" args={['#1a1a1a']} />
      <PerspectiveCamera makeDefault position={[4, 4, 4]} fov={50} />
      <OrbitControls enablePan enableZoom enableRotate minPolarAngle={Math.PI / 4} maxPolarAngle={Math.PI / 2} />
      <Stage intensity={0.5} environment="city" shadows={{ type: 'accumulative', color: '#d9d9d9', colorBlend: 2, opacity: 1 }} adjustCamera={false}>
        {builtUrdf ? (
          <RobotModel
            urdf={builtUrdf}
            showAxes={showAxes}
            onLoaded={(api) => {
              robotApiRef.current = api
              onRobotApi?.(api)
            }}
            onError={(e) => onError?.(e)}
          />
        ) : null}
      </Stage>
      {/* Minimal deterministic player loop */}
      {profile ? (
        <AnimationTicker onTick={(delta) => {
          const player = playerRef.current
          const api = robotApiRef.current
          if (!player || !api) return
          player.play()
          player.tick(delta)
          const pose = player.sample()
          Object.entries(pose).forEach(([name, value]) => api.setJointValue(name, value))
        }} />
      ) : null}
    </Canvas>
  )
}

function AnimationTicker({ onTick }: { onTick: (deltaSeconds: number) => void }) {
  const lastRef = useRef<number | null>(null)
  const cbRef = useRef(onTick)
  cbRef.current = onTick
  useEffect(() => {
    let raf = 0
    const loop = (t: number) => {
      if (lastRef.current != null) {
        const delta = (t - lastRef.current) / 1000
        cbRef.current(delta)
      }
      lastRef.current = t
      raf = requestAnimationFrame(loop)
    }
    raf = requestAnimationFrame(loop)
    return () => cancelAnimationFrame(raf)
  }, [])
  return null
}