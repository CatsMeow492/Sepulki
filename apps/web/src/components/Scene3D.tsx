import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment, ContactShadows } from '@react-three/drei'
import { useEffect, useMemo, useRef, useState } from 'react'
import type { MotionProfile, RobotSpec } from '@/types/robot'
import { specToUrdf } from '@/lib/specToUrdf'
import { RobotModel } from './RobotModel'
import { MotionPlayer } from '@/lib/motionPlayer'
import { ErrorBanner } from '@/components/ErrorBanner'

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
  playing?: boolean
  playbackRate?: number
  seekTime?: number
}

export function Scene3D({ spec, urdf, assetBaseUrl, showAxes = false, onRobotApi, onError, profile, playing = false, playbackRate = 1, seekTime }: SceneProps) {
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
  const [errorMsg, setErrorMsg] = useState<string | null>(null)

  if (!playerRef.current) playerRef.current = new MotionPlayer()
  // Load profile when it changes
  useEffect(() => {
    if (profile) {
      playerRef.current?.loadProfile(profile)
    }
  }, [profile])
  // Apply rate when it changes
  useEffect(() => {
    playerRef.current?.setRate(playbackRate)
  }, [playbackRate])

  return (
    <>
      {errorMsg && (
        <div className="absolute left-0 right-0 top-0 z-10 p-3">
          <ErrorBanner message={errorMsg} onRetry={() => {
            setErrorMsg(null)
          }} />
        </div>
      )}
      <Canvas shadows={false}>
      <color attach="background" args={['#1a1a1a']} />
      <PerspectiveCamera makeDefault position={[4, 4, 4]} fov={50} />
      <OrbitControls enablePan enableZoom enableRotate minPolarAngle={Math.PI / 4} maxPolarAngle={Math.PI / 2} />
        {/* Environment and lightweight lights for realistic look */}
        <Environment preset="warehouse" />
        <ambientLight intensity={0.6} />
        <directionalLight position={[5, 10, 5]} intensity={0.8} />

        {/* Ground plane and contact shadow for spatial context */}
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]} receiveShadow>
          <planeGeometry args={[20, 20]} />
          <meshStandardMaterial color="#2b2b2b" metalness={0} roughness={1} />
        </mesh>
        <ContactShadows position={[0, -0.01, 0]} opacity={0.5} width={10} height={10} blur={2.5} far={5} />
        {builtUrdf ? (
          <RobotModel
            urdf={builtUrdf}
            showAxes={showAxes}
            onLoaded={(api) => {
              robotApiRef.current = api
              onRobotApi?.(api)
            }}
            onError={(e) => {
              const msg = e instanceof Error ? e.message : String(e)
              setErrorMsg(msg)
              onError?.(e)
            }}
          />
        ) : null}
      {/* Minimal deterministic player loop */}
      {profile ? (
        <AnimationTicker onTick={(delta) => {
          const player = playerRef.current
          const api = robotApiRef.current
          if (!player || !api) return
          if (seekTime != null) {
            player.seek(seekTime)
          }
          if (playing) player.play()
          else player.pause()
          player.tick(delta)
          const pose = player.sample()
          Object.entries(pose).forEach(([name, value]) => api.setJointValue(name, value))
        }} />
      ) : null}
      </Canvas>
    </>
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