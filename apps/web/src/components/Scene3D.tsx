import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment, useGLTF } from '@react-three/drei'
import { Box3, Vector3, Group } from 'three'
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
  heroUrl?: string // when provided, render a static GLB hero instead of URDF
}

export function Scene3D({ spec, urdf, assetBaseUrl, showAxes = false, onRobotApi, onError, profile, playing = false, playbackRate = 1, seekTime, heroUrl }: SceneProps) {
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
  const [floorOffset, setFloorOffset] = useState<number>(0)

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

        {/* Floor removed per request to avoid overlap; environment lighting retained */}
        {heroUrl ? (
          <GroundSnap>
            <HeroModel url={heroUrl} />
          </GroundSnap>
        ) : builtUrdf ? (
          <GroundSnap onFloorChange={(y)=>setFloorOffset(y)}>
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
              onBounds={(minY)=> setFloorOffset(minY)}
            />
          </GroundSnap>
        ) : null}
      {/* Minimal deterministic player loop */}
      {profile && !heroUrl ? (
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

function HeroModel({ url, scale = 1 }: { url: string; scale?: number }) {
  const gltf = useGLTF(url)
  useEffect(() => {
    const scene = gltf.scene
    // Ensure world matrices are current
    scene.updateMatrixWorld(true)
    // Auto-scale to a reasonable height and place on ground (y=0)
    const box = new Box3().setFromObject(scene)
    const size = new Vector3()
    box.getSize(size)
    const currentHeight = size.y || 1
    const targetHeight = 2.0 // meters
    const s = Math.min(scale, targetHeight / currentHeight)
    scene.scale.setScalar(s)
    // Recompute bounds after scaling, then lift so min.y = 0
    scene.updateMatrixWorld(true)
    const scaledBox = new Box3().setFromObject(scene)
    const minY = scaledBox.min.y
    scene.position.y += -minY + 0.02 // small epsilon to avoid z-fighting with ground
  }, [gltf, scale])
  return <primitive object={gltf.scene} />
}

function GroundSnap({ children, onFloorChange }: { children: React.ReactNode; onFloorChange?: (y:number)=>void }) {
  const group = useRef<Group>(null)
  useEffect(() => {
    if (!group.current) return
    // compute bounds of child content and lift to y=0
    const box = new Box3().setFromObject(group.current)
    const minY = box.min.y
    if (Number.isFinite(minY) && minY !== 0) {
      group.current.position.y += -minY + 0.02
      onFloorChange?.(minY)
    }
  }, [children])
  return <group ref={group}>{children}</group>
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