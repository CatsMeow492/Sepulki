import { useEffect, useMemo, useRef, useState } from 'react'
import { Group, Mesh, MeshBasicMaterial, BoxGeometry } from 'three'
import { useThree } from '@react-three/fiber'
import URDFLoader from 'urdf-loader'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

type JointMeta = { name: string; limit?: { lower: number; upper: number } }

type RobotApi = {
  getJoint: (name: string) => any
  setJointValue: (name: string, value: number) => void
  getJointValue: (name: string) => number
  listJoints: () => JointMeta[]
}

export function RobotModel({ urdf, onLoaded, onError, onBounds, showAxes = false }: {
  urdf: string | URL
  onLoaded?: (api: RobotApi) => void
  onError?: (error: Error | string) => void
  onBounds?: (minY: number) => void
  showAxes?: boolean
}) {
  const groupRef = useRef<Group>(null)
  const [loader] = useState(() => new URDFLoader())
  const [robot, setRobot] = useState<any>(null)
  const { scene } = useThree()

  useEffect(() => {
    // Provide a basic mesh loader so URDF can resolve OBJ assets when used
    ;(loader as any).loadMeshCb = (
      path: string,
      manager: any,
      onComplete: (obj: any) => void,
    ) => {
      const lower = path.toLowerCase()
      if (lower.endsWith('.glb') || lower.endsWith('.gltf')) {
        const gltfLoader = new GLTFLoader(manager as any)
        gltfLoader.load(
          path,
          (gltf) => onComplete(gltf.scene || new Group()),
          undefined,
          () => onComplete(new Group()),
        )
        return
      }
      const objLoader = new OBJLoader(manager as any)
      objLoader.load(
        path,
        (obj) => onComplete(obj),
        undefined,
        () => onComplete(new Group()),
      )
    }

    // Memoized loader and stringified URL for stability
    loader.load(
      typeof urdf === 'string' ? urdf : urdf.toString(),
      (result: any) => {
        setRobot(result)
      },
      undefined,
      (e) => {
        // On error, attach a simple placeholder
        const placeholder = new Group()
        const mesh = new Mesh(new BoxGeometry(0.3, 0.3, 0.3), new MeshBasicMaterial({ color: 'red' }))
        placeholder.add(mesh)
        if (groupRef.current) groupRef.current.add(placeholder)
        onError?.(e instanceof Error ? e : 'ASSET_NOT_FOUND')
      },
    )
  }, [loader, urdf])

  const api: RobotApi = useMemo(() => ({
    getJoint: (name: string) => robot?.joints?.[name],
    setJointValue: (name: string, value: number) => {
      const j = robot?.joints?.[name]
      if (j) j.setJointValue?.(value)
    },
    getJointValue: (name: string) => {
      const j = robot?.joints?.[name]
      // urdf-loader joints often expose .jointValue or .angle
      return (j?.jointValue ?? j?.angle ?? 0) as number
    },
    listJoints: () => {
      if (!robot?.joints) return []
      return Object.keys(robot.joints).map((name) => {
        const j = robot.joints[name]
        const limit = j?.limit
        return {
          name,
          limit: limit && typeof limit.lower === 'number' && typeof limit.upper === 'number'
            ? { lower: limit.lower, upper: limit.upper }
            : undefined,
        }
      })
    },
  }), [robot])

  useEffect(() => {
    if (robot && groupRef.current) {
      groupRef.current.add(robot)
      // Notify once when robot is first added
      onLoaded?.(api)
      // Compute bounds and report to parent
      try {
        const box = new (require('three').Box3)().setFromObject(groupRef.current)
        if (box && isFinite(box.min.y)) onBounds?.(box.min.y)
      } catch {}
    }
    return () => {
      if (robot && groupRef.current) {
        groupRef.current.remove(robot)
      }
    }
  }, [robot])

  useEffect(() => {
    if (!robot) return
    robot.showAxes = showAxes
    scene.updateMatrixWorld(true)
  }, [robot, showAxes, scene])

  return <group ref={groupRef} />
}


