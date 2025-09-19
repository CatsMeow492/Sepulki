// Deprecated in favor of Scene3D's internal HeroModel; keeping in case we want
// a direct static loader elsewhere (must be used inside <Canvas>)
import { useEffect } from 'react'
import { useGLTF } from '@react-three/drei'

export function StaticModel({ url, scale = 1 }: { url: string; scale?: number }) {
  const gltf = useGLTF(url)
  useEffect(() => {
    ;(gltf.scene as any).scale.setScalar(scale)
  }, [gltf, scale])
  return <primitive object={gltf.scene} />
}


