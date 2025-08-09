import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { Group } from 'three'

export function RoboticArm() {
  const armRef = useRef<Group>(null)

  useFrame((state, delta) => {
    if (armRef.current) {
      // Gentle rotation animation
      armRef.current.rotation.y += delta * 0.2
    }
  })

  return (
    <group ref={armRef}>
      {/* Base */}
      <mesh position={[0, 0, 0]}>
        <cylinderGeometry args={[1, 1, 0.5, 32]} />
        <meshStandardMaterial color="#444" metalness={0.8} roughness={0.2} />
      </mesh>
      
      {/* Main arm segment */}
      <mesh position={[0, 2, 0]}>
        <boxGeometry args={[0.5, 4, 0.5]} />
        <meshStandardMaterial color="#666" metalness={0.8} roughness={0.2} />
      </mesh>
      
      {/* Forearm segment */}
      <mesh position={[0, 4, 1]}>
        <boxGeometry args={[0.4, 0.4, 2]} />
        <meshStandardMaterial color="#888" metalness={0.8} roughness={0.2} />
      </mesh>
      
      {/* End effector */}
      <mesh position={[0, 4, 2]}>
        <sphereGeometry args={[0.3, 16, 16]} />
        <meshStandardMaterial color="#aaa" metalness={0.8} roughness={0.2} />
      </mesh>
    </group>
  )
} 