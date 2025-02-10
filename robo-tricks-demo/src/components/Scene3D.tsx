import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment, Stage } from '@react-three/drei'
import { RoboticArm } from './RoboticArm'

export function Scene3D() {
  return (
    <Canvas shadows>
      <color attach="background" args={['#1a1a1a']} />
      <PerspectiveCamera makeDefault position={[4, 4, 4]} fov={50} />
      <OrbitControls 
        enablePan={true} 
        enableZoom={true} 
        enableRotate={true}
        minPolarAngle={Math.PI / 4}
        maxPolarAngle={Math.PI / 2}
      />
      
      <Stage
        intensity={0.5}
        environment="city"
        shadows={{ type: 'accumulative', color: '#d9d9d9', colorBlend: 2, opacity: 1 }}
        adjustCamera={false}
      >
        <RoboticArm />
      </Stage>
    </Canvas>
  )
} 