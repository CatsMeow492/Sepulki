export type JointType = 'revolute' | 'prismatic' | 'fixed'

export interface JointLimit {
  lower: number
  upper: number
  velocity?: number
}

export interface LinkMeshRef {
  uri: string
  scale?: [number, number, number]
}

export interface JointSpec {
  name: string
  type: JointType
  parent: string
  child: string
  axis?: [number, number, number]
  origin?: { xyz?: [number, number, number]; rpy?: [number, number, number] }
  limit?: JointLimit
}

export interface LinkSpec {
  name: string
  mesh?: LinkMeshRef
}

export interface RobotSpec {
  name: string
  links: LinkSpec[]
  joints: JointSpec[]
}

export interface JointPose {
  [jointName: string]: number
}

export interface MotionKeyframe {
  t: number
  pose: JointPose
}

export interface MotionProfile {
  name: string
  duration: number
  frames: MotionKeyframe[]
  loop?: boolean
}

export interface LoadOptions {
  assetBaseUrl?: string
}


