import { BaseEntity } from './index';

export enum AlloyType {
  ACTUATOR = 'ACTUATOR',
  SENSOR = 'SENSOR', 
  CONTROLLER = 'CONTROLLER',
  END_EFFECTOR = 'END_EFFECTOR',
  CHASSIS = 'CHASSIS',
  POWER = 'POWER',
  COMMUNICATION = 'COMMUNICATION'
}

export interface Alloy extends BaseEntity {
  name: string;
  description?: string;
  type: AlloyType;
  specifications: Record<string, any>;
  meshAssets: string[];
  urdfTemplate?: string;
  compatibility: AlloyCompatibility[];
  tags: string[];
  version: string;
}

export interface AlloyCompatibility {
  alloyType: AlloyType;
  requirements: Record<string, any>;
  constraints: Record<string, any>;
}

// Specific alloy specifications
export interface ActuatorSpecs {
  torque: number; // Nm
  speed: number; // RPM
  precision: number; // degrees
  voltage: number; // V
  current: number; // A
  protocol: string; // CAN, PWM, etc
}

export interface SensorSpecs {
  type: SensorType;
  range: number[];
  accuracy: number;
  sampleRate: number; // Hz
  protocol: string;
  voltage: number;
}

export enum SensorType {
  IMU = 'IMU',
  CAMERA = 'CAMERA',
  LIDAR = 'LIDAR',
  FORCE = 'FORCE',
  PROXIMITY = 'PROXIMITY',
  TEMPERATURE = 'TEMPERATURE',
  GPS = 'GPS'
}

export interface ControllerSpecs {
  cpu: string;
  memory: number; // MB
  storage: number; // GB
  interfaces: string[];
  operatingSystem: string;
  realTimeCapable: boolean;
}

export interface EndEffectorSpecs {
  type: EndEffectorType;
  payload: number; // kg
  reach: number; // mm
  degrees_of_freedom: number;
  precision: number; // mm
  toolInterface: string;
}

export enum EndEffectorType {
  GRIPPER = 'GRIPPER',
  SUCTION = 'SUCTION',
  MAGNETIC = 'MAGNETIC',
  WELDING = 'WELDING',
  CUTTING = 'CUTTING',
  CUSTOM = 'CUSTOM'
}
