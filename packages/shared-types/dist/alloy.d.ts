import { BaseEntity } from './index';
export declare enum AlloyType {
    ACTUATOR = "ACTUATOR",
    SENSOR = "SENSOR",
    CONTROLLER = "CONTROLLER",
    END_EFFECTOR = "END_EFFECTOR",
    CHASSIS = "CHASSIS",
    POWER = "POWER",
    COMMUNICATION = "COMMUNICATION"
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
export interface ActuatorSpecs {
    torque: number;
    speed: number;
    precision: number;
    voltage: number;
    current: number;
    protocol: string;
}
export interface SensorSpecs {
    type: SensorType;
    range: number[];
    accuracy: number;
    sampleRate: number;
    protocol: string;
    voltage: number;
}
export declare enum SensorType {
    IMU = "IMU",
    CAMERA = "CAMERA",
    LIDAR = "LIDAR",
    FORCE = "FORCE",
    PROXIMITY = "PROXIMITY",
    TEMPERATURE = "TEMPERATURE",
    GPS = "GPS"
}
export interface ControllerSpecs {
    cpu: string;
    memory: number;
    storage: number;
    interfaces: string[];
    operatingSystem: string;
    realTimeCapable: boolean;
}
export interface EndEffectorSpecs {
    type: EndEffectorType;
    payload: number;
    reach: number;
    degrees_of_freedom: number;
    precision: number;
    toolInterface: string;
}
export declare enum EndEffectorType {
    GRIPPER = "GRIPPER",
    SUCTION = "SUCTION",
    MAGNETIC = "MAGNETIC",
    WELDING = "WELDING",
    CUTTING = "CUTTING",
    CUSTOM = "CUSTOM"
}
