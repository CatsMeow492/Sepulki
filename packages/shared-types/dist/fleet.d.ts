import { BaseEntity } from './index';
import { Ingot } from './sepulka';
export declare enum FleetStatus {
    IDLE = "IDLE",
    ACTIVE = "ACTIVE",
    MAINTENANCE = "MAINTENANCE",
    ERROR = "ERROR",
    OFFLINE = "OFFLINE"
}
export declare enum RobotStatus {
    IDLE = "IDLE",
    WORKING = "WORKING",
    CHARGING = "CHARGING",
    MAINTENANCE = "MAINTENANCE",
    ERROR = "ERROR",
    OFFLINE = "OFFLINE"
}
export interface Fleet extends BaseEntity {
    name: string;
    description?: string;
    locus: Locus;
    robots: Robot[];
    activeTask?: string;
    status: FleetStatus;
    constraints: string[];
}
export interface Robot extends BaseEntity {
    name: string;
    sepulkaId: string;
    fleetId: string;
    currentIngot: Ingot;
    status: RobotStatus;
    lastSeen?: Date;
    pose?: RobotPose;
    batteryLevel?: number;
    healthScore?: number;
}
export interface Locus extends BaseEntity {
    name: string;
    description?: string;
    coordinates?: Coordinates;
    constraints: string[];
    safetyZones: SafetyZone[];
}
export interface Coordinates {
    latitude: number;
    longitude: number;
    altitude?: number;
}
export interface SafetyZone {
    id: string;
    name: string;
    type: SafetyZoneType;
    boundaries: GeoBoundary[];
    restrictions: string[];
}
export declare enum SafetyZoneType {
    EXCLUSION = "EXCLUSION",
    RESTRICTED = "RESTRICTED",
    MAINTENANCE = "MAINTENANCE",
    EMERGENCY = "EMERGENCY"
}
export interface GeoBoundary {
    type: 'polygon' | 'circle' | 'rectangle';
    coordinates: number[][];
    radius?: number;
}
export interface RobotPose {
    position: {
        x: number;
        y: number;
        z: number;
    };
    orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
    };
    jointPositions?: Record<string, number>;
    timestamp: Date;
}
