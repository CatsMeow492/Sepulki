import { BaseEntity } from './index';
import { Sepulka, Ingot } from './sepulka';

export enum FleetStatus {
  IDLE = 'IDLE',
  ACTIVE = 'ACTIVE',
  MAINTENANCE = 'MAINTENANCE',
  ERROR = 'ERROR',
  OFFLINE = 'OFFLINE'
}

export enum RobotStatus {
  IDLE = 'IDLE',
  WORKING = 'WORKING',
  CHARGING = 'CHARGING',
  MAINTENANCE = 'MAINTENANCE',
  ERROR = 'ERROR',
  OFFLINE = 'OFFLINE'
}

export interface Fleet extends BaseEntity {
  name: string;
  description?: string;
  locus: Locus;
  robots: Robot[];
  activeTask?: string; // Task ID
  status: FleetStatus;
  constraints: string[]; // Edict IDs
}

export interface Robot extends BaseEntity {
  name: string;
  sepulkaId: string;
  fleetId: string;
  currentIngot: Ingot;
  status: RobotStatus;
  lastSeen?: Date;
  pose?: RobotPose;
  batteryLevel?: number; // 0-100%
  healthScore?: number; // 0-100%
}

export interface Locus extends BaseEntity {
  name: string;
  description?: string;
  coordinates?: Coordinates;
  constraints: string[]; // Edict IDs
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

export enum SafetyZoneType {
  EXCLUSION = 'EXCLUSION',
  RESTRICTED = 'RESTRICTED',
  MAINTENANCE = 'MAINTENANCE',
  EMERGENCY = 'EMERGENCY'
}

export interface GeoBoundary {
  type: 'polygon' | 'circle' | 'rectangle';
  coordinates: number[][];
  radius?: number; // for circles
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
    w: number; // quaternion
  };
  jointPositions?: Record<string, number>;
  timestamp: Date;
}
