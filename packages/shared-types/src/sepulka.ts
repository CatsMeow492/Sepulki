import { BaseEntity } from './index';
import { Alloy } from './alloy';

export enum SepulkaStatus {
  FORGING = 'FORGING',
  CAST_READY = 'CAST_READY', 
  CASTING = 'CASTING',
  CAST_FAILED = 'CAST_FAILED',
  READY = 'READY'
}

export interface Sepulka extends BaseEntity {
  name: string;
  description?: string;
  version: string;
  pattern?: Pattern;
  alloys: Alloy[];
  ingots: Ingot[];
  status: SepulkaStatus;
  createdBy: string; // Smith ID
}

export interface Pattern extends BaseEntity {
  name: string;
  description?: string;
  parameters: Record<string, any>;
  defaults: Record<string, any>;
  template: string;
  category: PatternCategory;
  tags: string[];
}

export enum PatternCategory {
  INDUSTRIAL_ARM = 'INDUSTRIAL_ARM',
  MOBILE_ROBOT = 'MOBILE_ROBOT',
  HUMANOID = 'HUMANOID',
  QUADRUPED = 'QUADRUPED',
  DRONE = 'DRONE',
  CUSTOM = 'CUSTOM'
}

export enum IngotStatus {
  BUILDING = 'BUILDING',
  BUILD_FAILED = 'BUILD_FAILED',
  READY = 'READY',
  TEMPERING = 'TEMPERING',
  TEMPER_FAILED = 'TEMPER_FAILED',
  TEMPERED = 'TEMPERED',
  DEPLOYED = 'DEPLOYED'
}

export interface Ingot extends BaseEntity {
  sepulkaId: string;
  version: string;
  buildHash: string;
  artifacts: BuildArtifact[];
  status: IngotStatus;
  buildLogs: string[];
  tempered: boolean;
  temperResults?: TemperResults;
}

export interface BuildArtifact {
  type: ArtifactType;
  path: string;
  size: number;
  checksum: string;
}

export enum ArtifactType {
  URDF = 'URDF',
  CONTAINER = 'CONTAINER',
  FIRMWARE = 'FIRMWARE',
  CONFIG = 'CONFIG',
  MESH = 'MESH'
}

export interface TemperResults {
  originalMetrics: PerformanceMetrics;
  optimizedMetrics: PerformanceMetrics;
  improvements: Record<string, number>;
  appliedOptimizations: string[];
}

export interface PerformanceMetrics {
  latency: number;
  throughput: number;
  powerConsumption: number;
  accuracy: number;
  cost: number;
}
