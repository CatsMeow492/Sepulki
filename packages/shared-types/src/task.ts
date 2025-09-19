import { BaseEntity } from './index';

export enum TaskStatus {
  PENDING = 'PENDING',
  ASSIGNED = 'ASSIGNED',
  IN_PROGRESS = 'IN_PROGRESS',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  CANCELLED = 'CANCELLED'
}

export enum TaskPriority {
  LOW = 'LOW',
  NORMAL = 'NORMAL',
  HIGH = 'HIGH',
  URGENT = 'URGENT'
}

export enum TaskType {
  PICK_AND_PLACE = 'PICK_AND_PLACE',
  ASSEMBLY = 'ASSEMBLY',
  INSPECTION = 'INSPECTION',
  TRANSPORT = 'TRANSPORT',
  MAINTENANCE = 'MAINTENANCE',
  PATROL = 'PATROL',
  CUSTOM = 'CUSTOM'
}

export enum RunStatus {
  PENDING = 'PENDING',
  RUNNING = 'RUNNING',
  COMPLETED = 'COMPLETED',
  FAILED = 'FAILED',
  CANCELLED = 'CANCELLED'
}

export interface Task extends BaseEntity {
  name: string;
  description?: string;
  type: TaskType;
  parameters: Record<string, any>;
  assignedRobots: string[]; // Robot IDs
  runs: Run[];
  status: TaskStatus;
  priority: TaskPriority;
  scheduledAt?: Date;
  createdBy: string; // Smith ID
}

export interface Run extends BaseEntity {
  taskId: string;
  robotId: string;
  status: RunStatus;
  startedAt?: Date;
  completedAt?: Date;
  metrics?: RunMetrics;
  logs: string[];
}

export interface RunMetrics {
  duration: number; // seconds
  energyConsumed: number; // Wh
  accuracy: number; // percentage
  errors: number;
  warnings: number;
  performance: Record<string, number>;
}

export interface TaskAssignment {
  taskId: string;
  robotId: string;
  assignedAt: Date;
  estimatedDuration: number;
  confidence: number;
}

// Specific task parameter schemas
export interface PickAndPlaceParams {
  source: {
    position: [number, number, number];
    orientation?: [number, number, number, number];
    approach: 'top' | 'side' | 'custom';
  };
  destination: {
    position: [number, number, number];
    orientation?: [number, number, number, number];
    placement: 'drop' | 'place' | 'insert';
  };
  object: {
    type: string;
    dimensions: [number, number, number];
    weight?: number;
    fragile?: boolean;
  };
  constraints: {
    maxForce?: number;
    speed?: number;
    precision?: number;
  };
}

export interface AssemblyParams {
  parts: AssemblyPart[];
  sequence: AssemblyStep[];
  tolerances: Record<string, number>;
  tools: string[];
}

export interface AssemblyPart {
  id: string;
  type: string;
  position: [number, number, number];
  orientation: [number, number, number, number];
  constraints: string[];
}

export interface AssemblyStep {
  order: number;
  action: 'pick' | 'place' | 'fasten' | 'inspect';
  partId?: string;
  tool?: string;
  parameters: Record<string, any>;
}

export interface InspectionParams {
  target: {
    type: string;
    position: [number, number, number];
    features: string[];
  };
  criteria: {
    dimensions?: {
      min: number;
      max: number;
      tolerance: number;
    };
    quality?: {
      defects: string[];
      threshold: number;
    };
    visual?: {
      patterns: string[];
      colors: string[];
    };
  };
  methods: InspectionMethod[];
}

export interface InspectionMethod {
  type: 'visual' | 'tactile' | 'measurement' | 'scan';
  sensor: string;
  parameters: Record<string, any>;
}
