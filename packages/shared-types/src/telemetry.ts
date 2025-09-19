import { BaseEntity } from './index';

export interface BellowsStream {
  fleetId: string;
  metrics: TelemetryMetric[];
  events: TelemetryEvent[];
  realTime: boolean;
}

export interface TelemetryMetric {
  robotId: string;
  timestamp: Date;
  metric: MetricType;
  value: number;
  unit: string;
  tags?: Record<string, string>;
}

export interface TelemetryEvent {
  id: string;
  robotId: string;
  fleetId: string;
  timestamp: Date;
  type: EventType;
  severity: EventSeverity;
  message: string;
  data?: Record<string, any>;
  acknowledged: boolean;
}

export enum MetricType {
  // Performance
  TASK_LATENCY = 'TASK_LATENCY',
  THROUGHPUT = 'THROUGHPUT',
  SUCCESS_RATE = 'SUCCESS_RATE',
  ERROR_RATE = 'ERROR_RATE',
  
  // Hardware
  BATTERY_SOC = 'BATTERY_SOC',
  BATTERY_VOLTAGE = 'BATTERY_VOLTAGE',
  BATTERY_CURRENT = 'BATTERY_CURRENT',
  MOTOR_TEMPERATURE = 'MOTOR_TEMPERATURE',
  CPU_USAGE = 'CPU_USAGE',
  MEMORY_USAGE = 'MEMORY_USAGE',
  
  // Motion
  POSE_ERROR = 'POSE_ERROR',
  VELOCITY = 'VELOCITY',
  ACCELERATION = 'ACCELERATION',
  FORCE = 'FORCE',
  TORQUE = 'TORQUE',
  
  // Communication
  SIGNAL_STRENGTH = 'SIGNAL_STRENGTH',
  PACKET_LOSS = 'PACKET_LOSS',
  LATENCY = 'LATENCY',
  
  // Environmental
  AMBIENT_TEMPERATURE = 'AMBIENT_TEMPERATURE',
  HUMIDITY = 'HUMIDITY',
  VIBRATION = 'VIBRATION',
  NOISE_LEVEL = 'NOISE_LEVEL'
}

export enum EventType {
  // System events
  ROBOT_STARTED = 'ROBOT_STARTED',
  ROBOT_STOPPED = 'ROBOT_STOPPED',
  CONNECTION_LOST = 'CONNECTION_LOST',
  CONNECTION_RESTORED = 'CONNECTION_RESTORED',
  
  // Task events
  TASK_STARTED = 'TASK_STARTED',
  TASK_COMPLETED = 'TASK_COMPLETED',
  TASK_FAILED = 'TASK_FAILED',
  TASK_CANCELLED = 'TASK_CANCELLED',
  
  // Error events
  HARDWARE_ERROR = 'HARDWARE_ERROR',
  SOFTWARE_ERROR = 'SOFTWARE_ERROR',
  COMMUNICATION_ERROR = 'COMMUNICATION_ERROR',
  SAFETY_VIOLATION = 'SAFETY_VIOLATION',
  
  // Maintenance
  MAINTENANCE_REQUIRED = 'MAINTENANCE_REQUIRED',
  CALIBRATION_NEEDED = 'CALIBRATION_NEEDED',
  FIRMWARE_UPDATE = 'FIRMWARE_UPDATE',
  
  // Policy
  POLICY_VIOLATION = 'POLICY_VIOLATION',
  CONSTRAINT_BREACH = 'CONSTRAINT_BREACH',
  
  // Custom
  CUSTOM_EVENT = 'CUSTOM_EVENT'
}

export enum EventSeverity {
  DEBUG = 'DEBUG',
  INFO = 'INFO',
  WARNING = 'WARNING',
  ERROR = 'ERROR',
  CRITICAL = 'CRITICAL'
}

export interface HealthScore {
  robotId: string;
  timestamp: Date;
  overall: number; // 0-100
  components: {
    hardware: number;
    software: number;
    communication: number;
    performance: number;
  };
  alerts: HealthAlert[];
}

export interface HealthAlert {
  component: string;
  severity: EventSeverity;
  message: string;
  recommendation?: string;
}

export interface MetricThreshold {
  metricType: MetricType;
  robotId?: string;
  fleetId?: string;
  warning: number;
  critical: number;
  unit: string;
}

export interface AlertRule {
  id: string;
  name: string;
  description?: string;
  condition: AlertCondition;
  actions: AlertAction[];
  enabled: boolean;
  cooldown: number; // seconds
}

export interface AlertCondition {
  metric: MetricType;
  operator: 'gt' | 'lt' | 'eq' | 'gte' | 'lte';
  threshold: number;
  duration?: number; // seconds
  aggregation?: 'avg' | 'max' | 'min' | 'sum';
}

export interface AlertAction {
  type: 'email' | 'webhook' | 'slack' | 'emergency_stop';
  target: string;
  template?: string;
}
