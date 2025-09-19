import { BaseEntity } from './index';

export enum EdictType {
  SAFETY = 'SAFETY',
  PERFORMANCE = 'PERFORMANCE', 
  COMPLIANCE = 'COMPLIANCE',
  OPERATIONAL = 'OPERATIONAL'
}

export enum EdictSeverity {
  INFO = 'INFO',
  WARNING = 'WARNING',
  CRITICAL = 'CRITICAL'
}

export interface Edict extends BaseEntity {
  name: string;
  description?: string;
  type: EdictType;
  rules: EdictRule;
  severity: EdictSeverity;
  active: boolean;
  appliesTo: PolicyTarget;
  violations: PolicyViolation[];
}

export interface EdictRule {
  condition: PolicyCondition;
  actions: PolicyAction[];
  exceptions?: PolicyException[];
}

export interface PolicyCondition {
  type: ConditionType;
  operator: 'and' | 'or';
  rules: (SimpleCondition | PolicyCondition)[];
}

export interface SimpleCondition {
  field: string;
  operator: 'eq' | 'ne' | 'gt' | 'gte' | 'lt' | 'lte' | 'in' | 'contains';
  value: any;
}

export enum ConditionType {
  LOCATION = 'LOCATION',
  TIME = 'TIME',
  ROBOT_STATE = 'ROBOT_STATE',
  TASK_TYPE = 'TASK_TYPE',
  PERFORMANCE = 'PERFORMANCE',
  HARDWARE = 'HARDWARE',
  CUSTOM = 'CUSTOM'
}

export interface PolicyAction {
  type: ActionType;
  parameters: Record<string, any>;
  immediate: boolean;
}

export enum ActionType {
  BLOCK = 'BLOCK',
  WARN = 'WARN',
  LOG = 'LOG',
  ALERT = 'ALERT',
  EMERGENCY_STOP = 'EMERGENCY_STOP',
  REDIRECT = 'REDIRECT',
  THROTTLE = 'THROTTLE',
  REQUIRE_APPROVAL = 'REQUIRE_APPROVAL'
}

export interface PolicyTarget {
  fleets?: string[];
  robots?: string[];
  locations?: string[];
  taskTypes?: string[];
  alloys?: string[];
}

export interface PolicyException {
  name: string;
  condition: PolicyCondition;
  validUntil?: Date;
  approvedBy: string; // Smith ID
}

export interface PolicyViolation extends BaseEntity {
  edictId: string;
  robotId?: string;
  fleetId?: string;
  taskId?: string;
  severity: EdictSeverity;
  message: string;
  data: Record<string, any>;
  resolved: boolean;
  resolvedBy?: string; // Smith ID
  resolvedAt?: Date;
}

export interface PolicyBreach {
  violationId: string;
  edictId: string;
  severity: EdictSeverity;
  message: string;
  timestamp: Date;
  robotId?: string;
  fleetId?: string;
  acknowledged: boolean;
}

// Safety-specific policy types
export interface SafetyZonePolicy {
  zoneId: string;
  restrictions: SafetyRestriction[];
  emergencyActions: PolicyAction[];
}

export interface SafetyRestriction {
  type: 'entry_forbidden' | 'speed_limit' | 'tool_restriction' | 'supervision_required';
  parameters: Record<string, any>;
  exceptions: string[]; // Robot IDs that are exempt
}

// Performance policy types
export interface PerformancePolicy {
  metrics: PerformanceMetric[];
  thresholds: PerformanceThreshold[];
  actions: PolicyAction[];
}

export interface PerformanceMetric {
  name: string;
  measurement: 'avg' | 'max' | 'min' | 'p95' | 'p99';
  window: number; // seconds
}

export interface PerformanceThreshold {
  metric: string;
  warning: number;
  critical: number;
  unit: string;
}

// Compliance policy types
export interface CompliancePolicy {
  standard: string; // ISO, ANSI, etc.
  requirements: ComplianceRequirement[];
  auditSchedule: AuditSchedule;
}

export interface ComplianceRequirement {
  id: string;
  description: string;
  validation: PolicyCondition;
  evidence: string[];
}

export interface AuditSchedule {
  frequency: 'daily' | 'weekly' | 'monthly' | 'quarterly' | 'annually';
  nextAudit: Date;
  responsible: string; // Smith ID
}
