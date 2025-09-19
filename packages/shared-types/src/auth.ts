import { BaseEntity } from './index';

export enum SmithRole {
  SMITH = 'SMITH',
  OVER_SMITH = 'OVER_SMITH', 
  ADMIN = 'ADMIN'
}

export enum Permission {
  // Design permissions
  FORGE_SEPULKA = 'FORGE_SEPULKA',
  EDIT_SEPULKA = 'EDIT_SEPULKA',
  DELETE_SEPULKA = 'DELETE_SEPULKA',
  
  // Build permissions  
  CAST_INGOT = 'CAST_INGOT',
  TEMPER_INGOT = 'TEMPER_INGOT',
  
  // Deployment permissions
  QUENCH_TO_FLEET = 'QUENCH_TO_FLEET',
  RECALL_FLEET = 'RECALL_FLEET',
  EMERGENCY_STOP = 'EMERGENCY_STOP',
  
  // Fleet management
  VIEW_FLEET = 'VIEW_FLEET',
  MANAGE_FLEET = 'MANAGE_FLEET',
  VIEW_ROBOTS = 'VIEW_ROBOTS',
  MANAGE_ROBOTS = 'MANAGE_ROBOTS',
  
  // Task management
  CREATE_TASK = 'CREATE_TASK',
  ASSIGN_TASK = 'ASSIGN_TASK',
  CANCEL_TASK = 'CANCEL_TASK',
  VIEW_TASKS = 'VIEW_TASKS',
  
  // Catalog management
  VIEW_CATALOG = 'VIEW_CATALOG',
  MANAGE_ALLOYS = 'MANAGE_ALLOYS',
  MANAGE_PATTERNS = 'MANAGE_PATTERNS',
  
  // Policy management
  VIEW_EDICTS = 'VIEW_EDICTS',
  MANAGE_EDICTS = 'MANAGE_EDICTS',
  
  // Telemetry
  VIEW_BELLOWS = 'VIEW_BELLOWS',
  EXPORT_TELEMETRY = 'EXPORT_TELEMETRY',
  
  // Admin
  MANAGE_SMITHS = 'MANAGE_SMITHS',
  SYSTEM_CONFIG = 'SYSTEM_CONFIG',
  AUDIT_LOGS = 'AUDIT_LOGS'
}

export interface Smith extends BaseEntity {
  email: string;
  name: string;
  role: SmithRole;
  permissions: Permission[];
  isActive: boolean;
  lastLoginAt?: Date;
  preferences: SmithPreferences;
}

export interface SmithPreferences {
  theme: 'light' | 'dark' | 'auto';
  language: string;
  timezone: string;
  notifications: {
    email: boolean;
    push: boolean;
    slack?: string;
  };
  dashboard: {
    defaultView: string;
    widgets: string[];
  };
}

export interface AuthSession {
  smithId: string;
  token: string;
  refreshToken: string;
  expiresAt: Date;
  permissions: Permission[];
  role: SmithRole;
}

export interface LoginCredentials {
  email: string;
  password: string;
}

export interface LoginResponse {
  smith: Smith;
  session: AuthSession;
}

export interface RefreshTokenRequest {
  refreshToken: string;
}

// Role-based permission mapping
export const ROLE_PERMISSIONS: Record<SmithRole, Permission[]> = {
  [SmithRole.SMITH]: [
    Permission.FORGE_SEPULKA,
    Permission.EDIT_SEPULKA,
    Permission.CAST_INGOT,
    Permission.VIEW_FLEET,
    Permission.VIEW_ROBOTS,
    Permission.VIEW_TASKS,
    Permission.CREATE_TASK,
    Permission.VIEW_CATALOG,
    Permission.VIEW_BELLOWS
  ],
  [SmithRole.OVER_SMITH]: [
    // All Smith permissions plus:
    Permission.DELETE_SEPULKA,
    Permission.TEMPER_INGOT,
    Permission.QUENCH_TO_FLEET,
    Permission.RECALL_FLEET,
    Permission.MANAGE_FLEET,
    Permission.MANAGE_ROBOTS,
    Permission.ASSIGN_TASK,
    Permission.CANCEL_TASK,
    Permission.MANAGE_ALLOYS,
    Permission.MANAGE_PATTERNS,
    Permission.VIEW_EDICTS,
    Permission.EXPORT_TELEMETRY
  ],
  [SmithRole.ADMIN]: [
    // All permissions
    ...Object.values(Permission)
  ]
};
