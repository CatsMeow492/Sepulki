import { BaseEntity } from './index';
export declare enum SmithRole {
    SMITH = "SMITH",
    OVER_SMITH = "OVER_SMITH",
    ADMIN = "ADMIN"
}
export declare enum Permission {
    FORGE_SEPULKA = "FORGE_SEPULKA",
    EDIT_SEPULKA = "EDIT_SEPULKA",
    DELETE_SEPULKA = "DELETE_SEPULKA",
    CAST_INGOT = "CAST_INGOT",
    TEMPER_INGOT = "TEMPER_INGOT",
    QUENCH_TO_FLEET = "QUENCH_TO_FLEET",
    RECALL_FLEET = "RECALL_FLEET",
    EMERGENCY_STOP = "EMERGENCY_STOP",
    VIEW_FLEET = "VIEW_FLEET",
    MANAGE_FLEET = "MANAGE_FLEET",
    VIEW_ROBOTS = "VIEW_ROBOTS",
    MANAGE_ROBOTS = "MANAGE_ROBOTS",
    CREATE_TASK = "CREATE_TASK",
    ASSIGN_TASK = "ASSIGN_TASK",
    CANCEL_TASK = "CANCEL_TASK",
    VIEW_TASKS = "VIEW_TASKS",
    VIEW_CATALOG = "VIEW_CATALOG",
    MANAGE_ALLOYS = "MANAGE_ALLOYS",
    MANAGE_PATTERNS = "MANAGE_PATTERNS",
    VIEW_EDICTS = "VIEW_EDICTS",
    MANAGE_EDICTS = "MANAGE_EDICTS",
    VIEW_BELLOWS = "VIEW_BELLOWS",
    EXPORT_TELEMETRY = "EXPORT_TELEMETRY",
    MANAGE_SMITHS = "MANAGE_SMITHS",
    SYSTEM_CONFIG = "SYSTEM_CONFIG",
    AUDIT_LOGS = "AUDIT_LOGS"
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
export declare const ROLE_PERMISSIONS: Record<SmithRole, Permission[]>;
