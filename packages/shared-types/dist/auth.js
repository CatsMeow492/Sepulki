"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ROLE_PERMISSIONS = exports.Permission = exports.SmithRole = void 0;
var SmithRole;
(function (SmithRole) {
    SmithRole["SMITH"] = "SMITH";
    SmithRole["OVER_SMITH"] = "OVER_SMITH";
    SmithRole["ADMIN"] = "ADMIN";
})(SmithRole || (exports.SmithRole = SmithRole = {}));
var Permission;
(function (Permission) {
    // Design permissions
    Permission["FORGE_SEPULKA"] = "FORGE_SEPULKA";
    Permission["EDIT_SEPULKA"] = "EDIT_SEPULKA";
    Permission["DELETE_SEPULKA"] = "DELETE_SEPULKA";
    // Build permissions  
    Permission["CAST_INGOT"] = "CAST_INGOT";
    Permission["TEMPER_INGOT"] = "TEMPER_INGOT";
    // Deployment permissions
    Permission["QUENCH_TO_FLEET"] = "QUENCH_TO_FLEET";
    Permission["RECALL_FLEET"] = "RECALL_FLEET";
    Permission["EMERGENCY_STOP"] = "EMERGENCY_STOP";
    // Fleet management
    Permission["VIEW_FLEET"] = "VIEW_FLEET";
    Permission["MANAGE_FLEET"] = "MANAGE_FLEET";
    Permission["VIEW_ROBOTS"] = "VIEW_ROBOTS";
    Permission["MANAGE_ROBOTS"] = "MANAGE_ROBOTS";
    // Task management
    Permission["CREATE_TASK"] = "CREATE_TASK";
    Permission["ASSIGN_TASK"] = "ASSIGN_TASK";
    Permission["CANCEL_TASK"] = "CANCEL_TASK";
    Permission["VIEW_TASKS"] = "VIEW_TASKS";
    // Catalog management
    Permission["VIEW_CATALOG"] = "VIEW_CATALOG";
    Permission["MANAGE_ALLOYS"] = "MANAGE_ALLOYS";
    Permission["MANAGE_PATTERNS"] = "MANAGE_PATTERNS";
    // Policy management
    Permission["VIEW_EDICTS"] = "VIEW_EDICTS";
    Permission["MANAGE_EDICTS"] = "MANAGE_EDICTS";
    // Telemetry
    Permission["VIEW_BELLOWS"] = "VIEW_BELLOWS";
    Permission["EXPORT_TELEMETRY"] = "EXPORT_TELEMETRY";
    // Admin
    Permission["MANAGE_SMITHS"] = "MANAGE_SMITHS";
    Permission["SYSTEM_CONFIG"] = "SYSTEM_CONFIG";
    Permission["AUDIT_LOGS"] = "AUDIT_LOGS";
})(Permission || (exports.Permission = Permission = {}));
// Role-based permission mapping
exports.ROLE_PERMISSIONS = {
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
