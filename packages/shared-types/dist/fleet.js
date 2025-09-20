"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.SafetyZoneType = exports.RobotStatus = exports.FleetStatus = void 0;
var FleetStatus;
(function (FleetStatus) {
    FleetStatus["IDLE"] = "IDLE";
    FleetStatus["ACTIVE"] = "ACTIVE";
    FleetStatus["MAINTENANCE"] = "MAINTENANCE";
    FleetStatus["ERROR"] = "ERROR";
    FleetStatus["OFFLINE"] = "OFFLINE";
})(FleetStatus || (exports.FleetStatus = FleetStatus = {}));
var RobotStatus;
(function (RobotStatus) {
    RobotStatus["IDLE"] = "IDLE";
    RobotStatus["WORKING"] = "WORKING";
    RobotStatus["CHARGING"] = "CHARGING";
    RobotStatus["MAINTENANCE"] = "MAINTENANCE";
    RobotStatus["ERROR"] = "ERROR";
    RobotStatus["OFFLINE"] = "OFFLINE";
})(RobotStatus || (exports.RobotStatus = RobotStatus = {}));
var SafetyZoneType;
(function (SafetyZoneType) {
    SafetyZoneType["EXCLUSION"] = "EXCLUSION";
    SafetyZoneType["RESTRICTED"] = "RESTRICTED";
    SafetyZoneType["MAINTENANCE"] = "MAINTENANCE";
    SafetyZoneType["EMERGENCY"] = "EMERGENCY";
})(SafetyZoneType || (exports.SafetyZoneType = SafetyZoneType = {}));
