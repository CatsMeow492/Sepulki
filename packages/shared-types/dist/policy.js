"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ActionType = exports.ConditionType = exports.EdictSeverity = exports.EdictType = void 0;
var EdictType;
(function (EdictType) {
    EdictType["SAFETY"] = "SAFETY";
    EdictType["PERFORMANCE"] = "PERFORMANCE";
    EdictType["COMPLIANCE"] = "COMPLIANCE";
    EdictType["OPERATIONAL"] = "OPERATIONAL";
})(EdictType || (exports.EdictType = EdictType = {}));
var EdictSeverity;
(function (EdictSeverity) {
    EdictSeverity["INFO"] = "INFO";
    EdictSeverity["WARNING"] = "WARNING";
    EdictSeverity["CRITICAL"] = "CRITICAL";
})(EdictSeverity || (exports.EdictSeverity = EdictSeverity = {}));
var ConditionType;
(function (ConditionType) {
    ConditionType["LOCATION"] = "LOCATION";
    ConditionType["TIME"] = "TIME";
    ConditionType["ROBOT_STATE"] = "ROBOT_STATE";
    ConditionType["TASK_TYPE"] = "TASK_TYPE";
    ConditionType["PERFORMANCE"] = "PERFORMANCE";
    ConditionType["HARDWARE"] = "HARDWARE";
    ConditionType["CUSTOM"] = "CUSTOM";
})(ConditionType || (exports.ConditionType = ConditionType = {}));
var ActionType;
(function (ActionType) {
    ActionType["BLOCK"] = "BLOCK";
    ActionType["WARN"] = "WARN";
    ActionType["LOG"] = "LOG";
    ActionType["ALERT"] = "ALERT";
    ActionType["EMERGENCY_STOP"] = "EMERGENCY_STOP";
    ActionType["REDIRECT"] = "REDIRECT";
    ActionType["THROTTLE"] = "THROTTLE";
    ActionType["REQUIRE_APPROVAL"] = "REQUIRE_APPROVAL";
})(ActionType || (exports.ActionType = ActionType = {}));
