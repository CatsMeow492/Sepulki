"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.EventSeverity = exports.EventType = exports.MetricType = void 0;
var MetricType;
(function (MetricType) {
    // Performance
    MetricType["TASK_LATENCY"] = "TASK_LATENCY";
    MetricType["THROUGHPUT"] = "THROUGHPUT";
    MetricType["SUCCESS_RATE"] = "SUCCESS_RATE";
    MetricType["ERROR_RATE"] = "ERROR_RATE";
    // Hardware
    MetricType["BATTERY_SOC"] = "BATTERY_SOC";
    MetricType["BATTERY_VOLTAGE"] = "BATTERY_VOLTAGE";
    MetricType["BATTERY_CURRENT"] = "BATTERY_CURRENT";
    MetricType["MOTOR_TEMPERATURE"] = "MOTOR_TEMPERATURE";
    MetricType["CPU_USAGE"] = "CPU_USAGE";
    MetricType["MEMORY_USAGE"] = "MEMORY_USAGE";
    // Motion
    MetricType["POSE_ERROR"] = "POSE_ERROR";
    MetricType["VELOCITY"] = "VELOCITY";
    MetricType["ACCELERATION"] = "ACCELERATION";
    MetricType["FORCE"] = "FORCE";
    MetricType["TORQUE"] = "TORQUE";
    // Communication
    MetricType["SIGNAL_STRENGTH"] = "SIGNAL_STRENGTH";
    MetricType["PACKET_LOSS"] = "PACKET_LOSS";
    MetricType["LATENCY"] = "LATENCY";
    // Environmental
    MetricType["AMBIENT_TEMPERATURE"] = "AMBIENT_TEMPERATURE";
    MetricType["HUMIDITY"] = "HUMIDITY";
    MetricType["VIBRATION"] = "VIBRATION";
    MetricType["NOISE_LEVEL"] = "NOISE_LEVEL";
})(MetricType || (exports.MetricType = MetricType = {}));
var EventType;
(function (EventType) {
    // System events
    EventType["ROBOT_STARTED"] = "ROBOT_STARTED";
    EventType["ROBOT_STOPPED"] = "ROBOT_STOPPED";
    EventType["CONNECTION_LOST"] = "CONNECTION_LOST";
    EventType["CONNECTION_RESTORED"] = "CONNECTION_RESTORED";
    // Task events
    EventType["TASK_STARTED"] = "TASK_STARTED";
    EventType["TASK_COMPLETED"] = "TASK_COMPLETED";
    EventType["TASK_FAILED"] = "TASK_FAILED";
    EventType["TASK_CANCELLED"] = "TASK_CANCELLED";
    // Error events
    EventType["HARDWARE_ERROR"] = "HARDWARE_ERROR";
    EventType["SOFTWARE_ERROR"] = "SOFTWARE_ERROR";
    EventType["COMMUNICATION_ERROR"] = "COMMUNICATION_ERROR";
    EventType["SAFETY_VIOLATION"] = "SAFETY_VIOLATION";
    // Maintenance
    EventType["MAINTENANCE_REQUIRED"] = "MAINTENANCE_REQUIRED";
    EventType["CALIBRATION_NEEDED"] = "CALIBRATION_NEEDED";
    EventType["FIRMWARE_UPDATE"] = "FIRMWARE_UPDATE";
    // Policy
    EventType["POLICY_VIOLATION"] = "POLICY_VIOLATION";
    EventType["CONSTRAINT_BREACH"] = "CONSTRAINT_BREACH";
    // Custom
    EventType["CUSTOM_EVENT"] = "CUSTOM_EVENT";
})(EventType || (exports.EventType = EventType = {}));
var EventSeverity;
(function (EventSeverity) {
    EventSeverity["DEBUG"] = "DEBUG";
    EventSeverity["INFO"] = "INFO";
    EventSeverity["WARNING"] = "WARNING";
    EventSeverity["ERROR"] = "ERROR";
    EventSeverity["CRITICAL"] = "CRITICAL";
})(EventSeverity || (exports.EventSeverity = EventSeverity = {}));
