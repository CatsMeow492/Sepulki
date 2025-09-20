"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.EndEffectorType = exports.SensorType = exports.AlloyType = void 0;
var AlloyType;
(function (AlloyType) {
    AlloyType["ACTUATOR"] = "ACTUATOR";
    AlloyType["SENSOR"] = "SENSOR";
    AlloyType["CONTROLLER"] = "CONTROLLER";
    AlloyType["END_EFFECTOR"] = "END_EFFECTOR";
    AlloyType["CHASSIS"] = "CHASSIS";
    AlloyType["POWER"] = "POWER";
    AlloyType["COMMUNICATION"] = "COMMUNICATION";
})(AlloyType || (exports.AlloyType = AlloyType = {}));
var SensorType;
(function (SensorType) {
    SensorType["IMU"] = "IMU";
    SensorType["CAMERA"] = "CAMERA";
    SensorType["LIDAR"] = "LIDAR";
    SensorType["FORCE"] = "FORCE";
    SensorType["PROXIMITY"] = "PROXIMITY";
    SensorType["TEMPERATURE"] = "TEMPERATURE";
    SensorType["GPS"] = "GPS";
})(SensorType || (exports.SensorType = SensorType = {}));
var EndEffectorType;
(function (EndEffectorType) {
    EndEffectorType["GRIPPER"] = "GRIPPER";
    EndEffectorType["SUCTION"] = "SUCTION";
    EndEffectorType["MAGNETIC"] = "MAGNETIC";
    EndEffectorType["WELDING"] = "WELDING";
    EndEffectorType["CUTTING"] = "CUTTING";
    EndEffectorType["CUSTOM"] = "CUSTOM";
})(EndEffectorType || (exports.EndEffectorType = EndEffectorType = {}));
