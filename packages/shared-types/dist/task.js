"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.RunStatus = exports.TaskType = exports.TaskPriority = exports.TaskStatus = void 0;
var TaskStatus;
(function (TaskStatus) {
    TaskStatus["PENDING"] = "PENDING";
    TaskStatus["ASSIGNED"] = "ASSIGNED";
    TaskStatus["IN_PROGRESS"] = "IN_PROGRESS";
    TaskStatus["COMPLETED"] = "COMPLETED";
    TaskStatus["FAILED"] = "FAILED";
    TaskStatus["CANCELLED"] = "CANCELLED";
})(TaskStatus || (exports.TaskStatus = TaskStatus = {}));
var TaskPriority;
(function (TaskPriority) {
    TaskPriority["LOW"] = "LOW";
    TaskPriority["NORMAL"] = "NORMAL";
    TaskPriority["HIGH"] = "HIGH";
    TaskPriority["URGENT"] = "URGENT";
})(TaskPriority || (exports.TaskPriority = TaskPriority = {}));
var TaskType;
(function (TaskType) {
    TaskType["PICK_AND_PLACE"] = "PICK_AND_PLACE";
    TaskType["ASSEMBLY"] = "ASSEMBLY";
    TaskType["INSPECTION"] = "INSPECTION";
    TaskType["TRANSPORT"] = "TRANSPORT";
    TaskType["MAINTENANCE"] = "MAINTENANCE";
    TaskType["PATROL"] = "PATROL";
    TaskType["CUSTOM"] = "CUSTOM";
})(TaskType || (exports.TaskType = TaskType = {}));
var RunStatus;
(function (RunStatus) {
    RunStatus["PENDING"] = "PENDING";
    RunStatus["RUNNING"] = "RUNNING";
    RunStatus["COMPLETED"] = "COMPLETED";
    RunStatus["FAILED"] = "FAILED";
    RunStatus["CANCELLED"] = "CANCELLED";
})(RunStatus || (exports.RunStatus = RunStatus = {}));
