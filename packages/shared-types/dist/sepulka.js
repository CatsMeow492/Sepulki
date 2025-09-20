"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ArtifactType = exports.IngotStatus = exports.PatternCategory = exports.SepulkaStatus = void 0;
var SepulkaStatus;
(function (SepulkaStatus) {
    SepulkaStatus["FORGING"] = "FORGING";
    SepulkaStatus["CAST_READY"] = "CAST_READY";
    SepulkaStatus["CASTING"] = "CASTING";
    SepulkaStatus["CAST_FAILED"] = "CAST_FAILED";
    SepulkaStatus["READY"] = "READY";
})(SepulkaStatus || (exports.SepulkaStatus = SepulkaStatus = {}));
var PatternCategory;
(function (PatternCategory) {
    PatternCategory["INDUSTRIAL_ARM"] = "INDUSTRIAL_ARM";
    PatternCategory["MOBILE_ROBOT"] = "MOBILE_ROBOT";
    PatternCategory["HUMANOID"] = "HUMANOID";
    PatternCategory["QUADRUPED"] = "QUADRUPED";
    PatternCategory["DRONE"] = "DRONE";
    PatternCategory["CUSTOM"] = "CUSTOM";
})(PatternCategory || (exports.PatternCategory = PatternCategory = {}));
var IngotStatus;
(function (IngotStatus) {
    IngotStatus["BUILDING"] = "BUILDING";
    IngotStatus["BUILD_FAILED"] = "BUILD_FAILED";
    IngotStatus["READY"] = "READY";
    IngotStatus["TEMPERING"] = "TEMPERING";
    IngotStatus["TEMPER_FAILED"] = "TEMPER_FAILED";
    IngotStatus["TEMPERED"] = "TEMPERED";
    IngotStatus["DEPLOYED"] = "DEPLOYED";
})(IngotStatus || (exports.IngotStatus = IngotStatus = {}));
var ArtifactType;
(function (ArtifactType) {
    ArtifactType["URDF"] = "URDF";
    ArtifactType["CONTAINER"] = "CONTAINER";
    ArtifactType["FIRMWARE"] = "FIRMWARE";
    ArtifactType["CONFIG"] = "CONFIG";
    ArtifactType["MESH"] = "MESH";
})(ArtifactType || (exports.ArtifactType = ArtifactType = {}));
