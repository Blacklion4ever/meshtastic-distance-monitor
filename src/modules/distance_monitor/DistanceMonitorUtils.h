#pragma once
#include "DistanceMonitorTypes.h"
#include <cstddef>
#include <cstdint>

void dmFormatBattery(uint8_t batteryPercent, char *buffer, size_t bufferSize);
uint32_t dmElapsedMs(uint32_t nowMs, uint32_t previousMs);
double dmCalculateDistanceMeters(
    double latitude1,
    double longitude1,
    double latitude2,
    double longitude2);
uint8_t dmComputeMaxReportIntervalSec(float maxDistanceMeters);
uint8_t dmComputeReportIntervalSec(float distanceRatio, float maxDistanceMeters);
uint32_t dmDistanceBipIntervalMs(float distanceRatio);

const char *dmPositionKindName(DmPositionKind kind);
const char *dmDistanceSourceName(DmDistanceSource source);
const char *dmDistanceBandName(DmDistanceBand band);
const char *dmFaultCauseName(DmFaultCause cause);
const char *dmSosCauseName(DmSosCause cause);
