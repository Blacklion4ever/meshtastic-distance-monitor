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

// Return the best available DOP. Zero means the corresponding DOP is absent;
// when both are zero the result is zero and accuracy becomes infinity.
uint32_t dmBestDop(uint32_t pdop, uint32_t hdop);

// Convert Meshtastic DOP hundredths into a conservative approximate accuracy.
// DOP==0 is unknown and therefore returns +infinity.
double dmAccuracyMetersFromDop(uint32_t dop);

const char *dmPositionKindName(DmPositionKind kind);
const char *dmDistanceSourceName(DmDistanceSource source);
const char *dmDistanceBandName(DmDistanceBand band);
const char *dmFaultCauseName(DmFaultCause cause);
const char *dmSosCauseName(DmSosCause cause);
