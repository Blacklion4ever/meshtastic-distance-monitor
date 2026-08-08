#pragma once

#include "DistanceMonitorTypes.h"

#include <cstddef>
#include <cstdint>

/**
 * Format a Unix timestamp as a compact UTC string.
 *
 * @param timestamp Unix epoch timestamp in seconds. Zero means unavailable.
 * @param buffer Destination character buffer.
 * @param bufferSize Size of the destination buffer.
 */
void dmFormatTimestampUtc(uint32_t timestamp, char *buffer, size_t bufferSize);

/**
 * Format a duration in a compact human-readable form suitable for summary logs.
 *
 * Examples: "12s", "1m05s", "2h03m", "1d02h".
 *
 * @param totalSeconds Duration in seconds.
 * @param buffer Destination character buffer.
 * @param bufferSize Size of the destination buffer.
 */
void dmFormatAge(uint32_t totalSeconds, char *buffer, size_t bufferSize);

/**
 * Return elapsed milliseconds using unsigned subtraction so millis() rollover
 * remains safe as long as the measured interval is below half the uint32 range.
 *
 * @param nowMs Current monotonic millisecond counter.
 * @param previousMs Previous monotonic millisecond counter.
 * @return Elapsed milliseconds.
 */
uint32_t dmElapsedMs(uint32_t nowMs, uint32_t previousMs);

/**
 * Calculate great-circle distance between two WGS84 latitude/longitude pairs.
 *
 * @return Distance in meters using the Haversine approximation.
 */
double dmCalculateDistanceMeters(
    double latitude1,
    double longitude1,
    double latitude2,
    double longitude2);

/** Return a stable printable name for a radio state. */
const char *dmRadioStateName(DmRadioState state);

/** Return a stable printable name for a position state. */
const char *dmPositionStateName(DmPositionState state);

/** Return a stable printable name for a distance state. */
const char *dmDistanceStateName(DmDistanceState state);

/** Return a stable printable name for an alarm state. */
const char *dmAlarmStateName(DmAlarmState state);
