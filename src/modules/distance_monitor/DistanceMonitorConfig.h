#pragma once

/**
 * Compile-time role of the distance monitor firmware.
 */
enum class DistanceMonitorRole : unsigned char {
    Base = 1,
    Tracker = 2,
};

#ifndef DISTANCE_MONITOR_ROLE
#define DISTANCE_MONITOR_ROLE 1
#endif

#ifndef DISTANCE_MONITOR_PROTOCOL_VERSION
#define DISTANCE_MONITOR_PROTOCOL_VERSION 1
#endif

#ifndef DISTANCE_MONITOR_FIRMWARE_VERSION
#define DISTANCE_MONITOR_FIRMWARE_VERSION "0.1.0-dev"
#endif

#ifndef DISTANCE_MONITOR_TICK_INTERVAL_MS
#define DISTANCE_MONITOR_TICK_INTERVAL_MS 1000
#endif

static_assert(
    DISTANCE_MONITOR_ROLE == static_cast<int>(DistanceMonitorRole::Base) ||
        DISTANCE_MONITOR_ROLE == static_cast<int>(DistanceMonitorRole::Tracker),
    "DISTANCE_MONITOR_ROLE must be 1 (Base) or 2 (Tracker)");