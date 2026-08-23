#pragma once

#include <cstdint>

// Vehicle / high-speed movement detector tuning.
//
// The GNSS runs at 0.5 Hz in Distance Monitor. High-speed movement therefore
// requires several consecutive, good-quality fixes instead of reacting to a
// single ground-speed sample.
static constexpr uint8_t DM_VEHICLE_CONFIRM_SAMPLES = 3U;
static constexpr uint32_t DM_VEHICLE_CONFIRM_MAX_GAP_MS = 5000U;

// Keep the generic GPS fix acceptance unchanged. These limits are only an
// additional quality gate for the vehicle alarm.
static constexpr uint8_t DM_VEHICLE_MIN_SATS = 4U;
static constexpr float DM_VEHICLE_MAX_ACCURACY_M = 30.0F;

// Entry keeps the existing DM_HIGH_SPEED_THRESHOLD_KMH (20 km/h). A lower
// clear threshold adds hysteresis so speed jitter around 20 km/h cannot
// repeatedly arm/clear the state.
static constexpr float DM_VEHICLE_CLEAR_THRESHOLD_KMH = 15.0F;

struct DmVehicleDetectorState
{
    uint8_t consecutiveHighSpeedSamples = 0U;
    uint32_t lastHighSpeedSampleMs = 0U;
    bool hasLastHighSpeedSampleTime = false;
};
