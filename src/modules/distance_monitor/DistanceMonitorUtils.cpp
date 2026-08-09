#include "DistanceMonitorUtils.h"

#include <cmath>
#include <cstdio>

void dmFormatAge(uint32_t totalSeconds, char *buffer, size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0)
    {
        return;
    }

    const uint32_t days = totalSeconds / 86400U;
    const uint32_t hours = (totalSeconds % 86400U) / 3600U;
    const uint32_t minutes = (totalSeconds % 3600U) / 60U;
    const uint32_t seconds = totalSeconds % 60U;

    if (days > 0U)
    {
        std::snprintf(buffer, bufferSize, "%lud%02luh", static_cast<unsigned long>(days), static_cast<unsigned long>(hours));
    }
    else if (hours > 0U)
    {
        std::snprintf(buffer, bufferSize, "%luh%02lum", static_cast<unsigned long>(hours), static_cast<unsigned long>(minutes));
    }
    else if (minutes > 0U)
    {
        std::snprintf(buffer, bufferSize, "%lum%02lus", static_cast<unsigned long>(minutes), static_cast<unsigned long>(seconds));
    }
    else
    {
        std::snprintf(buffer, bufferSize, "%lus", static_cast<unsigned long>(seconds));
    }
}

uint32_t dmElapsedMs(uint32_t nowMs, uint32_t previousMs)
{
    return static_cast<uint32_t>(nowMs - previousMs);
}

double dmCalculateDistanceMeters(
    double latitude1,
    double longitude1,
    double latitude2,
    double longitude2)
{
    constexpr double EARTH_RADIUS_METERS = 6371000.0;
    constexpr double DEG_TO_RAD_LOCAL = 0.01745329251994329576923690768489;

    const double latitude1Rad = latitude1 * DEG_TO_RAD_LOCAL;
    const double longitude1Rad = longitude1 * DEG_TO_RAD_LOCAL;
    const double latitude2Rad = latitude2 * DEG_TO_RAD_LOCAL;
    const double longitude2Rad = longitude2 * DEG_TO_RAD_LOCAL;

    const double deltaLatitude = latitude2Rad - latitude1Rad;
    const double deltaLongitude = longitude2Rad - longitude1Rad;

    const double sinHalfDeltaLatitude = std::sin(deltaLatitude * 0.5);
    const double sinHalfDeltaLongitude = std::sin(deltaLongitude * 0.5);

    double haversine =
        sinHalfDeltaLatitude * sinHalfDeltaLatitude +
        std::cos(latitude1Rad) * std::cos(latitude2Rad) *
            sinHalfDeltaLongitude * sinHalfDeltaLongitude;

    // Clamp rounding noise before sqrt(1 - haversine).
    if (haversine < 0.0)
    {
        haversine = 0.0;
    }
    else if (haversine > 1.0)
    {
        haversine = 1.0;
    }

    const double angularDistance =
        2.0 * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));

    return EARTH_RADIUS_METERS * angularDistance;
}

const char *dmRadioStateName(DmRadioState state)
{
    switch (state)
    {
    case DmRadioState::Alive:
        return "ALIVE";
    case DmRadioState::Stale:
        return "STALE";
    case DmRadioState::Lost:
        return "LOST";
    case DmRadioState::Unknown:
    default:
        return "UNKNOWN";
    }
}

const char *dmPositionKindName(DmPositionKind kind)
{
    switch (kind)
    {
    case DmPositionKind::FreshFix:
        return "FRESH";
    case DmPositionKind::CachedStationary:
        return "CACHED";
    case DmPositionKind::NoFix:
    default:
        return "NO_FIX";
    }
}

const char *dmDistanceStateName(DmDistanceState state)
{
    switch (state)
    {
    case DmDistanceState::Local:
        return "LOCAL";
    case DmDistanceState::Valid:
        return "VALID";
    case DmDistanceState::Grace:
        return "GRACE";
    case DmDistanceState::Unknown:
    default:
        return "UNKNOWN";
    }
}

const char *dmAlarmStateName(DmAlarmState state)
{
    switch (state)
    {
    case DmAlarmState::Safe:
        return "SAFE";
    case DmAlarmState::Warning:
        return "WARNING";
    case DmAlarmState::Alarm:
        return "ALARM";
    case DmAlarmState::TrackingLost:
        return "TRACK_LOST";
    default:
        return "UNKNOWN";
    }
}
