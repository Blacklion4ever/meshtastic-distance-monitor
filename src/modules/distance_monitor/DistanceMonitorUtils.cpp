#include "DistanceMonitorUtils.h"

#include "DistanceMonitorConfig.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
float interpolate(
    float value,
    float x0,
    float y0,
    float x1,
    float y1)
{
    if (value <= x0)
    {
        return y0;
    }
    if (value >= x1)
    {
        return y1;
    }

    const float t = (value - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}

uint8_t quantizeOddDown(float seconds)
{
    int value = static_cast<int>(std::floor(seconds));
    if ((value & 1) == 0)
    {
        --value;
    }

    value = std::max<int>(value, DM_MIN_REPORT_INTERVAL_S);
    value = std::min<int>(value, DM_MAX_REPORT_INTERVAL_CAP_S);
    return static_cast<uint8_t>(value);
}
}

void dmFormatAge(uint32_t totalSeconds, char *buffer, size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0U)
    {
        return;
    }

    const uint32_t days = totalSeconds / 86400U;
    const uint32_t hours = (totalSeconds % 86400U) / 3600U;
    const uint32_t minutes = (totalSeconds % 3600U) / 60U;
    const uint32_t seconds = totalSeconds % 60U;

    if (days > 0U)
    {
        std::snprintf(
            buffer,
            bufferSize,
            "%lud%02luh",
            static_cast<unsigned long>(days),
            static_cast<unsigned long>(hours));
    }
    else if (hours > 0U)
    {
        std::snprintf(
            buffer,
            bufferSize,
            "%luh%02lum",
            static_cast<unsigned long>(hours),
            static_cast<unsigned long>(minutes));
    }
    else if (minutes > 0U)
    {
        std::snprintf(
            buffer,
            bufferSize,
            "%lum%02lus",
            static_cast<unsigned long>(minutes),
            static_cast<unsigned long>(seconds));
    }
    else
    {
        std::snprintf(
            buffer,
            bufferSize,
            "%lus",
            static_cast<unsigned long>(seconds));
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

    haversine = std::max(0.0, std::min(1.0, haversine));
    const double angularDistance =
        2.0 * std::atan2(
                  std::sqrt(haversine),
                  std::sqrt(1.0 - haversine));

    return EARTH_RADIUS_METERS * angularDistance;
}

uint8_t dmComputeMaxReportIntervalSec(float maxDistanceMeters)
{
    const float denominator =
        DM_DESIGN_RELATIVE_SPEED_MPS *
        static_cast<float>(DM_MIN_SAMPLES_PER_DMAX);

    if (maxDistanceMeters <= 0.0F || denominator <= 0.0F)
    {
        return DM_MIN_REPORT_INTERVAL_S;
    }

    const float rawSeconds = maxDistanceMeters / denominator;
    const float limitedSeconds =
        std::min<float>(rawSeconds, DM_MAX_REPORT_INTERVAL_CAP_S);

    if (limitedSeconds <= static_cast<float>(DM_MIN_REPORT_INTERVAL_S))
    {
        return DM_MIN_REPORT_INTERVAL_S;
    }

    return quantizeOddDown(limitedSeconds);
}

uint8_t dmComputeReportIntervalSec(
    float distanceRatio,
    float maxDistanceMeters)
{
    const uint8_t maximum = dmComputeMaxReportIntervalSec(maxDistanceMeters);
    const float ratio = std::max(0.0F, std::min(1.0F, distanceRatio));
    const float seconds =
        static_cast<float>(maximum) -
        ratio * static_cast<float>(maximum - DM_MIN_REPORT_INTERVAL_S);

    return quantizeOddDown(seconds);
}

uint32_t dmDistanceBipIntervalMs(float distanceRatio)
{
    if (distanceRatio < 0.80F)
    {
        return 0U;
    }

    float seconds = 1.0F;
    if (distanceRatio < 0.90F)
    {
        seconds = interpolate(distanceRatio, 0.80F, 30.0F, 0.90F, 22.5F);
    }
    else if (distanceRatio < 1.00F)
    {
        seconds = interpolate(distanceRatio, 0.90F, 22.5F, 1.00F, 3.0F);
    }
    else if (distanceRatio < 1.50F)
    {
        seconds = interpolate(distanceRatio, 1.00F, 3.0F, 1.50F, 2.0F);
    }
    else if (distanceRatio < 2.00F)
    {
        seconds = interpolate(distanceRatio, 1.50F, 2.0F, 2.00F, 1.0F);
    }

    return static_cast<uint32_t>(seconds * 1000.0F + 0.5F);
}

const char *dmRadioStateName(DmRadioState state)
{
    switch (state)
    {
    case DmRadioState::Pairing:
        return "PAIRING";
    case DmRadioState::Alive:
        return "ALIVE";
    case DmRadioState::Suspected:
        return "SUSPECTED";
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
        return "CACHED_STATIONARY";
    case DmPositionKind::NoFix:
    default:
        return "NO_FIX";
    }
}

const char *dmDistanceSourceName(DmDistanceSource source)
{
    switch (source)
    {
    case DmDistanceSource::Gps:
        return "GPS";
    case DmDistanceSource::Rssi:
        return "RSSI";
    case DmDistanceSource::None:
    default:
        return "NONE";
    }
}

const char *dmDistanceBandName(DmDistanceBand band)
{
    switch (band)
    {
    case DmDistanceBand::Near:
        return "NEAR";
    case DmDistanceBand::Mid:
        return "MID";
    case DmDistanceBand::Warning:
        return "WARNING";
    case DmDistanceBand::Beyond:
        return "BEYOND";
    case DmDistanceBand::Unknown:
    default:
        return "UNKNOWN";
    }
}

const char *dmFaultCauseName(DmFaultCause cause)
{
    switch (cause)
    {
    case DmFaultCause::RadioLowBattery:
        return "RADIO_LOW_BATTERY";
    case DmFaultCause::RadioRangeLoss:
        return "RADIO_RANGE_LOSS";
    case DmFaultCause::RadioComSaturation:
        return "RADIO_COM_SATURATION";
    case DmFaultCause::RadioUnexpectedLoss:
        return "RADIO_UNEXPECTED_LOSS";
    case DmFaultCause::RemoteShutdown:
        return "REMOTE_SHUTDOWN";
    case DmFaultCause::DistanceEstimateUnavailable:
        return "DISTANCE_ESTIMATE_UNAVAILABLE";
    case DmFaultCause::None:
    default:
        return "NONE";
    }
}

const char *dmSosCauseName(DmSosCause cause)
{
    switch (cause)
    {
    case DmSosCause::FallDetected:
        return "FALL_DETECTED";
    case DmSosCause::HighSpeedMovement:
        return "HIGH_SPEED_MOVEMENT";
    case DmSosCause::ManualButton:
    default:
        return "MANUAL_BUTTON";
    }
}

const char *dmMessageTypeName(DmMessageType type)
{
    switch (type)
    {
    case DmMessageType::AliveRequest:
        return "ALIVE_REQUEST";
    case DmMessageType::AliveResponse:
        return "ALIVE_RESPONSE";
    case DmMessageType::PairConfirm:
        return "PAIR_CONFIRM";
    case DmMessageType::PositionReport:
        return "POSITION_REPORT";
    case DmMessageType::SetPositionInterval:
        return "SET_POSITION_INTERVAL";
    case DmMessageType::BaseBeacon:
        return "BASE_BEACON";
    case DmMessageType::Sos:
        return "SOS";
    case DmMessageType::SosAck:
        return "SOS_ACK";
    case DmMessageType::Notification:
        return "NOTIFICATION";
    case DmMessageType::NotificationAck:
        return "NOTIFICATION_ACK";
    case DmMessageType::ShutdownNotice:
        return "SHUTDOWN_NOTICE";
    case DmMessageType::SearchStart:
        return "SEARCH_START";
    case DmMessageType::SearchBeacon:
        return "SEARCH_BEACON";
    case DmMessageType::SearchStop:
        return "SEARCH_STOP";
    default:
        return "UNKNOWN";
    }
}
