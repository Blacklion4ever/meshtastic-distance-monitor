#include "DistanceMonitorRssi.h"

#include "DistanceMonitorConfig.h"
#include "DistanceMonitorUtils.h"
#include <cmath>

void DmRssiFilter::reset()
{
    initialized_ = false;
    meanDbm_ = 0.0F;
    lastSampleDbm_ = 0.0F;
    trendDbPerSec_ = 0.0F;
    lastUpdateMs_ = 0U;
}

void DmRssiFilter::update(float rssiDbm, uint32_t nowMs)
{
    if (!std::isfinite(rssiDbm))
        return;

    if (!initialized_)
    {
        initialized_ = true;
        meanDbm_ = rssiDbm;
        lastSampleDbm_ = rssiDbm;
        lastUpdateMs_ = nowMs;
        trendDbPerSec_ = 0.0F;
        return;
    }

    const uint32_t elapsedMs = dmElapsedMs(nowMs, lastUpdateMs_);
    if (elapsedMs > 0U)
    {
        const float instantTrend =
            (rssiDbm - lastSampleDbm_) * 1000.0F /
            static_cast<float>(elapsedMs);
        trendDbPerSec_ +=
            DM_RSSI_TREND_ALPHA * (instantTrend - trendDbPerSec_);
    }

    meanDbm_ += DM_RSSI_FILTER_ALPHA * (rssiDbm - meanDbm_);
    lastSampleDbm_ = rssiDbm;
    lastUpdateMs_ = nowMs;
}

bool dmSelectBestRssi(
    bool btValid,
    float rssiBtDbm,
    bool tbValid,
    float rssiTbDbm,
    float &bestRssiDbm)
{
    if (btValid && tbValid)
    {
        bestRssiDbm = rssiBtDbm > rssiTbDbm ? rssiBtDbm : rssiTbDbm;
        return true;
    }

    if (btValid)
    {
        bestRssiDbm = rssiBtDbm;
        return true;
    }

    if (tbValid)
    {
        bestRssiDbm = rssiTbDbm;
        return true;
    }

    bestRssiDbm = 0.0F;
    return false;
}

DmDistanceBand dmDistanceBandFromRssi(float bestRssiDbm)
{
    if (!std::isfinite(bestRssiDbm))
        return DmDistanceBand::Unknown;

    if (bestRssiDbm > DM_RSSI_NEAR_THRESHOLD_DBM)
        return DmDistanceBand::Near;

    if (bestRssiDbm > DM_RSSI_MEDIUM_THRESHOLD_DBM)
        return DmDistanceBand::Medium;

    if (bestRssiDbm > DM_RSSI_WARNING_THRESHOLD_DBM)
        return DmDistanceBand::Warning;

    return DmDistanceBand::VeryFar;
}

float dmRssiAlertRatio(DmDistanceBand band)
{
    switch (band)
    {
    case DmDistanceBand::Warning:
        return DM_RSSI_WARNING_ALERT_RATIO;
    case DmDistanceBand::VeryFar:
        return DM_RSSI_VERY_FAR_ALERT_RATIO;
    case DmDistanceBand::Near:
    case DmDistanceBand::Medium:
    case DmDistanceBand::Unknown:
    default:
        return 0.0F;
    }
}
