#include "DistanceMonitorRssi.h"

#include "DistanceMonitorConfig.h"
#include "DistanceMonitorUtils.h"

#include <algorithm>
#include <cmath>

namespace
{
float clampStd(float value)
{
    return std::max(value, DM_RSSI_MIN_STD_DB);
}

float gaussianLikelihood(float sample, float mean, float stdDev)
{
    const float sigma = clampStd(stdDev);
    const float z = (sample - mean) / sigma;
    return std::exp(-0.5F * z * z) / sigma;
}
} // namespace

void DmRssiFilter::reset()
{
    initialized_ = false;
    sampleCount_ = 0U;
    lastUpdateMs_ = 0U;
    meanDbm_ = 0.0F;
    varianceDb2_ = 0.0F;
    trendDbPerSec_ = 0.0F;
}

void DmRssiFilter::update(float sampleDbm, uint32_t nowMs)
{
    if (!initialized_)
    {
        initialized_ = true;
        sampleCount_ = 1U;
        lastUpdateMs_ = nowMs;
        meanDbm_ = sampleDbm;
        varianceDb2_ = DM_RSSI_BOOTSTRAP_STD_DB * DM_RSSI_BOOTSTRAP_STD_DB;
        trendDbPerSec_ = 0.0F;
        return;
    }

    const uint32_t elapsedMs = dmElapsedMs(nowMs, lastUpdateMs_);
    const float previousMean = meanDbm_;
    const float delta = sampleDbm - meanDbm_;

    meanDbm_ += DM_RSSI_FILTER_ALPHA * delta;
    varianceDb2_ =
        (1.0F - DM_RSSI_FILTER_ALPHA) *
        (varianceDb2_ + DM_RSSI_FILTER_ALPHA * delta * delta);

    if (elapsedMs > 0U)
    {
        const float rawTrend =
            (meanDbm_ - previousMean) * 1000.0F / static_cast<float>(elapsedMs);
        trendDbPerSec_ +=
            DM_RSSI_TREND_ALPHA * (rawTrend - trendDbPerSec_);
    }

    lastUpdateMs_ = nowMs;
    ++sampleCount_;
}

bool DmRssiFilter::isUsable(uint32_t nowMs, uint32_t maxAgeMs) const
{
    return initialized_ &&
           sampleCount_ >= DM_RSSI_MIN_SAMPLES &&
           dmElapsedMs(nowMs, lastUpdateMs_) <= maxAgeMs;
}

float DmRssiFilter::stdDb() const
{
    return std::sqrt(std::max(varianceDb2_, 0.0F));
}

void DmOnlineStats::reset()
{
    count_ = 0U;
    mean_ = 0.0F;
    m2_ = 0.0F;
}

void DmOnlineStats::seed(float mean, float stdDev, uint32_t count)
{
    count_ = std::max<uint32_t>(count, 2U);
    mean_ = mean;
    m2_ = stdDev * stdDev * static_cast<float>(count_ - 1U);
}

void DmOnlineStats::update(float value)
{
    ++count_;
    const float delta = value - mean_;
    mean_ += delta / static_cast<float>(count_);
    const float delta2 = value - mean_;
    m2_ += delta * delta2;
}

float DmOnlineStats::stdDev() const
{
    if (count_ < 2U)
    {
        return DM_RSSI_BOOTSTRAP_STD_DB;
    }
    return std::sqrt(std::max(m2_ / static_cast<float>(count_ - 1U), 0.0F));
}

DmRssiCalibration::DmRssiCalibration()
{
    reset();
}

void DmRssiCalibration::reset()
{
    buckets_[0].seed(
        DM_RSSI_BOOTSTRAP_NEAR_MEAN_DBM,
        DM_RSSI_BOOTSTRAP_STD_DB,
        DM_RSSI_CALIBRATION_PRIOR_COUNT);
    buckets_[1].seed(
        DM_RSSI_BOOTSTRAP_MID_MEAN_DBM,
        DM_RSSI_BOOTSTRAP_STD_DB,
        DM_RSSI_CALIBRATION_PRIOR_COUNT);
    buckets_[2].seed(
        DM_RSSI_BOOTSTRAP_WARNING_MEAN_DBM,
        DM_RSSI_BOOTSTRAP_STD_DB,
        DM_RSSI_CALIBRATION_PRIOR_COUNT);
    buckets_[3].seed(
        DM_RSSI_BOOTSTRAP_BEYOND_MEAN_DBM,
        DM_RSSI_BOOTSTRAP_STD_DB,
        DM_RSSI_CALIBRATION_PRIOR_COUNT);
}

int DmRssiCalibration::bandIndex(DmDistanceBand band)
{
    switch (band)
    {
    case DmDistanceBand::Near:
        return 0;
    case DmDistanceBand::Mid:
        return 1;
    case DmDistanceBand::Warning:
        return 2;
    case DmDistanceBand::Beyond:
        return 3;
    case DmDistanceBand::Unknown:
    default:
        return -1;
    }
}

void DmRssiCalibration::update(DmDistanceBand band, float fusedRssiDbm)
{
    const int index = bandIndex(band);
    if (index >= 0)
    {
        buckets_[index].update(fusedRssiDbm);
    }
}

DmDistanceBandEstimate DmRssiCalibration::estimate(
    float fusedRssiDbm,
    float fusedTrendDbPerSec,
    bool moving) const
{
    DmDistanceBandEstimate result;
    result.fusedRssiDbm = fusedRssiDbm;
    result.fusedTrendDbPerSec = fusedTrendDbPerSec;

    const float motionConfidence = moving ? DM_RSSI_MOVING_CONFIDENCE : 1.0F;
    float sum = 0.0F;
    for (size_t index = 0U; index < 4U; ++index)
    {
        const float effectiveStd =
            clampStd(buckets_[index].stdDev()) / motionConfidence;
        result.probabilities[index] =
            gaussianLikelihood(fusedRssiDbm, buckets_[index].mean(), effectiveStd);
        sum += result.probabilities[index];
    }

    if (sum <= 0.0F)
    {
        return result;
    }

    for (float &probability : result.probabilities)
    {
        probability /= sum;
    }

    size_t bestIndex = 0U;
    size_t secondIndex = 1U;
    if (result.probabilities[secondIndex] > result.probabilities[bestIndex])
    {
        std::swap(bestIndex, secondIndex);
    }

    for (size_t index = 2U; index < 4U; ++index)
    {
        if (result.probabilities[index] > result.probabilities[bestIndex])
        {
            secondIndex = bestIndex;
            bestIndex = index;
        }
        else if (result.probabilities[index] > result.probabilities[secondIndex])
        {
            secondIndex = index;
        }
    }

    const float best = result.probabilities[bestIndex];
    const float second = result.probabilities[secondIndex];
    result.confidence = best;

    if (best < DM_RSSI_MIN_CONFIDENCE ||
        (best - second) < DM_RSSI_MIN_MARGIN)
    {
        result.band = DmDistanceBand::Unknown;
        return result;
    }

    static constexpr DmDistanceBand bands[4] = {
        DmDistanceBand::Near,
        DmDistanceBand::Mid,
        DmDistanceBand::Warning,
        DmDistanceBand::Beyond,
    };
    result.band = bands[bestIndex];
    return result;
}

float dmFuseRssi(float baseToTrackerDbm, float trackerToBaseDbm)
{
    return 0.5F * baseToTrackerDbm + 0.5F * trackerToBaseDbm;
}

float dmFuseRssiTrend(
    float baseToTrackerTrendDbPerSec,
    float trackerToBaseTrendDbPerSec)
{
    return 0.5F * baseToTrackerTrendDbPerSec +
           0.5F * trackerToBaseTrendDbPerSec;
}

DmDistanceBand dmDistanceBandFromRatio(float ratio)
{
    if (ratio < 0.30F)
    {
        return DmDistanceBand::Near;
    }
    if (ratio < 0.80F)
    {
        return DmDistanceBand::Mid;
    }
    if (ratio < 1.00F)
    {
        return DmDistanceBand::Warning;
    }
    return DmDistanceBand::Beyond;
}
