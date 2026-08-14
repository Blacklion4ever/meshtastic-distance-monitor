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

float binCenter(const DmRssiCalibrationBin &bin)
{
    if (bin.maxDistanceM >= DM_RSSI_OPEN_BIN_MAX_M)
        return bin.minDistanceM + 60.0F;

    return 0.5F * (bin.minDistanceM + bin.maxDistanceM);
}
} // namespace

void DmRssiFilter::reset()
{
    initialized_ = false;
    sampleCount_ = 0U;
    lastUpdateMs_ = 0U;
    meanDbm_ = 0.0F;
    lastSampleDbm_ = 0.0F;
    varianceDb2_ = 0.0F;
    trendDbPerSec_ = 0.0F;
}

void DmRssiFilter::update(float sampleDbm, uint32_t nowMs)
{
    lastSampleDbm_ = sampleDbm;
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

void DmOnlineStats::restore(uint32_t count, float mean, float m2)
{
    count_ = count;
    mean_ = count > 0U ? mean : 0.0F;
    m2_ = count > 1U ? std::max(m2, 0.0F) : 0.0F;
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
        return DM_RSSI_BOOTSTRAP_STD_DB;

    return std::sqrt(std::max(
        m2_ / static_cast<float>(count_ - 1U),
        0.0F));
}

DmRssiCalibrationProfile::DmRssiCalibrationProfile()
{
    reset();
}

void DmRssiCalibrationProfile::reset()
{
    static constexpr float edges[DM_RSSI_CALIBRATION_BIN_COUNT + 1U] = {
        0.0F,
        15.0F,
        30.0F,
        60.0F,
        80.0F,
        100.0F,
        120.0F,
        180.0F,
        DM_RSSI_OPEN_BIN_MAX_M,
    };

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        bins_[i] = DmRssiCalibrationBin{};
        bins_[i].minDistanceM = edges[i];
        bins_[i].maxDistanceM = edges[i + 1U];
    }
}

int DmRssiCalibrationProfile::findBin(float distanceMeters) const
{
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0F)
        return -1;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        if (distanceMeters >= bins_[i].minDistanceM &&
            distanceMeters < bins_[i].maxDistanceM)
        {
            return static_cast<int>(i);
        }
    }

    return static_cast<int>(DM_RSSI_CALIBRATION_BIN_COUNT - 1U);
}

bool DmRssiCalibrationProfile::update(float distanceMeters, float bestRssiDbm)
{
    const int index = findBin(distanceMeters);
    if (index < 0 || !std::isfinite(bestRssiDbm))
        return false;

    bins_[static_cast<size_t>(index)].bestRssi.update(bestRssiDbm);
    return true;
}

uint32_t DmRssiCalibrationProfile::totalSamples() const
{
    uint32_t total = 0U;
    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
        total += bins_[i].bestRssi.count();

    return total;
}

bool DmRssiCalibrationProfile::hasSafetyCoverage() const
{
    bool hasSafe = false;
    bool hasAlert = false;
    const float alertDistance = DM_MAX_DISTANCE_M * DM_DISTANCE_ALERT_START_RATIO;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        const DmRssiCalibrationBin &bin = bins_[i];
        if (bin.bestRssi.count() < DM_RSSI_TABLE_MIN_BIN_SAMPLES)
            continue;

        if (bin.maxDistanceM <= alertDistance)
            hasSafe = true;
        if (bin.minDistanceM >= alertDistance)
            hasAlert = true;
    }

    return hasSafe && hasAlert;
}

DmRssiTableEstimate DmRssiCalibrationProfile::estimate(float bestRssiDbm) const
{
    DmRssiTableEstimate out;
    out.bestRssiDbm = bestRssiDbm;
    if (!std::isfinite(bestRssiDbm))
        return out;

    float likelihoods[DM_RSSI_CALIBRATION_BIN_COUNT] = {};
    float likelihoodSum = 0.0F;
    size_t usableBins = 0U;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        const DmRssiCalibrationBin &bin = bins_[i];
        if (bin.bestRssi.count() < DM_RSSI_TABLE_MIN_BIN_SAMPLES)
            continue;

        // The learned standard deviation is the width of this bin's RSSI
        // likelihood. A floor prevents a low-sample/too-stable bin from
        // becoming unrealistically dominant.
        const float likelihood = gaussianLikelihood(
            bestRssiDbm,
            bin.bestRssi.mean(),
            bin.bestRssi.stdDev());
        likelihoods[i] = std::max(likelihood, 1.0e-12F);
        likelihoodSum += likelihoods[i];
        ++usableBins;
    }

    if (usableBins < 2U || likelihoodSum <= 0.0F)
        return out;

    double weightedDistance = 0.0;
    float bestProbability = 0.0F;
    float alertProbability = 0.0F;
    const float alertDistance = DM_MAX_DISTANCE_M * DM_DISTANCE_ALERT_START_RATIO;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        if (likelihoods[i] <= 0.0F)
            continue;

        const float probability = likelihoods[i] / likelihoodSum;
        weightedDistance +=
            static_cast<double>(probability) * static_cast<double>(binCenter(bins_[i]));
        bestProbability = std::max(bestProbability, probability);

        if (bins_[i].minDistanceM >= alertDistance)
            alertProbability += probability;
    }

    out.calibrated = true;
    out.predictedDistanceM = weightedDistance;
    out.band = dmDistBandFromDist(weightedDistance);
    out.confidence = bestProbability;
    out.positiveAlert =
        hasSafetyCoverage() && alertProbability >= DM_RSSI_ALERT_PROBABILITY;
    return out;
}

bool dmSelectBestRssi(
    bool rssiBtValid,
    float rssiBtDbm,
    bool rssiTbValid,
    float rssiTbDbm,
    float &bestRssiDbm)
{
    if (rssiBtValid && std::isfinite(rssiBtDbm) &&
        rssiTbValid && std::isfinite(rssiTbDbm))
    {
        bestRssiDbm = std::max(rssiBtDbm, rssiTbDbm);
        return true;
    }

    if (rssiBtValid && std::isfinite(rssiBtDbm))
    {
        bestRssiDbm = rssiBtDbm;
        return true;
    }

    if (rssiTbValid && std::isfinite(rssiTbDbm))
    {
        bestRssiDbm = rssiTbDbm;
        return true;
    }

    bestRssiDbm = 0.0F;
    return false;
}

double dmApproxDistBFromRSSI(
    const DmRssiCalibrationProfile &profile,
    bool rssiBtValid,
    float rssiBtDbm,
    bool rssiTbValid,
    float rssiTbDbm,
    float *confidence,
    DmRssiTableEstimate *detail)
{
    float bestRssiDbm = 0.0F;
    if (!dmSelectBestRssi(
            rssiBtValid,
            rssiBtDbm,
            rssiTbValid,
            rssiTbDbm,
            bestRssiDbm))
    {
        if (confidence != nullptr)
            *confidence = 0.0F;
        return -1.0;
    }

    const DmRssiTableEstimate estimate = profile.estimate(bestRssiDbm);
    if (confidence != nullptr)
        *confidence = estimate.confidence;
    if (detail != nullptr)
        *detail = estimate;

    return estimate.calibrated ? estimate.predictedDistanceM : -1.0;
}

DmDistanceBand dmDistBandFromDist(double distanceMeters, float maxDistanceMeters)
{
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0 || maxDistanceMeters <= 0.0F)
        return DmDistanceBand::Unknown;

    return dmDistanceBandFromRatio(
        static_cast<float>(distanceMeters / static_cast<double>(maxDistanceMeters)));
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
        return DmDistanceBand::Near;
    if (ratio < 0.80F)
        return DmDistanceBand::Mid;
    if (ratio < 1.00F)
        return DmDistanceBand::Warning;
    return DmDistanceBand::Beyond;
}

const char *dmRssiBinLabel(size_t index)
{
    static constexpr const char *labels[DM_RSSI_CALIBRATION_BIN_COUNT] = {
        "0-15",
        "15-30",
        "30-60",
        "60-80",
        "80-100",
        "100-120",
        "120-180",
        "180+",
    };

    return index < DM_RSSI_CALIBRATION_BIN_COUNT ? labels[index] : "?";
}
