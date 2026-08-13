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

float freshnessWeight(uint32_t ageMs, bool moving)
{
    const float tau = moving
                          ? static_cast<float>(DM_RSSI_MOVING_FRESHNESS_MS)
                          : static_cast<float>(DM_RSSI_STATIONARY_FRESHNESS_MS);
    return std::exp(-static_cast<float>(ageMs) / std::max(tau, 1.0F));
}

float directionQuality(uint32_t ageMs, float stdDb, bool moving)
{
    const float sigma = clampStd(stdDb);
    return freshnessWeight(ageMs, moving) / (sigma * sigma);
}

float binCenter(const DmRssiCalibrationBin &bin)
{
    if (bin.maxDistanceM >= DM_RSSI_OPEN_BIN_MAX_M)
    {
        return bin.minDistanceM + 60.0F;
    }
    return 0.5F * (bin.minDistanceM + bin.maxDistanceM);
}

DmDistanceBand bandFromDistance(float distanceMeters)
{
    const float ratio = DM_MAX_DISTANCE_M > 0.0F
                            ? distanceMeters / DM_MAX_DISTANCE_M
                            : 0.0F;
    return dmDistanceBandFromRatio(ratio);
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
    {
        return DM_RSSI_BOOTSTRAP_STD_DB;
    }

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
    {
        return -1;
    }

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

bool DmRssiCalibrationProfile::update(float distanceMeters,
                                      bool baseToTrackerValid,
                                      float baseToTrackerDbm,
                                      bool trackerToBaseValid,
                                      float trackerToBaseDbm)
{
    const int index = findBin(distanceMeters);
    if (index < 0)
    {
        return false;
    }

    bool changed = false;
    DmRssiCalibrationBin &target = bins_[static_cast<size_t>(index)];

    if (baseToTrackerValid && std::isfinite(baseToTrackerDbm))
    {
        target.baseToTracker.update(baseToTrackerDbm);
        changed = true;
    }

    if (trackerToBaseValid && std::isfinite(trackerToBaseDbm))
    {
        target.trackerToBase.update(trackerToBaseDbm);
        changed = true;
    }

    return changed;
}

uint32_t DmRssiCalibrationProfile::totalSamples() const
{
    uint32_t total = 0U;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        total += bins_[i].baseToTracker.count();
        total += bins_[i].trackerToBase.count();
    }

    return total;
}

bool DmRssiCalibrationProfile::hasSafetyCoverage() const
{
    bool hasSafe = false;
    bool hasAlert = false;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        const DmRssiCalibrationBin &b = bins_[i];
        const bool usable =
            b.baseToTracker.count() >= DM_RSSI_TABLE_MIN_BIN_SAMPLES ||
            b.trackerToBase.count() >= DM_RSSI_TABLE_MIN_BIN_SAMPLES;

        if (!usable)
        {
            continue;
        }

        if (b.maxDistanceM <= DM_MAX_DISTANCE_M * DM_DISTANCE_ALERT_START_RATIO)
        {
            hasSafe = true;
        }

        if (b.minDistanceM >= DM_MAX_DISTANCE_M * DM_DISTANCE_ALERT_START_RATIO)
        {
            hasAlert = true;
        }
    }

    return hasSafe && hasAlert;
}

DmRssiTableEstimate DmRssiCalibrationProfile::estimate(
    bool baseToTrackerValid,
    float baseToTrackerDbm,
    float baseToTrackerStdDb,
    uint32_t baseToTrackerAgeMs,
    bool trackerToBaseValid,
    float trackerToBaseDbm,
    float trackerToBaseStdDb,
    uint32_t trackerToBaseAgeMs,
    float fusedTrendDbPerSec,
    bool moving) const
{
    DmRssiTableEstimate out;
    out.fusedTrendDbPerSec = fusedTrendDbPerSec;

    const float btQuality = baseToTrackerValid
                                ? directionQuality(baseToTrackerAgeMs,
                                                   baseToTrackerStdDb,
                                                   moving)
                                : 0.0F;
    const float tbQuality = trackerToBaseValid
                                ? directionQuality(trackerToBaseAgeMs,
                                                   trackerToBaseStdDb,
                                                   moving)
                                : 0.0F;
    const float qualitySum = btQuality + tbQuality;

    if (qualitySum <= 0.0F)
    {
        return out;
    }

    out.fusedRssiDbm =
        (btQuality * baseToTrackerDbm + tbQuality * trackerToBaseDbm) /
        qualitySum;

    float probabilities[DM_RSSI_CALIBRATION_BIN_COUNT] = {};
    float sum = 0.0F;
    size_t usableBins = 0U;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        const DmRssiCalibrationBin &b = bins_[i];
        float logLikelihood = 0.0F;
        float usedWeight = 0.0F;

        if (baseToTrackerValid &&
            b.baseToTracker.count() >= DM_RSSI_TABLE_MIN_BIN_SAMPLES)
        {
            const float sigma = std::max(
                clampStd(b.baseToTracker.stdDev()),
                clampStd(baseToTrackerStdDb));
            const float likelihood = std::max(
                gaussianLikelihood(baseToTrackerDbm,
                                   b.baseToTracker.mean(),
                                   sigma),
                1.0e-12F);
            logLikelihood += (btQuality / qualitySum) * std::log(likelihood);
            usedWeight += btQuality / qualitySum;
        }

        if (trackerToBaseValid &&
            b.trackerToBase.count() >= DM_RSSI_TABLE_MIN_BIN_SAMPLES)
        {
            const float sigma = std::max(
                clampStd(b.trackerToBase.stdDev()),
                clampStd(trackerToBaseStdDb));
            const float likelihood = std::max(
                gaussianLikelihood(trackerToBaseDbm,
                                   b.trackerToBase.mean(),
                                   sigma),
                1.0e-12F);
            logLikelihood += (tbQuality / qualitySum) * std::log(likelihood);
            usedWeight += tbQuality / qualitySum;
        }

        if (usedWeight <= 0.0F)
        {
            continue;
        }

        probabilities[i] = std::exp(logLikelihood / usedWeight);
        sum += probabilities[i];
        ++usableBins;
    }

    if (sum <= 0.0F || usableBins < 2U)
    {
        return out;
    }

    size_t bestIndex = 0U;
    float best = 0.0F;
    float alertProbability = 0.0F;
    const float alertDistance =
        DM_MAX_DISTANCE_M * DM_DISTANCE_ALERT_START_RATIO;

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        probabilities[i] /= sum;

        if (probabilities[i] > best)
        {
            best = probabilities[i];
            bestIndex = i;
        }

        if (bins_[i].minDistanceM >= alertDistance)
        {
            alertProbability += probabilities[i];
        }
    }

    out.calibrated = true;
    out.confidence = best;
    out.alertProbability = alertProbability;
    out.band = bandFromDistance(binCenter(bins_[bestIndex]));

    // RSSI remains a normal Distance Monitor fallback only. It may raise a
    // distance alarm on positive calibrated evidence, but SEARCH never uses it.
    out.positiveAlert = hasSafetyCoverage() &&
                        alertProbability >= DM_RSSI_ALERT_PROBABILITY;

    return out;
}

float dmFuseRssi(float baseToTrackerDbm, float trackerToBaseDbm)
{
    return 0.5F * baseToTrackerDbm + 0.5F * trackerToBaseDbm;
}

float dmFuseRssiTrend(float baseToTrackerTrendDbPerSec,
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
