#pragma once

#include "DistanceMonitorTypes.h"
#include <cstddef>
#include <cstdint>

class DmRssiFilter
{
  public:
    void reset();
    void update(float sampleDbm, uint32_t nowMs);
    bool isUsable(uint32_t nowMs, uint32_t maxAgeMs) const;
    bool isInitialized() const { return initialized_; }
    uint32_t sampleCount() const { return sampleCount_; }
    uint32_t lastUpdateMs() const { return lastUpdateMs_; }
    float meanDbm() const { return meanDbm_; }
    float lastSampleDbm() const { return lastSampleDbm_; }
    float stdDb() const;
    float trendDbPerSec() const { return trendDbPerSec_; }

  private:
    bool initialized_ = false;
    uint32_t sampleCount_ = 0U;
    uint32_t lastUpdateMs_ = 0U;
    float meanDbm_ = 0.0F;
    float lastSampleDbm_ = 0.0F;
    float varianceDb2_ = 0.0F;
    float trendDbPerSec_ = 0.0F;
};

class DmOnlineStats
{
  public:
    void reset();
    void restore(uint32_t count, float mean, float m2);
    void update(float value);
    uint32_t count() const { return count_; }
    float mean() const { return mean_; }
    float m2() const { return m2_; }
    float stdDev() const;

  private:
    uint32_t count_ = 0U;
    float mean_ = 0.0F;
    float m2_ = 0.0F;
};

struct DmRssiCalibrationBin
{
    float minDistanceM = 0.0F;
    float maxDistanceM = 0.0F;
    DmOnlineStats baseToTracker;
    DmOnlineStats trackerToBase;
};

struct DmRssiTableEstimate
{
    DmDistanceBand band = DmDistanceBand::Unknown;
    float confidence = 0.0F;
    float alertProbability = 0.0F;
    float fusedRssiDbm = 0.0F;
    float fusedTrendDbPerSec = 0.0F;
    bool calibrated = false;
    bool positiveAlert = false;
};

class DmRssiCalibrationProfile
{
  public:
    DmRssiCalibrationProfile();
    void reset();

    bool update(float distanceMeters,
                bool baseToTrackerValid,
                float baseToTrackerDbm,
                bool trackerToBaseValid,
                float trackerToBaseDbm);

    DmRssiTableEstimate estimate(
        bool baseToTrackerValid,
        float baseToTrackerDbm,
        float baseToTrackerStdDb,
        uint32_t baseToTrackerAgeMs,
        bool trackerToBaseValid,
        float trackerToBaseDbm,
        float trackerToBaseStdDb,
        uint32_t trackerToBaseAgeMs,
        float fusedTrendDbPerSec,
        bool moving) const;

    size_t binCount() const { return DM_RSSI_CALIBRATION_BIN_COUNT; }
    const DmRssiCalibrationBin &bin(size_t index) const { return bins_[index]; }
    DmRssiCalibrationBin &bin(size_t index) { return bins_[index]; }

    uint32_t totalSamples() const;
    bool hasSafetyCoverage() const;

  private:
    DmRssiCalibrationBin bins_[DM_RSSI_CALIBRATION_BIN_COUNT] = {};
    int findBin(float distanceMeters) const;
};

float dmFuseRssi(float baseToTrackerDbm, float trackerToBaseDbm);
float dmFuseRssiTrend(float baseToTrackerTrendDbPerSec, float trackerToBaseTrendDbPerSec);
DmDistanceBand dmDistanceBandFromRatio(float ratio);
const char *dmRssiBinLabel(size_t index);
