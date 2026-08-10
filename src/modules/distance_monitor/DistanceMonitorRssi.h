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
    float stdDb() const;
    float trendDbPerSec() const { return trendDbPerSec_; }

  private:
    bool initialized_ = false;
    uint32_t sampleCount_ = 0U;
    uint32_t lastUpdateMs_ = 0U;
    float meanDbm_ = 0.0F;
    float varianceDb2_ = 0.0F;
    float trendDbPerSec_ = 0.0F;
};

class DmOnlineStats
{
  public:
    void reset();
    void seed(float mean, float stdDev, uint32_t count);
    void update(float value);

    uint32_t count() const { return count_; }
    float mean() const { return mean_; }
    float stdDev() const;

  private:
    uint32_t count_ = 0U;
    float mean_ = 0.0F;
    float m2_ = 0.0F;
};

class DmRssiCalibration
{
  public:
    DmRssiCalibration();

    void reset();
    void update(DmDistanceBand band, float fusedRssiDbm);
    DmDistanceBandEstimate estimate(
        float fusedRssiDbm,
        float fusedTrendDbPerSec,
        bool moving) const;

  private:
    DmOnlineStats buckets_[4];

    static int bandIndex(DmDistanceBand band);
};

float dmFuseRssi(float baseToTrackerDbm, float trackerToBaseDbm);
float dmFuseRssiTrend(float baseToTrackerTrendDbPerSec, float trackerToBaseTrendDbPerSec);
DmDistanceBand dmDistanceBandFromRatio(float ratio);
