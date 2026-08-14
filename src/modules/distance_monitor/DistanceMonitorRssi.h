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

// Welford online statistics. Keeping M2 instead of all samples makes the
// calibration table small while preserving an unbiased sample standard deviation.
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
    DmOnlineStats bestRssi;
};

struct DmRssiTableEstimate
{
    DmDistanceBand band = DmDistanceBand::Unknown;
    double predictedDistanceM = 0.0;
    float confidence = 0.0F;
    float bestRssiDbm = 0.0F;
    bool calibrated = false;
    bool positiveAlert = false;
};

class DmRssiCalibrationProfile
{
  public:
    DmRssiCalibrationProfile();
    void reset();

    // Add one GPS-labelled sample to the matching distance bin. The sample is
    // already the best (strongest / least negative) value from RSSI_BT/RSSI_TB.
    bool update(float distanceMeters, float bestRssiDbm);

    // Estimate distance from one best-direction RSSI observation. Per-bin
    // standard deviation controls the Gaussian likelihood of each distance bin.
    DmRssiTableEstimate estimate(float bestRssiDbm) const;

    size_t binCount() const { return DM_RSSI_CALIBRATION_BIN_COUNT; }
    const DmRssiCalibrationBin &bin(size_t index) const { return bins_[index]; }
    DmRssiCalibrationBin &bin(size_t index) { return bins_[index]; }

    uint32_t totalSamples() const;
    bool hasSafetyCoverage() const;

  private:
    DmRssiCalibrationBin bins_[DM_RSSI_CALIBRATION_BIN_COUNT] = {};
    int findBin(float distanceMeters) const;
};

// Select the strongest available direction. RSSI is negative in dBm, so the
// strongest value is the numerically largest value (for example -55 > -72).
bool dmSelectBestRssi(
    bool rssiBtValid,
    float rssiBtDbm,
    bool rssiTbValid,
    float rssiTbDbm,
    float &bestRssiDbm);

// Convenience function used by DistanceMonitorModule: select the best current
// RSSI direction, then infer a distance from the calibrated table.
double dmApproxDistBFromRSSI(
    const DmRssiCalibrationProfile &profile,
    bool rssiBtValid,
    float rssiBtDbm,
    bool rssiTbValid,
    float rssiTbDbm,
    float *confidence = nullptr,
    DmRssiTableEstimate *detail = nullptr);

DmDistanceBand dmDistBandFromDist(
    double distanceMeters,
    float maxDistanceMeters = DM_MAX_DISTANCE_M);

float dmFuseRssi(float baseToTrackerDbm, float trackerToBaseDbm);
float dmFuseRssiTrend(float baseToTrackerTrendDbPerSec, float trackerToBaseTrendDbPerSec);
DmDistanceBand dmDistanceBandFromRatio(float ratio);
const char *dmRssiBinLabel(size_t index);
