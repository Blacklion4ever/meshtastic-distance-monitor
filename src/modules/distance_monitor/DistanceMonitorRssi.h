#pragma once

#include "DistanceMonitorTypes.h"
#include <cstdint>

// Lightweight EMA filter for direct RSSI samples. The calibration table and
// flash persistence from protocol V6 are intentionally removed.
class DmRssiFilter
{
  public:
    void reset();
    void update(float rssiDbm, uint32_t nowMs);

    bool isInitialized() const { return initialized_; }
    float meanDbm() const { return meanDbm_; }
    float lastSampleDbm() const { return lastSampleDbm_; }
    float trendDbPerSec() const { return trendDbPerSec_; }
    uint32_t lastUpdateMs() const { return lastUpdateMs_; }

  private:
    bool initialized_ = false;
    float meanDbm_ = 0.0F;
    float lastSampleDbm_ = 0.0F;
    float trendDbPerSec_ = 0.0F;
    uint32_t lastUpdateMs_ = 0U;
};

// Select the strongest available direct path. With negative dBm values, the
// strongest path is the numerically largest value.
bool dmSelectBestRssi(
    bool btValid,
    float rssiBtDbm,
    bool tbValid,
    float rssiTbDbm,
    float &bestRssiDbm);

// Manual indoor proximity LUT. It deliberately returns a band, not metres.
DmDistanceBand dmDistanceBandFromRssi(float bestRssiDbm);

// Map an RSSI band to the existing alert-ratio machinery.
float dmRssiAlertRatio(DmDistanceBand band);
