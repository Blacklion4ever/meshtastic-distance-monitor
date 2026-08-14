#pragma once

#include "DistanceMonitorConfig.h"
#include "concurrency/OSThread.h"
#include <cstddef>
#include <cstdint>

class DistanceMonitorAudio : private concurrency::OSThread
{
  public:
    DistanceMonitorAudio();

    bool setup();
    bool isAvailable() const { return initialized_ && buzzerPin_ != 0U; }

    void playDistanceBip();
    void playConfirmationBop();
    void playFaultBops();
    void playPairingBops();
    void playTrackerNotification();
    void playSearchEnter();
    void playSearchExit();
    // proximity is clamped to [0,1]. At 1.0 (<=15 m in SEARCH) the detector
    // uses its highest frequency and an intentionally longer pulse.
    void playSearchPulse(float proximity);
    bool startSos();
    void stopSos();
    bool isSosActive() const { return sosActive_; }
    bool isBusyAboveSearch() const;

  protected:
    int32_t runOnce() override;

  public:
    struct Step
    {
        uint16_t frequencyHz;
        uint16_t durationMs;
    };

  private:
    enum class Pattern : uint8_t
    {
        None = 0,
        SearchPulse,
        DistanceBip,
        ConfirmationBop,
        FaultBops,
        Pairing,
        TrackerNotification,
        SearchEnter,
        SearchExit,
    };

    uint8_t buzzerPin_ = 0U;
    bool initialized_ = false;
    bool sosActive_ = false;

    Pattern pattern_ = Pattern::None;
    size_t stepIndex_ = 0U;
    uint32_t stepStartedMs_ = 0U;
    bool stepRunning_ = false;
    Step dynamicSearchSteps_[7] = {};
    size_t dynamicSearchStepCount_ = 0U;

    void startPattern(Pattern pattern);
    int patternPriority(Pattern pattern) const;
    const Step *patternSteps(Pattern pattern, size_t &count) const;
    void startTone(uint16_t frequencyHz);
    void stopTone();
};
