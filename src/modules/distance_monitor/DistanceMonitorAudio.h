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
    void playShutdownReadyBip();
    bool startSos();
    void stopSos();
    bool isSosActive() const { return sosActive_; }

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
        DistanceBip,
        ConfirmationBop,
        ShutdownReadyBip,
        FaultBops,
        Pairing,
        TrackerNotification,
    };

    uint8_t buzzerPin_ = 0U;
    bool initialized_ = false;
    bool sosActive_ = false;

    Pattern pattern_ = Pattern::None;
    size_t stepIndex_ = 0U;
    uint32_t stepStartedMs_ = 0U;
    bool stepRunning_ = false;
    size_t sosStepIndex_ = 0U;
    uint32_t sosStepStartedMs_ = 0U;
    bool sosStepRunning_ = false;

    void startPattern(Pattern pattern);
    int patternPriority(Pattern pattern) const;
    const Step *patternSteps(Pattern pattern, size_t &count) const;
    void startTone(uint16_t frequencyHz);
    void stopTone();
};
