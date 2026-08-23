#include "DistanceMonitorAudio.h"

#include "NodeDB.h"
#include "configuration.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#if !defined(ARCH_ESP32) && !defined(ARCH_RP2040) && !defined(ARCH_PORTDUINO)
#include "Tone.h"
#endif

namespace
{
static constexpr uint32_t DM_AUDIO_TICK_MS = 5U;

static constexpr DistanceMonitorAudio::Step DISTANCE_BIP_STEPS[] = {
    {DM_BIP_FREQ_START_HZ, DM_BIP_STAGE_1_MS},
    {DM_BIP_FREQ_END_HZ, DM_BIP_STAGE_2_MS},
};

static constexpr DistanceMonitorAudio::Step CONFIRMATION_BOP_STEPS[] = {
    {DM_BOP_FREQ_HZ, DM_BOP_DURATION_MS},
};

static constexpr DistanceMonitorAudio::Step FAULT_BOPS_STEPS[] = {
    {DM_BOP_FREQ_HZ, 180U}, {0U, 120U},
    {DM_BOP_FREQ_HZ, 180U}, {0U, 120U},
    {DM_BOP_FREQ_HZ, 180U},
};

static constexpr DistanceMonitorAudio::Step PAIRING_STEPS[] = {
    {DM_BOP_FREQ_HZ, DM_BOP_DURATION_MS},
    {0U, 400U},
    {DM_BOP_FREQ_HZ, DM_BOP_DURATION_MS},
};

static constexpr DistanceMonitorAudio::Step TRACKER_NOTIFICATION_STEPS[] = {
    {DM_BOP_FREQ_HZ, DM_BOP_DURATION_MS}, {0U, 250U},
    {DM_BIP_FREQ_START_HZ, DM_BIP_STAGE_1_MS}, {DM_BIP_FREQ_END_HZ, DM_BIP_STAGE_2_MS}, {0U, 250U},
    {DM_BOP_FREQ_HZ, DM_BOP_DURATION_MS}, {0U, 250U},
    {DM_BIP_FREQ_START_HZ, DM_BIP_STAGE_1_MS}, {DM_BIP_FREQ_END_HZ, DM_BIP_STAGE_2_MS}, {0U, 250U},
    {DM_BOP_FREQ_HZ, DM_BOP_DURATION_MS}, {0U, 250U},
    {DM_BIP_FREQ_START_HZ, DM_BIP_STAGE_1_MS}, {DM_BIP_FREQ_END_HZ, DM_BIP_STAGE_2_MS},
};

static constexpr DistanceMonitorAudio::Step SHUTDOWN_READY_BIP_STEPS[] = {
    {DM_BOP_FREQ_HZ, 80U},
};

static constexpr DistanceMonitorAudio::Step SOS_STEPS[] = {
    {DM_BIP_FREQ_START_HZ, 120U}, {0U, 120U},
    {DM_BIP_FREQ_START_HZ, 120U}, {0U, 1640U},
};
}

DistanceMonitorAudio::DistanceMonitorAudio()
    : concurrency::OSThread("DistanceMonitorAudio")
{
}

bool DistanceMonitorAudio::setup()
{
    if (initialized_)
        return true;

#if defined(PIN_BUZZER)
    buzzerPin_ = static_cast<uint8_t>(PIN_BUZZER);
#else
    buzzerPin_ = config.device.buzzer_gpio != 0U
                     ? static_cast<uint8_t>(config.device.buzzer_gpio)
                     : 25U;
#endif
    config.device.buzzer_gpio = buzzerPin_;

#if defined(BUZZER_EN_PIN)
    pinMode(BUZZER_EN_PIN, OUTPUT);
    digitalWrite(BUZZER_EN_PIN, HIGH);
#endif

    pinMode(buzzerPin_, OUTPUT);
    digitalWrite(buzzerPin_, LOW);
    noTone(buzzerPin_);
    initialized_ = true;

    LOG_INFO(
        "{DM@Audio} state=READY gpio=%u config_gpio=%u",
        static_cast<unsigned>(buzzerPin_),
        static_cast<unsigned>(config.device.buzzer_gpio));

    setIntervalFromNow(DM_AUDIO_TICK_MS);
    return true;
}

void DistanceMonitorAudio::playDistanceBip()
{
    startPattern(Pattern::DistanceBip);
}

void DistanceMonitorAudio::playConfirmationBop()
{
    startPattern(Pattern::ConfirmationBop);
}

void DistanceMonitorAudio::playFaultBops()
{
    startPattern(Pattern::FaultBops);
}

void DistanceMonitorAudio::playPairingBops()
{
    startPattern(Pattern::Pairing);
}

void DistanceMonitorAudio::playTrackerNotification()
{
    startPattern(Pattern::TrackerNotification);
}

void DistanceMonitorAudio::playShutdownReadyBip()
{
    startPattern(Pattern::ShutdownReadyBip);
}

bool DistanceMonitorAudio::startSos()
{
    if (!initialized_)
    {
        LOG_ERROR("{DM@Audio} state=ERROR reason=BUZZER_UNAVAILABLE action=START_SOS");
        return false;
    }

    if (sosActive_)
        return true;

    sosActive_ = true;
    pattern_ = Pattern::None;
    stepRunning_ = false;
    sosStepIndex_ = 0U;
    sosStepRunning_ = false;
    LOG_WARN("{DM@Audio} state=PLAYING reason=SOS");
    return true;
}

void DistanceMonitorAudio::stopSos()
{
    if (!sosActive_)
        return;
    sosActive_ = false;
    sosStepIndex_ = 0U;
    sosStepRunning_ = false;
    stopTone();
}

void DistanceMonitorAudio::startPattern(Pattern pattern)
{
    if (!initialized_ || sosActive_)
        return;
    if (pattern_ != Pattern::None &&
        patternPriority(pattern) < patternPriority(pattern_))
    {
        return;
    }

    pattern_ = pattern;
    stepIndex_ = 0U;
    stepRunning_ = false;
}

int DistanceMonitorAudio::patternPriority(Pattern pattern) const
{
    switch (pattern)
    {
    case Pattern::TrackerNotification:
        return 7;
    case Pattern::Pairing:
        return 6;
    case Pattern::FaultBops:
        return 5;
    case Pattern::DistanceBip:
        return 4;
    case Pattern::ShutdownReadyBip:
        return 3;
    case Pattern::ConfirmationBop:
        return 2;
    case Pattern::None:
    default:
        return 0;
    }
}

const DistanceMonitorAudio::Step *DistanceMonitorAudio::patternSteps(
    Pattern pattern,
    size_t &count) const
{
    switch (pattern)
    {
    case Pattern::DistanceBip:
        count = sizeof(DISTANCE_BIP_STEPS) / sizeof(DISTANCE_BIP_STEPS[0]);
        return DISTANCE_BIP_STEPS;
    case Pattern::ConfirmationBop:
        count = sizeof(CONFIRMATION_BOP_STEPS) / sizeof(CONFIRMATION_BOP_STEPS[0]);
        return CONFIRMATION_BOP_STEPS;
    case Pattern::ShutdownReadyBip:
        count = sizeof(SHUTDOWN_READY_BIP_STEPS) / sizeof(SHUTDOWN_READY_BIP_STEPS[0]);
        return SHUTDOWN_READY_BIP_STEPS;
    case Pattern::FaultBops:
        count = sizeof(FAULT_BOPS_STEPS) / sizeof(FAULT_BOPS_STEPS[0]);
        return FAULT_BOPS_STEPS;
    case Pattern::Pairing:
        count = sizeof(PAIRING_STEPS) / sizeof(PAIRING_STEPS[0]);
        return PAIRING_STEPS;
    case Pattern::TrackerNotification:
        count = sizeof(TRACKER_NOTIFICATION_STEPS) / sizeof(TRACKER_NOTIFICATION_STEPS[0]);
        return TRACKER_NOTIFICATION_STEPS;
    case Pattern::None:
    default:
        count = 0U;
        return nullptr;
    }
}

void DistanceMonitorAudio::startTone(uint16_t frequencyHz)
{
    if (!initialized_)
        return;

#if defined(BUZZER_EN_PIN)
    digitalWrite(BUZZER_EN_PIN, HIGH);
#endif

    if (frequencyHz == 0U)
    {
        stopTone();
        return;
    }
    tone(buzzerPin_, frequencyHz);
}

void DistanceMonitorAudio::stopTone()
{
    if (!initialized_)
        return;
    noTone(buzzerPin_);
    digitalWrite(buzzerPin_, LOW);
}

int32_t DistanceMonitorAudio::runOnce()
{
    if (!initialized_)
        return DM_AUDIO_TICK_MS;

    if (sosActive_)
    {
        const uint32_t nowMs = millis();
        const size_t stepCount = sizeof(SOS_STEPS) / sizeof(SOS_STEPS[0]);

        if (!sosStepRunning_)
        {
            startTone(SOS_STEPS[sosStepIndex_].frequencyHz);
            sosStepStartedMs_ = nowMs;
            sosStepRunning_ = true;
            return DM_AUDIO_TICK_MS;
        }

        if (static_cast<uint32_t>(nowMs - sosStepStartedMs_) >=
            SOS_STEPS[sosStepIndex_].durationMs)
        {
            stopTone();
            sosStepIndex_ = (sosStepIndex_ + 1U) % stepCount;
            sosStepRunning_ = false;
        }

        return DM_AUDIO_TICK_MS;
    }

    if (pattern_ == Pattern::None)
        return DM_AUDIO_TICK_MS;

    size_t stepCount = 0U;
    const Step *steps = patternSteps(pattern_, stepCount);
    if (steps == nullptr || stepIndex_ >= stepCount)
    {
        stopTone();
        pattern_ = Pattern::None;
        stepRunning_ = false;
        return DM_AUDIO_TICK_MS;
    }

    const uint32_t nowMs = millis();
    if (!stepRunning_)
    {
        startTone(steps[stepIndex_].frequencyHz);
        stepStartedMs_ = nowMs;
        stepRunning_ = true;
        return DM_AUDIO_TICK_MS;
    }

    if (static_cast<uint32_t>(nowMs - stepStartedMs_) >=
        steps[stepIndex_].durationMs)
    {
        stopTone();
        ++stepIndex_;
        stepRunning_ = false;
        if (stepIndex_ >= stepCount)
            pattern_ = Pattern::None;
    }

    return DM_AUDIO_TICK_MS;
}
