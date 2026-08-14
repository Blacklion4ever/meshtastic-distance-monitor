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

static constexpr DistanceMonitorAudio::Step SEARCH_ENTER_STEPS[] = {
    {1500U, 80U}, {1950U, 80U}, {2450U, 100U},
};

static constexpr DistanceMonitorAudio::Step SEARCH_EXIT_STEPS[] = {
    {2450U, 80U}, {1950U, 80U}, {1500U, 100U},
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
        "Distance Monitor audio: buzzer ready gpio=%u config_gpio=%u",
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

void DistanceMonitorAudio::playSearchEnter()
{
    startPattern(Pattern::SearchEnter);
}

void DistanceMonitorAudio::playSearchExit()
{
    startPattern(Pattern::SearchExit);
}

void DistanceMonitorAudio::playSearchPulse(float proximity)
{
    if (!initialized_ || sosActive_)
        return;

    proximity = std::max(0.0F, std::min(1.0F, proximity));

    // Short radar-like preamble.
    dynamicSearchSteps_[0] = {330U, 6U};
    dynamicSearchSteps_[1] = {290U, 7U};
    dynamicSearchSteps_[2] = {250U, 8U};
    dynamicSearchSteps_[3] = {215U, 10U};
    dynamicSearchSteps_[4] = {260U, 8U};

    static constexpr float radarPulseDurationMs = 39.0F;
    if (proximity <= 0.0F)
    {
        dynamicSearchStepCount_ = 5U;
        startPattern(Pattern::SearchPulse);
        return;
    }

    const float detectionStartMs =
        static_cast<float>(DM_SEARCH_DETECTION_DELAY_FAR_MS) -
        proximity * static_cast<float>(
            DM_SEARCH_DETECTION_DELAY_FAR_MS -
            DM_SEARCH_DETECTION_DELAY_NEAR_MS);
    const uint16_t gapMs = static_cast<uint16_t>(
        std::max(5.0F, detectionStartMs - radarPulseDurationMs));
    dynamicSearchSteps_[5] = {0U, gapMs};

    // <=15 m maps to proximity=1. At that point use a deliberately distinct,
    // high and long detector tone so the operator cannot miss contact range.
    if (proximity >= 1.0F)
    {
        dynamicSearchSteps_[6] = {
            DM_SEARCH_DETECTION_CONTACT_HZ,
            DM_SEARCH_DETECTION_CONTACT_DURATION_MS};
    }
    else
    {
        const float shaped = std::sqrt(proximity);
        const float detectionHz =
            static_cast<float>(DM_SEARCH_DETECTION_MIN_HZ) +
            shaped * static_cast<float>(
                DM_SEARCH_DETECTION_NEAR_HZ -
                DM_SEARCH_DETECTION_MIN_HZ);
        const float detectionDurationMs =
            static_cast<float>(DM_SEARCH_DETECTION_DURATION_MIN_MS) +
            proximity * static_cast<float>(
                DM_SEARCH_DETECTION_DURATION_MAX_MS -
                DM_SEARCH_DETECTION_DURATION_MIN_MS);

        dynamicSearchSteps_[6] = {
            static_cast<uint16_t>(std::lround(detectionHz)),
            static_cast<uint16_t>(std::lround(detectionDurationMs))};
    }

    dynamicSearchStepCount_ = 7U;
    startPattern(Pattern::SearchPulse);
}

bool DistanceMonitorAudio::startSos()
{
    if (!initialized_)
    {
        LOG_ERROR("Distance Monitor audio: cannot start SOS, buzzer unavailable");
        return false;
    }

    if (sosActive_)
        return true;

    sosActive_ = true;
    pattern_ = Pattern::None;
    stepRunning_ = false;
    startTone(DM_BIP_FREQ_START_HZ);
    LOG_WARN("Distance Monitor audio: SOS tone started");
    return true;
}

void DistanceMonitorAudio::stopSos()
{
    if (!sosActive_)
        return;
    sosActive_ = false;
    stopTone();
}

bool DistanceMonitorAudio::isBusyAboveSearch() const
{
    return sosActive_ ||
           (pattern_ != Pattern::None &&
            patternPriority(pattern_) > patternPriority(Pattern::SearchPulse));
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
    case Pattern::ConfirmationBop:
        return 3;
    case Pattern::SearchEnter:
    case Pattern::SearchExit:
        return 2;
    case Pattern::SearchPulse:
        return 1;
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
    case Pattern::FaultBops:
        count = sizeof(FAULT_BOPS_STEPS) / sizeof(FAULT_BOPS_STEPS[0]);
        return FAULT_BOPS_STEPS;
    case Pattern::Pairing:
        count = sizeof(PAIRING_STEPS) / sizeof(PAIRING_STEPS[0]);
        return PAIRING_STEPS;
    case Pattern::TrackerNotification:
        count = sizeof(TRACKER_NOTIFICATION_STEPS) / sizeof(TRACKER_NOTIFICATION_STEPS[0]);
        return TRACKER_NOTIFICATION_STEPS;
    case Pattern::SearchEnter:
        count = sizeof(SEARCH_ENTER_STEPS) / sizeof(SEARCH_ENTER_STEPS[0]);
        return SEARCH_ENTER_STEPS;
    case Pattern::SearchExit:
        count = sizeof(SEARCH_EXIT_STEPS) / sizeof(SEARCH_EXIT_STEPS[0]);
        return SEARCH_EXIT_STEPS;
    case Pattern::SearchPulse:
        count = dynamicSearchStepCount_;
        return dynamicSearchSteps_;
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
        startTone(DM_BIP_FREQ_START_HZ);
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
