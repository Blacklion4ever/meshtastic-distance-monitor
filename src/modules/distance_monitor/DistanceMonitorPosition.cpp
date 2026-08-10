#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "NodeDB.h"
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_GPS
#include "gps/GPS.h"
#endif

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && defined(HAS_QMA6100P)
#include "motion/QMA6100PSensor.h"
#endif

#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace
{
    static constexpr float STANDARD_GRAVITY_MPS2 = 9.80665F;
}

void DistanceMonitorModule::startLocalPositionManager(uint32_t nowMs)
{
    localPositionStarted_ = true;
    lastMotionMs_ = nowMs;

#if MESHTASTIC_EXCLUDE_GPS
    LOG_WARN("Distance Monitor GPS: excluded from firmware");
#else
    if (gps == nullptr)
    {
        LOG_WARN("Distance Monitor GPS: unavailable");
        return;
    }

    if (config.position.fixed_position)
    {
        LOG_WARN("Distance Monitor GPS: fixed_position is enabled");
        return;
    }

    if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT)
    {
        LOG_WARN("Distance Monitor GPS: marked NOT_PRESENT");
        return;
    }

    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
    gps->enable();

    // Force one initial power transition. Later calls to wakeGps() are no-ops
    // while the receiver is already awake.
    gpsSleeping_ = true;
    wakeGps(localDesiredGpsIntervalSec_);
#endif
}

bool DistanceMonitorModule::readImuSample(
    bool &motionDetected,
    float &dynamicAccelerationMps2,
    float &rawNormG)
{
    motionDetected = false;
    dynamicAccelerationMps2 = 0.0F;
    rawNormG = 1.0F;

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && defined(HAS_QMA6100P)
    QMA6100PSingleton *sensor = QMA6100PSingleton::GetInstance();
    if (sensor == nullptr)
    {
        return false;
    }

    outputData sample = {};
    if (!sensor->getAccelData(&sample))
    {
        return false;
    }

    rawNormG = std::sqrt(
        sample.xData * sample.xData +
        sample.yData * sample.yData +
        sample.zData * sample.zData);

    if (!gravityReferenceValid_)
    {
        gravityX_ = sample.xData;
        gravityY_ = sample.yData;
        gravityZ_ = sample.zData;
        gravityReferenceValid_ = true;
        return true;
    }

    // The low-pass vector follows gravity while preserving short dynamic acceleration.
    gravityX_ += DM_IMU_GRAVITY_ALPHA * (sample.xData - gravityX_);
    gravityY_ += DM_IMU_GRAVITY_ALPHA * (sample.yData - gravityY_);
    gravityZ_ += DM_IMU_GRAVITY_ALPHA * (sample.zData - gravityZ_);

    const float dynamicX = sample.xData - gravityX_;
    const float dynamicY = sample.yData - gravityY_;
    const float dynamicZ = sample.zData - gravityZ_;
    const float dynamicNormG = std::sqrt(
        dynamicX * dynamicX +
        dynamicY * dynamicY +
        dynamicZ * dynamicZ);

    dynamicAccelerationMps2 = dynamicNormG * STANDARD_GRAVITY_MPS2;
    motionDetected = dynamicNormG >= DM_IMU_MOTION_THRESHOLD_G;
    return true;
#else
    return false;
#endif
}

uint32_t DistanceMonitorModule::currentGpsSolutionId() const
{
    return localPosition.timestamp != 0U
               ? localPosition.timestamp
               : localPosition.time;
}

bool DistanceMonitorModule::readNewValidGpsFix()
{
    const uint32_t solutionId = currentGpsSolutionId();
    if (solutionId == 0U || solutionId == lastGpsSolutionId_)
    {
        return false;
    }

    // Process each Meshtastic GNSS solution once.
    lastGpsSolutionId_ = solutionId;

    if (localPosition.sats_in_view <= 2U)
    {
        return false;
    }

    const uint32_t dop =
        localPosition.PDOP > 0U ? localPosition.PDOP : localPosition.HDOP;

    return dop > 0U && dop <= runtimeConfig_.maxDop;
}

void DistanceMonitorModule::captureGpsFix(uint32_t nowMs)
{
    const bool acquired = localPositionKind_ == DmPositionKind::NoFix;

    localFix_ = localPosition;
    localFixMs_ = nowMs;
    localPositionKind_ = DmPositionKind::FreshFix;

    if (acquired)
    {
        LOG_INFO(
            "Distance Monitor GPS fix acquired: sats=%lu pdop=%lu hdop=%lu",
            static_cast<unsigned long>(localFix_.sats_in_view),
            static_cast<unsigned long>(localFix_.PDOP),
            static_cast<unsigned long>(localFix_.HDOP));
    }

    if (localFix_.has_ground_speed)
    {
        const float speedKmh =
            static_cast<float>(localFix_.ground_speed) * 3.6F;

        if (speedKmh > DM_HIGH_SPEED_THRESHOLD_KMH)
        {
            highSpeedBelowSinceMs_ = 0U;
            if (!highSpeedSosLatched_)
            {
                highSpeedSosLatched_ = true;
                LOG_WARN(
                    "Distance Monitor high speed: %.1f km/h",
                    static_cast<double>(speedKmh));
                triggerLocalSos(DmSosCause::HighSpeedMovement);
            }
        }
        else if (highSpeedSosLatched_)
        {
            if (highSpeedBelowSinceMs_ == 0U)
            {
                highSpeedBelowSinceMs_ = nowMs;
            }
            else if (dmElapsedMs(nowMs, highSpeedBelowSinceMs_) >=
                     DM_HIGH_SPEED_CLEAR_MS)
            {
                highSpeedSosLatched_ = false;
                highSpeedBelowSinceMs_ = 0U;
            }
        }
    }
}

void DistanceMonitorModule::applyGpsInterval(uint8_t updateIntervalSec)
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        return;
    }

    const uint32_t desired =
        std::max<uint32_t>(1U, static_cast<uint32_t>(updateIntervalSec));

    if (config.position.gps_update_interval != desired)
    {
        config.position.gps_update_interval = desired;
        LOG_DEBUG(
            "Distance Monitor GPS interval: %lus",
            static_cast<unsigned long>(desired));
    }
#else
    (void)updateIntervalSec;
#endif
}

void DistanceMonitorModule::wakeGps(uint8_t updateIntervalSec)
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        return;
    }

    applyGpsInterval(updateIntervalSec);

    if (!gpsSleeping_)
    {
        return;
    }

    LOG_INFO("Distance Monitor GPS: wake");
    gpsSleeping_ = false;
    gps->up();
#else
    (void)updateIntervalSec;
#endif
}

void DistanceMonitorModule::sleepGps()
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr || gpsSleeping_)
    {
        return;
    }

    config.position.gps_update_interval = DM_GPS_SLEEP_UPDATE_INTERVAL_S;
    gps->down();
    gpsSleeping_ = true;

    LOG_INFO(
        "Distance Monitor GPS: stationary cache, age=%lus",
        static_cast<unsigned long>(localFixAgeSeconds(millis())));
#endif
}

uint32_t DistanceMonitorModule::localFixAgeSeconds(uint32_t nowMs) const
{
    if (localPositionKind_ == DmPositionKind::NoFix)
    {
        return 0U;
    }

    return dmElapsedMs(nowMs, localFixMs_) / 1000U;
}

uint32_t DistanceMonitorModule::freshFixMaxAgeSeconds() const
{
    const uint32_t requested =
        highSpeedWatchUntilMs_ != 0U
            ? DM_HIGH_SPEED_GPS_INTERVAL_S
            : localDesiredGpsIntervalSec_;

    return std::max<uint32_t>(requested, 1U) +
           DM_FRESH_FIX_EXTRA_GRACE_S;
}

void DistanceMonitorModule::copyLocalPositionToState(
    DmNodeState &localState,
    uint32_t nowMs) const
{
    localState.positionKind = localPositionKind_;
    localState.positionRxMs = nowMs;
    localState.moving = localMoving_;

    if (localPositionKind_ == DmPositionKind::NoFix)
    {
        localState.positionAgeAtRxSeconds = 0U;
        localState.latitudeI = 0;
        localState.longitudeI = 0;
        return;
    }

    localState.latitudeI = localFix_.latitude_i;
    localState.longitudeI = localFix_.longitude_i;
    localState.positionAgeAtRxSeconds = localFixAgeSeconds(nowMs);
}

void DistanceMonitorModule::updateHighSpeedDetection(
    uint32_t nowMs,
    float dynamicAccelerationMps2)
{
    if (dynamicAccelerationMps2 > DM_VEHICLE_ACCEL_THRESHOLD_MPS2)
    {
        if (vehicleAccelConsecutiveSamples_ < 255U)
        {
            ++vehicleAccelConsecutiveSamples_;
        }
    }
    else
    {
        vehicleAccelConsecutiveSamples_ = 0U;
    }

    if (vehicleAccelConsecutiveSamples_ >=
        DM_VEHICLE_ACCEL_CONFIRM_SAMPLES)
    {
        highSpeedWatchUntilMs_ = nowMs + DM_HIGH_SPEED_WATCH_HOLD_MS;
        vehicleAccelConsecutiveSamples_ = 0U;
        wakeGps(DM_HIGH_SPEED_GPS_INTERVAL_S);
    }

    if (highSpeedWatchUntilMs_ != 0U &&
        static_cast<int32_t>(nowMs - highSpeedWatchUntilMs_) >= 0)
    {
        highSpeedWatchUntilMs_ = 0U;
    }
}

void DistanceMonitorModule::updateFallDetection(
    uint32_t nowMs,
    float rawNormG)
{
    if (!fallFreefallArmed_)
    {
        if (rawNormG < DM_FALL_FREEFALL_THRESHOLD_G)
        {
            fallFreefallArmed_ = true;
            fallFreefallMs_ = nowMs;
        }
        return;
    }

    if (dmElapsedMs(nowMs, fallFreefallMs_) >
        DM_FALL_IMPACT_WINDOW_MS)
    {
        fallFreefallArmed_ = false;
        return;
    }

    if (rawNormG > DM_FALL_IMPACT_THRESHOLD_G)
    {
        fallFreefallArmed_ = false;
        triggerLocalSos(DmSosCause::FallDetected);
    }
}

void DistanceMonitorModule::updateLocalPosition(
    DmNodeState &localState,
    uint32_t nowMs)
{
    bool motionDetected = false;
    float dynamicAccelerationMps2 = 0.0F;
    float rawNormG = 1.0F;

    const bool imuOk = readImuSample(
        motionDetected,
        dynamicAccelerationMps2,
        rawNormG);

    if (imuOk)
    {
        if (!imuAvailable_)
        {
            LOG_INFO("Distance Monitor IMU: ready");
        }
        imuAvailable_ = true;
        imuWarningLogged_ = false;

        updateHighSpeedDetection(nowMs, dynamicAccelerationMps2);
        if (!localState.isBase)
        {
            updateFallDetection(nowMs, rawNormG);
        }
    }
    else
    {
        if (imuAvailable_ || !imuWarningLogged_)
        {
            LOG_WARN("Distance Monitor IMU: unavailable; GPS stays active");
        }

        imuAvailable_ = false;
        imuWarningLogged_ = true;
        gravityReferenceValid_ = false;
        localMoving_ = true;

        if (localPositionKind_ == DmPositionKind::CachedStationary)
        {
            localPositionKind_ = DmPositionKind::NoFix;
        }
    }

    if (motionDetected)
    {
        lastMotionMs_ = nowMs;
        localMoving_ = true;

        if (localPositionKind_ == DmPositionKind::CachedStationary)
        {
            localPositionKind_ = DmPositionKind::NoFix;
        }
    }
    else if (imuAvailable_ &&
             dmElapsedMs(nowMs, lastMotionMs_) >=
                 DM_STATIONARY_CONFIRM_MS)
    {
        localMoving_ = false;
    }

    uint8_t gpsInterval = localDesiredGpsIntervalSec_;
    if (highSpeedWatchUntilMs_ != 0U)
    {
        gpsInterval = DM_HIGH_SPEED_GPS_INTERVAL_S;
    }

    if (localMoving_ || !imuAvailable_)
    {
        wakeGps(gpsInterval);
    }

    if (localPositionKind_ != DmPositionKind::CachedStationary &&
        readNewValidGpsFix())
    {
        captureGpsFix(nowMs);
    }

    if (localPositionKind_ == DmPositionKind::FreshFix)
    {
        if (imuAvailable_ && !localMoving_)
        {
            localPositionKind_ = DmPositionKind::CachedStationary;
            sleepGps();
            LOG_INFO(
                "Distance Monitor position: FRESH -> CACHED_STATIONARY");
        }
        else if (localFixAgeSeconds(nowMs) >=
                 freshFixMaxAgeSeconds())
        {
            localPositionKind_ = DmPositionKind::NoFix;
            LOG_INFO(
                "Distance Monitor position: FRESH -> NO_FIX (stale)");
        }
    }

    copyLocalPositionToState(localState, nowMs);
}

void DistanceMonitorModule::triggerLocalSos(DmSosCause cause)
{
    if (nodeDB == nullptr)
    {
        return;
    }

    size_t localIndex = 0U;
    size_t baseIndex = 0U;
    if (!findLocalIndex(localIndex) ||
        nodeStates_[localIndex].isBase ||
        !findBaseIndex(baseIndex))
    {
        return;
    }

    const uint32_t nowMs = millis();
    if (cause != DmSosCause::ManualButton &&
        lastSosTriggerMs_ != 0U &&
        dmElapsedMs(nowMs, lastSosTriggerMs_) < DM_SOS_REARM_MS)
    {
        return;
    }

    const uint32_t sequence = allocateSequenceNumber();
    pendingSos_.active = true;
    pendingSos_.targetNode = runtimeConfig_.members[baseIndex].nodeNum;
    pendingSos_.sequence = sequence;
    pendingSos_.cause = cause;
    pendingSos_.lastTxMs = nowMs;
    lastSosTriggerMs_ = nowMs;

    LOG_WARN(
        "Distance Monitor SOS TX: cause=%s seq=%lu",
        dmSosCauseName(cause),
        static_cast<unsigned long>(sequence));

    sendSos(
        pendingSos_.targetNode,
        pendingSos_.sequence,
        pendingSos_.cause);
}
