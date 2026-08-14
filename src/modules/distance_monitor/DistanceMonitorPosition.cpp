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
#include <limits>

void DistanceMonitorModule::startLocalPositionManager(uint32_t nowMs)
{
    localPositionStarted_ = true;
    lastMotionMs_ = nowMs;

#if MESHTASTIC_EXCLUDE_GPS
    LOG_WARN("{GPS} unavailable: excluded from firmware");
#else
    if (gps == nullptr)
    {
        LOG_WARN("{GPS} unavailable");
        return;
    }

    if (config.position.fixed_position)
    {
        LOG_WARN("{GPS} fixed_position enabled");
        return;
    }

    if (config.position.gps_mode ==
        meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT)
    {
        LOG_WARN("{GPS} marked NOT_PRESENT");
        return;
    }

    // Distance Monitor owns the GNSS cadence and keeps it continuously enabled.
    ensureGpsAlwaysOn(nowMs, true);
#endif
}

void DistanceMonitorModule::ensureGpsAlwaysOn(
    uint32_t nowMs,
    bool forceDiagnostic)
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        if (forceDiagnostic ||
            lastGpsFunctionalCheckMs_ == 0U ||
            dmElapsedMs(nowMs, lastGpsFunctionalCheckMs_) >=
                DM_GPS_FUNCTIONAL_CHECK_MS)
        {
            lastGpsFunctionalCheckMs_ = nowMs;
            LOG_INFO("{GPS} state=OFF flow=NO sats=0 dop=0 acc=INF pdop=0 hdop=0");
        }
        return;
    }

    bool changed = false;

    if (config.position.gps_update_interval != 1U)
    {
        config.position.gps_update_interval = 1U;
        changed = true;
    }

    const bool stateOn =
        config.position.gps_mode ==
            meshtastic_Config_PositionConfig_GpsMode_ENABLED &&
        !gps->isPowerSaving();

    if (!stateOn)
    {
        config.position.gps_mode =
            meshtastic_Config_PositionConfig_GpsMode_ENABLED;
        gps->enable();
        changed = true;
        LOG_WARN("{GPS} state=OFF -> forcing enable");
    }

    if (changed)
        LOG_INFO("{GPS} enabled interval=1s");

    const bool diagnosticDue =
        forceDiagnostic ||
        lastGpsFunctionalCheckMs_ == 0U ||
        dmElapsedMs(nowMs, lastGpsFunctionalCheckMs_) >=
            DM_GPS_FUNCTIONAL_CHECK_MS;
    if (!diagnosticDue)
        return;

    lastGpsFunctionalCheckMs_ = nowMs;

    const uint32_t sats = localPosition.sats_in_view;
    const uint32_t pdop = localPosition.PDOP;
    const uint32_t hdop = localPosition.HDOP;
    const uint32_t dop = dmBestDop(pdop, hdop);
    const double accuracyM = dmAccuracyMetersFromDop(dop);
    const bool flow =
        sats != 0U ||
        pdop != 0U ||
        hdop != 0U;

    const bool effectiveStateOn =
        config.position.gps_mode ==
            meshtastic_Config_PositionConfig_GpsMode_ENABLED &&
        !gps->isPowerSaving();

    if (std::isfinite(accuracyM))
    {
        LOG_INFO(
            "{GPS} state=%s flow=%s sats=%lu dop=%lu acc=%.0fm pdop=%lu hdop=%lu",
            effectiveStateOn ? "ON" : "OFF",
            flow ? "YES" : "NO",
            static_cast<unsigned long>(sats),
            static_cast<unsigned long>(dop),
            accuracyM,
            static_cast<unsigned long>(pdop),
            static_cast<unsigned long>(hdop));
    }
    else
    {
        LOG_INFO(
            "{GPS} state=%s flow=%s sats=%lu dop=0 acc=INF pdop=%lu hdop=%lu",
            effectiveStateOn ? "ON" : "OFF",
            flow ? "YES" : "NO",
            static_cast<unsigned long>(sats),
            static_cast<unsigned long>(pdop),
            static_cast<unsigned long>(hdop));
    }
#else
    (void)nowMs;
    (void)forceDiagnostic;
#endif
}

bool DistanceMonitorModule::readImuSample(
    bool &motionDetected,
    float &rawNormG)
{
    motionDetected = false;
    rawNormG = 1.0F;

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && defined(HAS_QMA6100P)
    QMA6100PSingleton *sensor = QMA6100PSingleton::GetInstance();
    if (sensor == nullptr)
        return false;

    outputData sample = {};
    if (!sensor->getAccelData(&sample))
        return false;

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

    // Follow gravity slowly so short accelerations remain movement events.
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

    motionDetected = dynamicNormG >= DM_IMU_MOTION_THRESHOLD_G;
    return true;
#else
    return false;
#endif
}

uint32_t DistanceMonitorModule::currentGpsSolutionId() const
{
#if MESHTASTIC_EXCLUDE_GPS
    return 0U;
#else
    const meshtastic_Position &position = localPosition;
    if (position.latitude_i == 0 || position.longitude_i == 0)
        return 0U;

    if (position.timestamp != 0U)
        return position.timestamp;
    if (position.time != 0U)
        return position.time;

    // A content fingerprint handles producers that publish no timestamp.
    uint32_t hash = 2166136261U;
    const auto mix = [&hash](uint32_t value)
    {
        hash ^= value;
        hash *= 16777619U;
    };

    mix(static_cast<uint32_t>(position.latitude_i));
    mix(static_cast<uint32_t>(position.longitude_i));
    mix(static_cast<uint32_t>(position.sats_in_view));
    mix(static_cast<uint32_t>(position.PDOP));
    mix(static_cast<uint32_t>(position.HDOP));
    return hash != 0U ? hash : 1U;
#endif
}

bool DistanceMonitorModule::readNewUsableGpsPosition()
{
#if MESHTASTIC_EXCLUDE_GPS
    return false;
#else
    if (gps == nullptr)
        return false;

    const meshtastic_Position &position = localPosition;
    if (position.latitude_i == 0 ||
        position.longitude_i == 0 ||
        position.sats_in_view <= 2U)
    {
        return false;
    }

    // DM uses actual coordinates + DOP and lets the distance layer decide
    // whether the resulting accuracy is sufficient.
    const uint32_t dop = dmBestDop(position.PDOP, position.HDOP);
    if (dop == 0U)
        return false;

    const uint32_t solutionId = currentGpsSolutionId();
    if (solutionId == 0U || solutionId == lastGpsSolutionId_)
        return false;

    lastGpsSolutionId_ = solutionId;
    return true;
#endif
}

void DistanceMonitorModule::captureGpsFix(uint32_t nowMs)
{
#if MESHTASTIC_EXCLUDE_GPS
    (void)nowMs;
#else
    if (gps == nullptr)
        return;

    const bool acquired = localPositionKind_ == DmPositionKind::NoFix;
    localFix_ = localPosition;
    localFixMs_ = nowMs;
    localPositionKind_ = DmPositionKind::FreshFix;

    if (acquired)
    {
        const uint32_t dop = dmBestDop(localFix_.PDOP, localFix_.HDOP);
        const double accuracyM = dmAccuracyMetersFromDop(dop);
        LOG_INFO(
            "{GPS} fix sats=%lu dop=%lu acc=%.0fm pdop=%lu hdop=%lu",
            static_cast<unsigned long>(localFix_.sats_in_view),
            static_cast<unsigned long>(dop),
            accuracyM,
            static_cast<unsigned long>(localFix_.PDOP),
            static_cast<unsigned long>(localFix_.HDOP));
    }

    if (!localFix_.has_ground_speed)
        return;

    const float speedKmh = static_cast<float>(localFix_.ground_speed) * 3.6F;
    if (speedKmh > DM_HIGH_SPEED_THRESHOLD_KMH)
    {
        highSpeedBelowSinceMs_ = 0U;
        if (!highSpeedSosLatched_)
        {
            highSpeedSosLatched_ = true;
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
#endif
}

uint32_t DistanceMonitorModule::localFixAgeSeconds(uint32_t nowMs) const
{
    if (localPositionKind_ == DmPositionKind::NoFix)
        return 0U;
    return dmElapsedMs(nowMs, localFixMs_) / 1000U;
}

uint32_t DistanceMonitorModule::freshFixMaxAgeSeconds() const
{
    return 1U + DM_FRESH_FIX_EXTRA_GRACE_S;
}

void DistanceMonitorModule::copyLocalPositionToState(
    DmNodeState &localState,
    uint32_t) const
{
    localState.positionKind = localPositionKind_;
    localState.moving = localMoving_;

    if (localPositionKind_ == DmPositionKind::NoFix)
    {
        localState.latitudeI = 0;
        localState.longitudeI = 0;
        localState.positionDop = 0U;
        localState.positionAccuracyMeters =
            std::numeric_limits<double>::infinity();
        return;
    }

    localState.latitudeI = localFix_.latitude_i;
    localState.longitudeI = localFix_.longitude_i;

    const uint32_t dop = dmBestDop(localFix_.PDOP, localFix_.HDOP);
    localState.positionDop = static_cast<uint16_t>(
        std::min<uint32_t>(dop, static_cast<uint32_t>(UINT16_MAX)));
    localState.positionAccuracyMeters =
        dmAccuracyMetersFromDop(localState.positionDop);
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

    const uint32_t elapsed = dmElapsedMs(nowMs, fallFreefallMs_);
    if (elapsed > DM_FALL_IMPACT_WINDOW_MS)
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

void DistanceMonitorModule::sampleLocalImu(
    DmNodeState &localState,
    uint32_t nowMs)
{
    bool motionDetected = false;
    float rawNormG = 1.0F;
    const bool imuOk = readImuSample(motionDetected, rawNormG);

    if (!imuOk)
    {
        if (imuAvailable_ || !imuWarningLogged_)
            LOG_WARN("{IMU} unavailable");
        imuAvailable_ = false;
        imuWarningLogged_ = true;
        gravityReferenceValid_ = false;
        localMoving_ = true;
        localState.moving = true;
        return;
    }

    if (!imuAvailable_)
        LOG_INFO("{IMU} ready");

    imuAvailable_ = true;
    imuWarningLogged_ = false;

    if (motionDetected)
    {
        lastMotionMs_ = nowMs;
        localMoving_ = true;
    }
    else if (dmElapsedMs(nowMs, lastMotionMs_) >= DM_STATIONARY_CONFIRM_MS)
    {
        localMoving_ = false;
    }

    localState.moving = localMoving_;
    if (!localState.isBase)
        updateFallDetection(nowMs, rawNormG);
}

void DistanceMonitorModule::updateLocalPosition(
    DmNodeState &localState,
    uint32_t nowMs)
{
    // Reassert every control tick, but only emit the detailed diagnostic once
    // per minute. This closes the window where another subsystem could disable
    // the GNSS without DM noticing.
    ensureGpsAlwaysOn(nowMs, false);

    if (readNewUsableGpsPosition())
        captureGpsFix(nowMs);

    if (localPositionKind_ == DmPositionKind::FreshFix &&
        localFixAgeSeconds(nowMs) >= freshFixMaxAgeSeconds())
    {
        localPositionKind_ = DmPositionKind::NoFix;
        LOG_INFO("{GPS} no fix");
    }

    copyLocalPositionToState(localState, nowMs);
}

void DistanceMonitorModule::triggerLocalSos(DmSosCause cause)
{
    if (nodeDB == nullptr)
        return;

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
        "{Alarm} Cause=%s silent=%s",
        dmSosCauseName(cause),
        DM_ALARM_AUDIO_SILENT ? "YES" : "NO");

    sendSos(
        pendingSos_.targetNode,
        pendingSos_.sequence,
        pendingSos_.cause);
}
