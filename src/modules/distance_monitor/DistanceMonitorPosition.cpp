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
    if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT)
    {
        LOG_WARN("{GPS} marked NOT_PRESENT");
        return;
    }
#endif
    // Distance Monitor keeps GNSS continuously enabled at a fixed one-second
    // cadence. Position-report cadence is handled separately by the protocol.
    ensureGpsAlwaysOn();
}

void DistanceMonitorModule::ensureGpsAlwaysOn()
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        LOG_INFO("{GPS} gps == nullptr");
        return;
    }

    bool changed = false;

    if (config.position.gps_update_interval != 1U)
    {
        config.position.gps_update_interval = 1U;
        changed = true;
    }

    const bool stateOn =
        config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_ENABLED &&
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

    const bool lock = gps->hasLock();
    const uint32_t sats = localPosition.sats_in_view;
    const uint32_t pdop = localPosition.PDOP;
    const uint32_t hdop = localPosition.HDOP;

    const bool flow =
        sats != 0U ||
        pdop != 0U ||
        hdop != 0U;

    LOG_INFO(
        "{GPS} state=%s flow=%s lock=%s sats=%lu pdop=%lu hdop=%lu",
        stateOn ? "ON" : "OFF",
        flow ? "YES" : "NO",
        lock ? "YES" : "NO",
        static_cast<unsigned long>(sats),
        static_cast<unsigned long>(pdop),
        static_cast<unsigned long>(hdop));
#else
LOG_WARN("{GPS} No GPS support in this build");
#endif
}

bool DistanceMonitorModule::readImuSample(bool &motionDetected, float &rawNormG)
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

    // Follow gravity slowly so short accelerations remain visible as movement.
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
    // Use Meshtastic's published position. GPS time alone is not proof that a
    // geographic fix exists because UTC may become valid before position lock.
    const meshtastic_Position &position = localPosition;
    if (position.latitude_i == 0 || position.longitude_i == 0)
        return 0U;

    if (position.timestamp != 0U)
        return position.timestamp;
    if (position.time != 0U)
        return position.time;

    // Some producers omit timestamps. A content fingerprint still lets us
    // recognize a newly published valid solution without inventing a clock.
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

bool DistanceMonitorModule::readNewValidGpsFix()
{
#if MESHTASTIC_EXCLUDE_GPS
    return false;
#else
    if (gps == nullptr || !gps->hasLock())
        return false;

    const meshtastic_Position &position = localPosition;
    if (position.latitude_i == 0 || position.longitude_i == 0 ||
        position.sats_in_view <= 2U)
    {
        return false;
    }

    const uint32_t hdop = position.HDOP > 0U ? position.HDOP : UINT32_MAX;
    const uint32_t pdop = position.PDOP > 0U ? position.PDOP : UINT32_MAX;
    const uint32_t dop = std::min(hdop, pdop);
    if (dop == UINT32_MAX || dop > runtimeConfig_.maxDop)
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
        LOG_INFO(
            "{GPS} fix sats=%lu pdop=%lu hdop=%lu",
            static_cast<unsigned long>(localFix_.sats_in_view),
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
        else if (dmElapsedMs(nowMs, highSpeedBelowSinceMs_) >= DM_HIGH_SPEED_CLEAR_MS)
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
    // GNSS is continuously active at 1 Hz; freshness no longer depends on
    // movement or on the Distance Monitor report interval.
    return 1U + DM_FRESH_FIX_EXTRA_GRACE_S;
}

void DistanceMonitorModule::copyLocalPositionToState(
    DmNodeState &localState,
    uint32_t nowMs) const
{
    localState.positionKind = localPositionKind_;
    localState.moving = localMoving_;

    if (localPositionKind_ == DmPositionKind::NoFix)
    {
        localState.latitudeI = 0;
        localState.longitudeI = 0;
        return;
    }

    localState.latitudeI = localFix_.latitude_i;
    localState.longitudeI = localFix_.longitude_i;
}

void DistanceMonitorModule::updateFallDetection(uint32_t nowMs, float rawNormG)
{
    // Fall detection deliberately uses the raw acceleration norm; the gravity
    // low-pass used for movement detection must not hide free-fall or impact.
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

void DistanceMonitorModule::sampleLocalImu(DmNodeState &localState, uint32_t nowMs)
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

void DistanceMonitorModule::updateLocalPosition(DmNodeState &localState, uint32_t nowMs)
{
    if( lastGpsFunctionalCheck == 0U || dmElapsedMs(nowMs, lastGpsFunctionalCheck) >= DM_GPS_FUNCTIONAL_CHECK_MS)
    {
        ensureGpsAlwaysOn();
        lastGpsFunctionalCheck = nowMs;
    }

    if (readNewValidGpsFix())
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

    LOG_WARN("{Alarm} Cause=%s", dmSosCauseName(cause));
    sendSos(
        pendingSos_.targetNode,
        pendingSos_.sequence,
        pendingSos_.cause);
}
