#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_GPS
#include "gps/GPS.h"
#endif

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && defined(HAS_QMA6100P)
#include "motion/QMA6100PSensor.h"
#endif

#include <Arduino.h>

void DistanceMonitorModule::startLocalPositionManager(uint32_t nowMs)
{
    localPositionStarted_ = true;
    lastMotionMs_ = nowMs;

#if MESHTASTIC_EXCLUDE_GPS
    LOG_WARN("Distance Monitor GPS: excluded from firmware -> NO_FIX");
#else
    if (gps == nullptr)
    {
        LOG_WARN("Distance Monitor GPS: unavailable -> NO_FIX");
        return;
    }

    if (config.position.fixed_position)
    {
        LOG_WARN("Distance Monitor GPS: fixed_position enabled; autonomous GNSS disabled");
        return;
    }

    if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT)
    {
        LOG_WARN("Distance Monitor GPS: NOT_PRESENT -> NO_FIX");
        return;
    }

    if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED)
    {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
        LOG_INFO("Distance Monitor GPS: ENABLED at runtime");
    }

    config.position.gps_update_interval = DM_GPS_ACTIVE_UPDATE_INTERVAL_S;
    gps->enable();
    gps->up();
    LOG_INFO("Distance Monitor GPS: ACTIVE");
#endif
}

bool DistanceMonitorModule::readImuMotion(bool &motionDetected)
{
    motionDetected = false;

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

    if (!imuReferenceValid_)
    {
        imuReferenceX_ = sample.xData;
        imuReferenceY_ = sample.yData;
        imuReferenceZ_ = sample.zData;
        imuReferenceValid_ = true;
        return true;
    }

    const float dx = sample.xData - imuReferenceX_;
    const float dy = sample.yData - imuReferenceY_;
    const float dz = sample.zData - imuReferenceZ_;
    const float deltaSquared = dx * dx + dy * dy + dz * dz;
    const float thresholdSquared =
        DM_IMU_MOTION_THRESHOLD_G * DM_IMU_MOTION_THRESHOLD_G;

    if (deltaSquared >= thresholdSquared)
    {
        motionDetected = true;
        imuReferenceX_ = sample.xData;
        imuReferenceY_ = sample.yData;
        imuReferenceZ_ = sample.zData;
    }

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

    // Each Meshtastic solution is considered only once.
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
    else
    {
        LOG_DEBUG("Distance Monitor GPS fix refreshed");
    }
}

void DistanceMonitorModule::wakeGps()
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        return;
    }

    config.position.gps_update_interval = DM_GPS_ACTIVE_UPDATE_INTERVAL_S;
    gps->up();
    LOG_INFO("Distance Monitor GPS: motion -> ACTIVE");
#endif
}

void DistanceMonitorModule::sleepGps()
{
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        return;
    }

    config.position.gps_update_interval = DM_GPS_SLEEP_UPDATE_INTERVAL_S;
    gps->down();
    LOG_INFO(
        "Distance Monitor GPS: stationary -> HARDSLEEP, cache age=%lus",
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

void DistanceMonitorModule::copyLocalPositionToState(
    DmNodeState &localState,
    uint32_t nowMs) const
{
    localState.positionKind = localPositionKind_;
    localState.positionRxMs = nowMs;

    if (localPositionKind_ == DmPositionKind::NoFix)
    {
        localState.positionAgeAtRxSeconds = 0U;
        return;
    }

    localState.latitudeI = localFix_.latitude_i;
    localState.longitudeI = localFix_.longitude_i;
    localState.positionAgeAtRxSeconds = localFixAgeSeconds(nowMs);
}

void DistanceMonitorModule::updateLocalPosition(
    DmNodeState &localState,
    uint32_t nowMs)
{
    bool motion = false;
    const bool imuOk = readImuMotion(motion);

    if (imuOk)
    {
        if (!imuAvailable_)
        {
            LOG_INFO(
                "Distance Monitor IMU: ready, motion threshold=%.2fg",
                static_cast<double>(DM_IMU_MOTION_THRESHOLD_G));
        }
        imuAvailable_ = true;
        imuWarningLogged_ = false;
    }
    else
    {
        if (imuAvailable_ || !imuWarningLogged_)
        {
            LOG_WARN("Distance Monitor IMU: unavailable; GPS stays ACTIVE");
        }
        imuAvailable_ = false;
        imuWarningLogged_ = true;
        imuReferenceValid_ = false;

        // Without IMU monitoring we cannot safely trust a stationary cache.
        if (localPositionKind_ == DmPositionKind::CachedStationary)
        {
            localPositionKind_ = DmPositionKind::NoFix;
            wakeGps();
        }
    }

    if (motion)
    {
        lastMotionMs_ = nowMs;

        if (localPositionKind_ == DmPositionKind::CachedStationary)
        {
            localPositionKind_ = DmPositionKind::NoFix;
            wakeGps();
        }
    }

    // Meshtastic already owns the GNSS driver and publishes validated samples
    // into localPosition. DistanceMonitor only applies its sats/DOP policy.
    if (localPositionKind_ != DmPositionKind::CachedStationary &&
        readNewValidGpsFix())
    {
        captureGpsFix(nowMs);
    }

    if (localPositionKind_ == DmPositionKind::FreshFix)
    {
        if (imuAvailable_ &&
            dmElapsedMs(nowMs, lastMotionMs_) >= DM_STATIONARY_CONFIRM_MS)
        {
            localPositionKind_ = DmPositionKind::CachedStationary;
            LOG_INFO("Distance Monitor position: FRESH -> CACHED_STATIONARY");
            sleepGps();
        }
        else if (localFixAgeSeconds(nowMs) >=
                 runtimeConfig_.freshPositionMaxAgeSeconds)
        {
            localPositionKind_ = DmPositionKind::NoFix;
            LOG_INFO("Distance Monitor position: FRESH -> NO_FIX (stale)");
        }
    }

    copyLocalPositionToState(localState, nowMs);
}

void DistanceMonitorModule::logLocalPositionSample(
    const DmNodeState &localState,
    uint32_t nowMs)
{
    if (hasLocalSampleLogTime_ &&
        dmElapsedMs(nowMs, lastLocalSampleLogMs_) < DM_POSITION_REQUEST_INTERVAL_MS)
    {
        return;
    }

    hasLocalSampleLogTime_ = true;
    lastLocalSampleLogMs_ = nowMs;

    const uint32_t age = positionAgeSeconds(localState, nowMs);
    const uint32_t quietSeconds =
        imuAvailable_ ? dmElapsedMs(nowMs, lastMotionMs_) / 1000U : 0U;
    const uint32_t dop =
        localPosition.PDOP > 0U ? localPosition.PDOP : localPosition.HDOP;

    LOG_INFO(
        "Distance Monitor local position: %s age=%lus imu=%s quiet=%lus gps=%s sats=%lu dop=%lu",
        dmPositionKindName(localState.positionKind),
        static_cast<unsigned long>(age),
        imuAvailable_ ? "OK" : "UNAVAILABLE",
        static_cast<unsigned long>(quietSeconds),
        localState.positionKind == DmPositionKind::CachedStationary
            ? "HARDSLEEP"
            : "ACTIVE",
        static_cast<unsigned long>(localPosition.sats_in_view),
        static_cast<unsigned long>(dop));
}
