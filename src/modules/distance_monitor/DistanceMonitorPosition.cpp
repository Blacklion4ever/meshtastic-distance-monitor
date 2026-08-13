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

    // Distance Monitor policy: GNSS is always on.
    //
    // Meshtastic 2.7.26 only attempts GPS sleep when the configured update
    // interval is greater than GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS (10 s).
    // Set a single fixed 1 s interval once, then never modulate it from IMU,
    // movement, SEARCH, vehicle watch, etc.
    config.position.gps_update_interval = 1U;

    // Do not issue an ACTIVE->ACTIVE transition if Meshtastic already has GPS
    // enabled. Only re-enable when GPS mode had actually been disabled.
    if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED ||
        gps->isPowerSaving())
    {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
        gps->enable();
        LOG_INFO("Distance Monitor GPS: enabled by ALWAYS_ON startup");
    }
    else
    {
        LOG_INFO("Distance Monitor GPS: already enabled, leave receiver untouched");
    }

    gpsSleeping_ = false;
    LOG_INFO("Distance Monitor GPS: ALWAYS_ON policy enabled (1s fixed)");
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
#if MESHTASTIC_EXCLUDE_GPS
    return 0U;
#else
    // localPosition is the position Meshtastic publishes to the rest of the
    // firmware. Never use GPS time by itself as evidence of a position fix:
    // an AG3335 can provide valid UTC time before it has a position lock.
    const meshtastic_Position &position = localPosition;

    if (position.latitude_i == 0 || position.longitude_i == 0)
    {
        return 0U;
    }

    // Prefer Meshtastic's position publication timestamp when available.
    // time is only a fallback *after* the position itself has been validated.
    if (position.timestamp != 0U)
    {
        return position.timestamp;
    }
    if (position.time != 0U)
    {
        return position.time;
    }

    // Some position producers do not populate timestamp/time. Build a stable
    // content fingerprint so the first valid position can still be accepted.
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
    {
        return false;
    }

    // Consume the position published by Meshtastic, not the GPS thread's
    // transient working structure. The GPS thread may acquire time before it
    // publishes a usable geographic position.
    const meshtastic_Position &position = localPosition;

    if (position.latitude_i == 0 || position.longitude_i == 0)
    {
        return false;
    }
    if (position.sats_in_view <= 2U)
    {
        return false;
    }

    const uint32_t hdop = position.HDOP > 0U ? position.HDOP : UINT32_MAX;

    const uint32_t pdop = position.PDOP > 0U ? position.PDOP : UINT32_MAX;

    const uint32_t dop = std::min(hdop, pdop);

    if (dop == UINT32_MAX || dop > runtimeConfig_.maxDop)
    {
        return false;
    }

    const uint32_t solutionId = currentGpsSolutionId();
    if (solutionId == 0U || solutionId == lastGpsSolutionId_)
    {
        return false;
    }

    lastGpsSolutionId_ = solutionId;
    return true;
#endif
}

void DistanceMonitorModule::captureGpsFix(uint32_t nowMs)
{
#if MESHTASTIC_EXCLUDE_GPS
    (void)nowMs;
    return;
#else
    if (gps == nullptr)
    {
        return;
    }

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
#endif
}

void DistanceMonitorModule::applyGpsInterval(uint8_t updateIntervalSec)
{
    // Compatibility entry point: other Distance Monitor source files still
    // call this when their reporting interval changes. GPS cadence is no
    // longer coupled to those values. Keep one fixed always-on interval.
    (void)updateIntervalSec;
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        return;
    }

    if (config.position.gps_update_interval != 1U)
    {
        config.position.gps_update_interval = 1U;
        LOG_INFO("Distance Monitor GPS: restore ALWAYS_ON 1s interval");
    }
#endif
}

void DistanceMonitorModule::wakeGps(uint8_t updateIntervalSec)
{
    // Compatibility entry point. The requested interval is intentionally
    // ignored: Distance Monitor now has one GNSS policy only: ALWAYS_ON.
    (void)updateIntervalSec;
#if !MESHTASTIC_EXCLUDE_GPS
    if (gps == nullptr)
    {
        return;
    }

    applyGpsInterval(1U);

    // Normal Meshtastic disable paths switch gps_mode away from ENABLED.
    // Re-enable only when that happened; do not repeatedly call up()/enable()
    // while the GPS is already under our always-on policy.
    if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED ||
        gps->isPowerSaving() || gpsSleeping_)
    {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
        gps->enable();
        gpsSleeping_ = false;
        LOG_WARN("Distance Monitor GPS: reactivated by ALWAYS_ON watchdog");
    }
#endif
}

void DistanceMonitorModule::sleepGps()
{
    // Deliberate no-op. Distance Monitor never puts GNSS to sleep anymore.
    gpsSleeping_ = false;
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
    // GNSS is continuously active at 1 Hz. Freshness is therefore unrelated
    // to IMU state or reporting cadence.
    return 1U + DM_FRESH_FIX_EXTRA_GRACE_S;
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
    // GNSS is always active now, so IMU activity no longer controls GPS power
    // or GPS cadence. Keep this lightweight diagnostic only; actual high-speed
    // SOS authority remains GNSS ground speed in captureGpsFix().
    if (dynamicAccelerationMps2 > DM_VEHICLE_ACCEL_THRESHOLD_MPS2)
    {
        if (vehicleAccelAboveSinceMs_ == 0U)
        {
            vehicleAccelAboveSinceMs_ = nowMs;
        }
        else if (dmElapsedMs(nowMs, vehicleAccelAboveSinceMs_) >=
                 DM_VEHICLE_ACCEL_CONFIRM_MS)
        {
            LOG_INFO(
                "Distance Monitor vehicle watch: IMU activity %.2fm/s2 for %lums",
                static_cast<double>(dynamicAccelerationMps2),
                static_cast<unsigned long>(
                    dmElapsedMs(nowMs, vehicleAccelAboveSinceMs_)));
            vehicleAccelAboveSinceMs_ = 0U;
        }
    }
    else
    {
        vehicleAccelAboveSinceMs_ = 0U;
    }

    // Legacy field retained for source compatibility with the rest of the
    // module, but no longer used to drive GNSS behavior.
    highSpeedWatchUntilMs_ = 0U;
}

void DistanceMonitorModule::updateFallDetection(
    uint32_t nowMs,
    float rawNormG)
{
    // IMPORTANT: rawNormG is the raw accelerometer norm. The gravity low-pass
    // used for motion/vehicle detection does not filter this fall detector.
    if (!fallFreefallArmed_)
    {
        if (rawNormG < DM_FALL_FREEFALL_THRESHOLD_G)
        {
            fallFreefallArmed_ = true;
            fallFreefallMs_ = nowMs;
            LOG_INFO(
                "Distance Monitor fall candidate: low_g=%.2f",
                static_cast<double>(rawNormG));
        }
        return;
    }

    const uint32_t elapsed = dmElapsedMs(nowMs, fallFreefallMs_);
    if (elapsed > DM_FALL_IMPACT_WINDOW_MS)
    {
        LOG_DEBUG(
            "Distance Monitor fall candidate expired: dt=%lums last=%.2fg",
            static_cast<unsigned long>(elapsed),
            static_cast<double>(rawNormG));
        fallFreefallArmed_ = false;
        return;
    }

    if (rawNormG > DM_FALL_IMPACT_THRESHOLD_G)
    {
        fallFreefallArmed_ = false;
        LOG_WARN(
            "Distance Monitor fall detected: impact=%.2fg dt=%lums",
            static_cast<double>(rawNormG),
            static_cast<unsigned long>(elapsed));
        triggerLocalSos(DmSosCause::FallDetected);
    }
}

void DistanceMonitorModule::sampleLocalImu(
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
    if (!imuOk)
    {
        if (imuAvailable_ || !imuWarningLogged_)
        {
            LOG_WARN("Distance Monitor IMU: unavailable; GPS stays active");
        }
        imuAvailable_ = false;
        imuWarningLogged_ = true;
        gravityReferenceValid_ = false;
        localMoving_ = true;
        localState.moving = true;

        // GNSS power is independent of IMU availability.
        return;
    }

    if (!imuAvailable_)
    {
        LOG_INFO("Distance Monitor IMU: ready (25Hz sampler)");
    }
    imuAvailable_ = true;
    imuWarningLogged_ = false;

    if (motionDetected)
    {
        lastMotionMs_ = nowMs;
        const bool motionStarted = !localMoving_;
        localMoving_ = true;

        // Movement does not control GNSS anymore. Keep the previous position
        // until a fresh GNSS solution replaces it.
        if (localPositionKind_ == DmPositionKind::CachedStationary && motionStarted)
        {
            LOG_INFO("Distance Monitor position: movement, keep cached fix");
        }
    }
    else if (dmElapsedMs(nowMs, lastMotionMs_) >= DM_STATIONARY_CONFIRM_MS)
    {
        localMoving_ = false;
    }

    localState.moving = localMoving_;
    updateHighSpeedDetection(nowMs, dynamicAccelerationMps2);
    if (!localState.isBase)
    {
        updateFallDetection(nowMs, rawNormG);
    }
}

void DistanceMonitorModule::updateLocalPosition(
    DmNodeState &localState,
    uint32_t nowMs)
{
#if !MESHTASTIC_EXCLUDE_GPS
    // ALWAYS_ON watchdog.
    //
    // Do not call gps->up()/enable() on every tick. Meshtastic keeps the GPS
    // hardware awake when gps_update_interval <= 10 s; ours is fixed at 1 s.
    // We only repair the policy if another code path changed gps_mode or the
    // configured update interval.
    if (gps != nullptr)
    {
        if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED ||
            gps->isPowerSaving() ||
            config.position.gps_update_interval != 1U)
        {
            wakeGps(1U);
        }
    }
#endif

    // Position acquisition is independent of IMU/motion state.
    if (readNewValidGpsFix())
    {
        captureGpsFix(nowMs);
    }

    if (localPositionKind_ == DmPositionKind::NoFix &&
        (!hasGpsNoFixDiagTime_ ||
         dmElapsedMs(nowMs, lastGpsNoFixDiagMs_) >=
             DM_GPS_NO_FIX_DIAG_INTERVAL_MS))
    {
        const uint32_t solutionId = currentGpsSolutionId();
#if MESHTASTIC_EXCLUDE_GPS
        LOG_INFO(
            "Distance Monitor GPS NO_FIX: policy=ALWAYS_ON available=no solution=0");
#else
        if (gps != nullptr)
        {
            const meshtastic_Position &raw = gps->p;
            const meshtastic_Position &published = localPosition;

            LOG_INFO(
                "Distance Monitor GPS RAW: policy=ALWAYS_ON flow=%s lock=%s ts=%lu time=%lu sats=%lu pdop=%lu hdop=%lu lat=%ld lon=%ld speed_valid=%s speed=%lu",
                gps->hasFlow() ? "yes" : "no",
                gps->hasLock() ? "yes" : "no",
                static_cast<unsigned long>(raw.timestamp),
                static_cast<unsigned long>(raw.time),
                static_cast<unsigned long>(raw.sats_in_view),
                static_cast<unsigned long>(raw.PDOP),
                static_cast<unsigned long>(raw.HDOP),
                static_cast<long>(raw.latitude_i),
                static_cast<long>(raw.longitude_i),
                raw.has_ground_speed ? "yes" : "no",
                static_cast<unsigned long>(raw.ground_speed));

            LOG_INFO(
                "Distance Monitor GPS PUB: ts=%lu time=%lu sats=%lu pdop=%lu hdop=%lu lat=%ld lon=%ld speed_valid=%s speed=%lu",
                static_cast<unsigned long>(published.timestamp),
                static_cast<unsigned long>(published.time),
                static_cast<unsigned long>(published.sats_in_view),
                static_cast<unsigned long>(published.PDOP),
                static_cast<unsigned long>(published.HDOP),
                static_cast<long>(published.latitude_i),
                static_cast<long>(published.longitude_i),
                published.has_ground_speed ? "yes" : "no",
                static_cast<unsigned long>(published.ground_speed));

            LOG_INFO(
                "Distance Monitor GPS NO_FIX: solution=%lu raw_has_position=%s pub_has_position=%s",
                static_cast<unsigned long>(solutionId),
                (raw.latitude_i != 0 && raw.longitude_i != 0) ? "yes" : "no",
                (published.latitude_i != 0 && published.longitude_i != 0) ? "yes" : "no");
        }
        else
        {
            LOG_INFO(
                "Distance Monitor GPS NO_FIX: policy=ALWAYS_ON available=no solution=0");
        }
#endif
        lastGpsNoFixDiagMs_ = nowMs;
        hasGpsNoFixDiagTime_ = true;
    }

    // Position-state semantics are retained for packets/UI, but they no longer
    // control GNSS power. A stationary fresh fix may become CACHED_STATIONARY;
    // the GPS itself continues running and any new valid solution is accepted.
    if (localPositionKind_ == DmPositionKind::FreshFix)
    {
        if (imuAvailable_ && !localMoving_)
        {
            localPositionKind_ = DmPositionKind::CachedStationary;
            LOG_INFO("Distance Monitor position: FRESH -> CACHED_STATIONARY (GPS remains active)");
        }
        else if (localFixAgeSeconds(nowMs) >= freshFixMaxAgeSeconds())
        {
            localPositionKind_ = DmPositionKind::NoFix;
            LOG_INFO("Distance Monitor position: FRESH -> NO_FIX (stale, GPS remains active)");
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