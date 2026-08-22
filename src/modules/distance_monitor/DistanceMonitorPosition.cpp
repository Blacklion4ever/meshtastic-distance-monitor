#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "DistanceMonitorVehicle.h"
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

namespace
{
struct DmImuCalibration
{
    NodeNum nodeNum;
    const char *name;
    bool valid;
    float offsetX;
    float gainX;
    float offsetY;
    float gainY;
    float offsetZ;
    float gainZ;
};

// Calibrations are tied to the physical nodeNum, never to BASE/TRACKER role.
// Corrected acceleration is:
//
//   a_cal = (a_raw - offset) * gain
//
// valid=false means: automatically launch the six-face calibration at boot.
// Once calibration completes, the entry is updated in RAM and valid becomes
// true. The final coefficients are logged so they can be copied back here to
// make the calibration permanent across future reboots.
static DmImuCalibration DM_IMU_CALIBRATIONS[] = {
    {
        0x8b3a4a14U,
        "base",
        true,
        -1.1447143F, 1.0388839F,
        -0.3513750F, 1.0003751F,
         0.4158636F, 1.0951267F,
    },
    {
        0xe87c9792U,
        "tracker",
        true,
         0.2682259F, 1.0301051F,
         0.7196746F, 1.0148181F,
        -0.3300820F, 1.0806831F,
    },
};

static constexpr uint16_t DM_IMU_CAL_STABLE_SAMPLES = 25U;
static constexpr float DM_IMU_CAL_MAX_SAMPLE_DELTA = 0.040F;
static constexpr float DM_IMU_CAL_MIN_AXIS_SPAN = 1.75F;
static constexpr float DM_IMU_CAL_MAX_AXIS_SPAN = 2.50F;
static constexpr uint32_t DM_IMU_CAL_PROGRESS_LOG_MS = 1500U;

// FALL PRE trial criteria. The ratio is relative to the PRE phase length, so
// changing the sampler/window size does not silently change the requirement in
// absolute sample count.
static constexpr float DM_FALL_PRE_MIN_G_THRESHOLD_G = 0.70F;
static constexpr float DM_FALL_PRE_LOW_G_THRESHOLD_G = 0.90F;
static constexpr float DM_FALL_PRE_MIN_LOW_G_RATIO = 0.25F;

struct DmImuAutoCalibrationState
{
    NodeNum nodeNum = 0U;
    bool active = false;
    bool announced = false;
    bool hasLastSample = false;

    float lastX = 0.0F;
    float lastY = 0.0F;
    float lastZ = 0.0F;

    float sumX = 0.0F;
    float sumY = 0.0F;
    float sumZ = 0.0F;
    float anchorX = 0.0F;
    float anchorY = 0.0F;
    float anchorZ = 0.0F;
    uint16_t stableSamples = 0U;

    bool haveExtrema = false;
    float minX = 0.0F;
    float maxX = 0.0F;
    float minY = 0.0F;
    float maxY = 0.0F;
    float minZ = 0.0F;
    float maxZ = 0.0F;

    uint32_t lastProgressLogMs = 0U;
};

static DmImuAutoCalibrationState dmImuAutoCal;

static DmVehicleDetectorState dmVehicleDetector;

static void dmResetVehicleCandidate()
{
    dmVehicleDetector.consecutiveHighSpeedSamples = 0U;
    dmVehicleDetector.lastHighSpeedSampleMs = 0U;
    dmVehicleDetector.hasLastHighSpeedSampleTime = false;
}

static DmImuCalibration *dmFindImuCalibration(NodeNum nodeNum)
{
    for (size_t i = 0U;
         i < sizeof(DM_IMU_CALIBRATIONS) / sizeof(DM_IMU_CALIBRATIONS[0]);
         ++i)
    {
        if (DM_IMU_CALIBRATIONS[i].nodeNum == nodeNum)
            return &DM_IMU_CALIBRATIONS[i];
    }

    return nullptr;
}

static void dmResetImuAutoCalibration(NodeNum nodeNum)
{
    dmImuAutoCal = DmImuAutoCalibrationState{};
    dmImuAutoCal.nodeNum = nodeNum;
    dmImuAutoCal.active = true;
}

static bool dmImuCalibrationInProgress(NodeNum nodeNum)
{
    return dmImuAutoCal.active && dmImuAutoCal.nodeNum == nodeNum;
}

static void dmResetStableWindow()
{
    dmImuAutoCal.sumX = 0.0F;
    dmImuAutoCal.sumY = 0.0F;
    dmImuAutoCal.sumZ = 0.0F;
    dmImuAutoCal.anchorX = 0.0F;
    dmImuAutoCal.anchorY = 0.0F;
    dmImuAutoCal.anchorZ = 0.0F;
    dmImuAutoCal.stableSamples = 0U;
}

static void dmAcceptStablePlateau(float x, float y, float z)
{
    if (!dmImuAutoCal.haveExtrema)
    {
        dmImuAutoCal.minX = dmImuAutoCal.maxX = x;
        dmImuAutoCal.minY = dmImuAutoCal.maxY = y;
        dmImuAutoCal.minZ = dmImuAutoCal.maxZ = z;
        dmImuAutoCal.haveExtrema = true;
        return;
    }

    dmImuAutoCal.minX = std::min(dmImuAutoCal.minX, x);
    dmImuAutoCal.maxX = std::max(dmImuAutoCal.maxX, x);
    dmImuAutoCal.minY = std::min(dmImuAutoCal.minY, y);
    dmImuAutoCal.maxY = std::max(dmImuAutoCal.maxY, y);
    dmImuAutoCal.minZ = std::min(dmImuAutoCal.minZ, z);
    dmImuAutoCal.maxZ = std::max(dmImuAutoCal.maxZ, z);
}

static bool dmAxisSpanReady(float span)
{
    return std::isfinite(span) &&
           span >= DM_IMU_CAL_MIN_AXIS_SPAN &&
           span <= DM_IMU_CAL_MAX_AXIS_SPAN;
}

static bool dmTryFinishImuAutoCalibration(
    DmImuCalibration &cal,
    uint32_t nowMs)
{
    if (!dmImuAutoCal.haveExtrema)
        return false;

    const float spanX = dmImuAutoCal.maxX - dmImuAutoCal.minX;
    const float spanY = dmImuAutoCal.maxY - dmImuAutoCal.minY;
    const float spanZ = dmImuAutoCal.maxZ - dmImuAutoCal.minZ;

    const bool xReady = dmAxisSpanReady(spanX);
    const bool yReady = dmAxisSpanReady(spanY);
    const bool zReady = dmAxisSpanReady(spanZ);

    if (!xReady || !yReady || !zReady)
    {
        if (dmImuAutoCal.lastProgressLogMs == 0U ||
            dmElapsedMs(nowMs, dmImuAutoCal.lastProgressLogMs) >=
                DM_IMU_CAL_PROGRESS_LOG_MS)
        {
            dmImuAutoCal.lastProgressLogMs = nowMs;
            LOG_INFO(
                "{DM@IMU_CAL} id=!%08lx progress X=%s %.3f Y=%s %.3f Z=%s %.3f",
                static_cast<unsigned long>(cal.nodeNum),
                xReady ? "OK" : "WAIT",
                static_cast<double>(spanX),
                yReady ? "OK" : "WAIT",
                static_cast<double>(spanY),
                zReady ? "OK" : "WAIT",
                static_cast<double>(spanZ));
        }
        return false;
    }

    cal.offsetX = 0.5F * (dmImuAutoCal.maxX + dmImuAutoCal.minX);
    cal.offsetY = 0.5F * (dmImuAutoCal.maxY + dmImuAutoCal.minY);
    cal.offsetZ = 0.5F * (dmImuAutoCal.maxZ + dmImuAutoCal.minZ);

    cal.gainX = 2.0F / spanX;
    cal.gainY = 2.0F / spanY;
    cal.gainZ = 2.0F / spanZ;

    if (!std::isfinite(cal.offsetX) || !std::isfinite(cal.offsetY) ||
        !std::isfinite(cal.offsetZ) || !std::isfinite(cal.gainX) ||
        !std::isfinite(cal.gainY) || !std::isfinite(cal.gainZ))
    {
        LOG_ERROR(
            "{DM@IMU_CAL} id=!%08lx invalid result; restarting calibration",
            static_cast<unsigned long>(cal.nodeNum));
        dmResetImuAutoCalibration(cal.nodeNum);
        return false;
    }

    cal.valid = true;
    dmImuAutoCal.active = false;

    LOG_INFO(
        "{DM@IMU_CAL} DONE id=!%08lx name=%s",
        static_cast<unsigned long>(cal.nodeNum),
        cal.name);
    LOG_INFO(
        "{DM@IMU_CAL} offsetX=%.7f gainX=%.7f offsetY=%.7f gainY=%.7f offsetZ=%.7f gainZ=%.7f",
        static_cast<double>(cal.offsetX),
        static_cast<double>(cal.gainX),
        static_cast<double>(cal.offsetY),
        static_cast<double>(cal.gainY),
        static_cast<double>(cal.offsetZ),
        static_cast<double>(cal.gainZ));
    LOG_INFO(
        "{DM@IMU_CAL} table: {!%08lx, valid=true, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f}",
        static_cast<unsigned long>(cal.nodeNum),
        static_cast<double>(cal.offsetX),
        static_cast<double>(cal.gainX),
        static_cast<double>(cal.offsetY),
        static_cast<double>(cal.gainY),
        static_cast<double>(cal.offsetZ),
        static_cast<double>(cal.gainZ));

    return true;
}

static bool dmUpdateImuAutoCalibration(
    DmImuCalibration &cal,
    float rawX,
    float rawY,
    float rawZ,
    uint32_t nowMs)
{
    if (!dmImuCalibrationInProgress(cal.nodeNum))
        dmResetImuAutoCalibration(cal.nodeNum);

    if (!dmImuAutoCal.announced)
    {
        dmImuAutoCal.announced = true;
        LOG_WARN(
            "{DM@IMU_CAL} START id=!%08lx name=%s - place device successively on all 6 faces",
            static_cast<unsigned long>(cal.nodeNum),
            cal.name);
    }

    if (!dmImuAutoCal.hasLastSample)
    {
        dmImuAutoCal.lastX = rawX;
        dmImuAutoCal.lastY = rawY;
        dmImuAutoCal.lastZ = rawZ;
        dmImuAutoCal.hasLastSample = true;
        dmResetStableWindow();
        return false;
    }

    const float dx = rawX - dmImuAutoCal.lastX;
    const float dy = rawY - dmImuAutoCal.lastY;
    const float dz = rawZ - dmImuAutoCal.lastZ;
    const float sampleDelta = std::sqrt(dx * dx + dy * dy + dz * dz);

    dmImuAutoCal.lastX = rawX;
    dmImuAutoCal.lastY = rawY;
    dmImuAutoCal.lastZ = rawZ;

    if (!std::isfinite(sampleDelta) ||
        sampleDelta > DM_IMU_CAL_MAX_SAMPLE_DELTA)
    {
        dmResetStableWindow();
        return false;
    }

    if (dmImuAutoCal.stableSamples == 0U)
    {
        dmImuAutoCal.anchorX = rawX;
        dmImuAutoCal.anchorY = rawY;
        dmImuAutoCal.anchorZ = rawZ;
    }

    const float anchorDx = rawX - dmImuAutoCal.anchorX;
    const float anchorDy = rawY - dmImuAutoCal.anchorY;
    const float anchorDz = rawZ - dmImuAutoCal.anchorZ;
    const float anchorDelta = std::sqrt(
        anchorDx * anchorDx +
        anchorDy * anchorDy +
        anchorDz * anchorDz);

    if (!std::isfinite(anchorDelta) ||
        anchorDelta > DM_IMU_CAL_MAX_SAMPLE_DELTA)
    {
        dmResetStableWindow();
        return false;
    }

    dmImuAutoCal.sumX += rawX;
    dmImuAutoCal.sumY += rawY;
    dmImuAutoCal.sumZ += rawZ;
    ++dmImuAutoCal.stableSamples;

    if (dmImuAutoCal.stableSamples < DM_IMU_CAL_STABLE_SAMPLES)
        return false;

    const float invCount =
        1.0F / static_cast<float>(dmImuAutoCal.stableSamples);
    const float avgX = dmImuAutoCal.sumX * invCount;
    const float avgY = dmImuAutoCal.sumY * invCount;
    const float avgZ = dmImuAutoCal.sumZ * invCount;

    dmAcceptStablePlateau(avgX, avgY, avgZ);

    LOG_INFO(
        "{DM@IMU_CAL} plateau id=!%08lx x=%.3f y=%.3f z=%.3f",
        static_cast<unsigned long>(cal.nodeNum),
        static_cast<double>(avgX),
        static_cast<double>(avgY),
        static_cast<double>(avgZ));

    dmResetStableWindow();
    return dmTryFinishImuAutoCalibration(cal, nowMs);
}

static bool dmCalibrateImuSample(
    const DmImuCalibration &cal,
    float rawX,
    float rawY,
    float rawZ,
    float &calX,
    float &calY,
    float &calZ)
{
    if (!cal.valid)
        return false;

    calX = (rawX - cal.offsetX) * cal.gainX;
    calY = (rawY - cal.offsetY) * cal.gainY;
    calZ = (rawZ - cal.offsetZ) * cal.gainZ;

    return std::isfinite(calX) &&
           std::isfinite(calY) &&
           std::isfinite(calZ);
}
} // namespace

void DistanceMonitorModule::startLocalPositionManager(uint32_t nowMs)
{
    localPositionStarted_ = true;
    lastMotionMs_ = nowMs;

#if MESHTASTIC_EXCLUDE_GPS
    LOG_WARN("{DM@GPS} unavailable: excluded from firmware");
#else
    if (gps == nullptr)
    {
        LOG_WARN("{DM@GPS} unavailable");
        return;
    }

    if (config.position.fixed_position)
    {
        LOG_WARN("{DM@GPS} fixed_position enabled");
        return;
    }

    if (config.position.gps_mode ==
        meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT)
    {
        LOG_WARN("{DM@GPS} marked NOT_PRESENT");
        return;
    }

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
            LOG_INFO("{DM@GPS} state=OFF flow=NO sats=0 dop=0 acc=INF pdop=0 hdop=0");
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
        LOG_WARN("{DM@GPS} state=OFF -> forcing enable");
    }

    if (changed)
        LOG_INFO("{DM@GPS} enabled interval=1s");

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
    const bool flow = sats != 0U || pdop != 0U || hdop != 0U;

    const bool effectiveStateOn =
        config.position.gps_mode ==
            meshtastic_Config_PositionConfig_GpsMode_ENABLED &&
        !gps->isPowerSaving();

    if (std::isfinite(accuracyM))
    {
        LOG_INFO(
            "{DM@GPS} state=%s flow=%s sats=%lu dop=%lu acc=%.0fm pdop=%lu hdop=%lu",
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
            "{DM@GPS} state=%s flow=%s sats=%lu dop=0 acc=INF pdop=%lu hdop=%lu",
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
    rawNormG = 0.0F;

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && defined(HAS_QMA6100P)
    QMA6100PSingleton *sensor = QMA6100PSingleton::GetInstance();
    if (sensor == nullptr)
        return false;

    outputData sample = {};
    if (!sensor->getAccelData(&sample))
        return false;

    if (!std::isfinite(sample.xData) ||
        !std::isfinite(sample.yData) ||
        !std::isfinite(sample.zData))
    {
        return false;
    }

    if (nodeDB == nullptr)
        return false;

    const NodeNum localNodeNum = nodeDB->getNodeNum();
    DmImuCalibration *cal = dmFindImuCalibration(localNodeNum);

    static NodeNum calibrationLogNode = 0U;
    static bool calibrationLogValid = false;

    if (cal == nullptr)
    {
        if (!calibrationLogValid || calibrationLogNode != localNodeNum)
        {
            calibrationLogNode = localNodeNum;
            calibrationLogValid = true;
            LOG_ERROR(
                "{DM@IMU_CAL} no table entry for id=!%08lx",
                static_cast<unsigned long>(localNodeNum));
        }
        return false;
    }

    // valid=false is not an error. It deliberately starts automatic six-face
    // calibration. While calibrating, FALL/MOVING are not fed with raw values.
    if (!cal->valid)
    {
        const bool justCompleted = dmUpdateImuAutoCalibration(
            *cal,
            sample.xData,
            sample.yData,
            sample.zData,
            millis());

        if (!justCompleted)
            return false;

        // A freshly completed calibration starts from a clean gravity vector.
        gravityReferenceValid_ = false;
    }

    if (!calibrationLogValid || calibrationLogNode != localNodeNum)
    {
        calibrationLogNode = localNodeNum;
        calibrationLogValid = true;
        LOG_INFO(
            "{DM@IMU} calibration id=!%08lx name=%s mode=6face",
            static_cast<unsigned long>(localNodeNum),
            cal->name);
    }

    float ax = 0.0F;
    float ay = 0.0F;
    float az = 0.0F;
    if (!dmCalibrateImuSample(
            *cal,
            sample.xData,
            sample.yData,
            sample.zData,
            ax,
            ay,
            az))
    {
        return false;
    }

    const float calibratedNormG = std::sqrt(
        ax * ax + ay * ay + az * az);

    if (!std::isfinite(calibratedNormG))
        return false;

    // FALL works on calibrated absolute acceleration magnitude:
    // static ~= 1 g, free fall -> 0 g, impact > 1 g.
    rawNormG = calibratedNormG;

    // MOVING/STATIONARY uses the calibrated vector and a slowly tracked
    // gravity reference.
    if (!gravityReferenceValid_)
    {
        gravityX_ = ax;
        gravityY_ = ay;
        gravityZ_ = az;
        gravityReferenceValid_ = true;
        motionDetected = false;
        return true;
    }

    gravityX_ += DM_IMU_GRAVITY_ALPHA * (ax - gravityX_);
    gravityY_ += DM_IMU_GRAVITY_ALPHA * (ay - gravityY_);
    gravityZ_ += DM_IMU_GRAVITY_ALPHA * (az - gravityZ_);

    const float dynamicX = ax - gravityX_;
    const float dynamicY = ay - gravityY_;
    const float dynamicZ = az - gravityZ_;
    const float dynamicNormG = std::sqrt(
        dynamicX * dynamicX +
        dynamicY * dynamicY +
        dynamicZ * dynamicZ);

    motionDetected =
        dynamicNormG >= DM_IMU_MOTION_THRESHOLD_G;

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

    const bool acquired =
        localPositionKind_ == DmPositionKind::NoFix;

    localFix_ = localPosition;
    localFixMs_ = nowMs;
    localPositionKind_ = DmPositionKind::FreshFix;

    if (acquired)
    {
        const uint32_t dop =
            dmBestDop(localFix_.PDOP, localFix_.HDOP);
        const double accuracyM = dmAccuracyMetersFromDop(dop);

        LOG_INFO(
            "{DM@GPS} fix sats=%lu dop=%lu acc=%.0fm pdop=%lu hdop=%lu",
            static_cast<unsigned long>(localFix_.sats_in_view),
            static_cast<unsigned long>(dop),
            accuracyM,
            static_cast<unsigned long>(localFix_.PDOP),
            static_cast<unsigned long>(localFix_.HDOP));
    }

    // Vehicle detection is tracker-only. The base still records its GNSS fix,
    // but driving with the base must never create an IN_VEHICLE state or SOS.
    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return;

    if (nodeStates_[localIndex].isBase)
    {
        dmResetVehicleCandidate();
        highSpeedSosLatched_ = false;
        highSpeedBelowSinceMs_ = 0U;
        return;
    }

    if (!localFix_.has_ground_speed)
    {
        dmResetVehicleCandidate();
        return;
    }

    const float speedKmh =
        static_cast<float>(localFix_.ground_speed) * 3.6F;

    const uint32_t vehicleDop =
        dmBestDop(localFix_.PDOP, localFix_.HDOP);
    const double vehicleAccuracyM =
        dmAccuracyMetersFromDop(vehicleDop);
    const bool vehicleGpsQualityOk =
        localFix_.sats_in_view >= DM_VEHICLE_MIN_SATS &&
        vehicleDop != 0U &&
        std::isfinite(vehicleAccuracyM) &&
        vehicleAccuracyM <= DM_VEHICLE_MAX_ACCURACY_M;

    // Never confirm or clear a vehicle state from a poor-quality GNSS sample.
    // If an above-threshold candidate was being accumulated, break the streak.
    if (!vehicleGpsQualityOk)
    {
        if (speedKmh > DM_HIGH_SPEED_THRESHOLD_KMH)
        {
            LOG_DEBUG(
                "{DM@VEHICLE} reject speed=%.1fkmh sats=%lu dop=%lu acc=%.0fm",
                static_cast<double>(speedKmh),
                static_cast<unsigned long>(localFix_.sats_in_view),
                static_cast<unsigned long>(vehicleDop),
                vehicleAccuracyM);
        }
        dmResetVehicleCandidate();
        return;
    }

    if (speedKmh > DM_HIGH_SPEED_THRESHOLD_KMH)
    {
        highSpeedBelowSinceMs_ = 0U;

        if (highSpeedSosLatched_)
            return;

        // A long gap means the previous high-speed sample cannot contribute to
        // the same confirmation sequence. With 1 Hz GNSS, 2500 ms comfortably
        // accepts normal scheduling jitter while rejecting stale samples.
        if (!dmVehicleDetector.hasLastHighSpeedSampleTime ||
            dmElapsedMs(nowMs, dmVehicleDetector.lastHighSpeedSampleMs) >
                DM_VEHICLE_CONFIRM_MAX_GAP_MS)
        {
            dmVehicleDetector.consecutiveHighSpeedSamples = 0U;
        }

        dmVehicleDetector.lastHighSpeedSampleMs = nowMs;
        dmVehicleDetector.hasLastHighSpeedSampleTime = true;
        if (dmVehicleDetector.consecutiveHighSpeedSamples <
            DM_VEHICLE_CONFIRM_SAMPLES)
        {
            ++dmVehicleDetector.consecutiveHighSpeedSamples;
        }

        LOG_INFO(
            "{DM@VEHICLE} candidate speed=%.1fkmh confirm=%u/%u sats=%lu dop=%lu acc=%.0fm",
            static_cast<double>(speedKmh),
            static_cast<unsigned>(dmVehicleDetector.consecutiveHighSpeedSamples),
            static_cast<unsigned>(DM_VEHICLE_CONFIRM_SAMPLES),
            static_cast<unsigned long>(localFix_.sats_in_view),
            static_cast<unsigned long>(vehicleDop),
            vehicleAccuracyM);

        if (dmVehicleDetector.consecutiveHighSpeedSamples <
            DM_VEHICLE_CONFIRM_SAMPLES)
        {
            return;
        }

        // triggerLocalSos() can legitimately refuse a non-manual SOS during the
        // global SOS rearm interval. Do not latch IN_VEHICLE until a new SOS was
        // actually armed; keep the confirmed candidate alive so the next good
        // 1 Hz fix retries automatically.
        const uint32_t previousSosSequence = pendingSos_.sequence;
        triggerLocalSos(DmSosCause::HighSpeedMovement);

        const bool vehicleSosArmed =
            pendingSos_.active &&
            pendingSos_.cause == DmSosCause::HighSpeedMovement &&
            pendingSos_.sequence != previousSosSequence;

        if (!vehicleSosArmed)
        {
            LOG_INFO(
                "{DM@VEHICLE} confirmed speed=%.1fkmh SOS deferred/retry",
                static_cast<double>(speedKmh));
            return;
        }

        highSpeedSosLatched_ = true;
        dmResetVehicleCandidate();
        LOG_WARN(
            "{DM@VEHICLE} CONFIRMED speed=%.1fkmh -> SOS",
            static_cast<double>(speedKmh));
        return;
    }

    // Any good-quality sample at or below the entry threshold breaks an
    // unconfirmed high-speed streak.
    dmResetVehicleCandidate();

    if (!highSpeedSosLatched_)
    {
        highSpeedBelowSinceMs_ = 0U;
        return;
    }

    // Hysteresis: once IN_VEHICLE is latched, speeds between 15 and 20 km/h
    // neither re-arm nor clear it. Clearing requires 10 s continuously below
    // the lower threshold using good-quality GNSS samples.
    if (speedKmh > DM_VEHICLE_CLEAR_THRESHOLD_KMH)
    {
        highSpeedBelowSinceMs_ = 0U;
        return;
    }

    if (highSpeedBelowSinceMs_ == 0U)
    {
        highSpeedBelowSinceMs_ = nowMs;
        LOG_INFO(
            "{DM@VEHICLE} clear candidate speed=%.1fkmh",
            static_cast<double>(speedKmh));
        return;
    }

    if (dmElapsedMs(nowMs, highSpeedBelowSinceMs_) >=
        DM_HIGH_SPEED_CLEAR_MS)
    {
        highSpeedSosLatched_ = false;
        highSpeedBelowSinceMs_ = 0U;
        LOG_INFO(
            "{DM@VEHICLE} CLEAR speed=%.1fkmh",
            static_cast<double>(speedKmh));
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
    uint32_t nowMs) const
{
    (void)nowMs;

    localState.positionKind = localPositionKind_;
    localState.moving = localMoving_;

    if (localPositionKind_ == DmPositionKind::NoFix)
    {
        // Preserve the last-known coordinates for passive display. The
        // positionKind flag remains NO_FIX, so distance logic cannot treat
        // these coordinates as a fresh GNSS measurement.
        localState.positionDop = 0U;
        localState.positionAccuracyMeters =
            std::numeric_limits<float>::infinity();
        return;
    }

    localState.latitudeI = localFix_.latitude_i;
    localState.longitudeI = localFix_.longitude_i;
    localState.positionDop = static_cast<uint16_t>(
        std::min<uint32_t>(
            dmBestDop(localFix_.PDOP, localFix_.HDOP),
            UINT16_MAX));
    localState.positionAccuracyMeters = static_cast<float>(
        dmAccuracyMetersFromDop(localState.positionDop));
}

size_t DistanceMonitorModule::fallBufferIndexFromAge(size_t age) const
{
    if (DM_MAX_ROLL_BUFFER == 0U)
        return 0U;

    age %= DM_MAX_ROLL_BUFFER;
    return (fallBufferHead_ + DM_MAX_ROLL_BUFFER - age) %
           DM_MAX_ROLL_BUFFER;
}

DmFallPhaseStats DistanceMonitorModule::analyzeFallPhase(
    size_t startAge,
    size_t sampleCount) const
{
    DmFallPhaseStats stats{};
    if (sampleCount == 0U ||
        fallBufferCount_ < DM_MAX_ROLL_BUFFER)
    {
        return stats;
    }

    double sum = 0.0;
    double sumSq = 0.0;

    for (size_t offset = 0U; offset < sampleCount; ++offset)
    {
        const size_t index =
            fallBufferIndexFromAge(startAge + offset);
        const double magnitudeG =
            static_cast<double>(fallMagnitudeBuffer_[index]);

        sum += magnitudeG;
        sumSq += magnitudeG * magnitudeG;
    }

    const double count = static_cast<double>(sampleCount);
    const double mean = sum / count;
    const double meanSq = sumSq / count;
    const double variance =
        std::max(0.0, meanSq - mean * mean);

    stats.meanG = static_cast<float>(mean);
    stats.stdG = static_cast<float>(std::sqrt(variance));
    stats.rmsG = static_cast<float>(std::sqrt(meanSq));
    return stats;
}

void DistanceMonitorModule::logFallAnalysis(
    uint32_t nowMs,
    const DmFallPhaseStats &pre,
    const DmFallPhaseStats &impact,
    const DmFallPhaseStats &post,
    bool prePass,
    bool impactPass,
    bool postPass)
{
    const uint8_t passMask =
        (prePass ? 0x01U : 0U) |
        (impactPass ? 0x02U : 0U) |
        (postPass ? 0x04U : 0U);

    if (passMask == 0U)
    {
        lastFallPassMask_ = 0U;
        return;
    }

    const bool logDue =
        passMask != lastFallPassMask_ ||
        !hasFallStatsLogTime_ ||
        dmElapsedMs(nowMs, lastFallStatsLogMs_) >=
            DM_FALL_LOG_INTERVAL_MS;

    lastFallPassMask_ = passMask;
    if (!logDue)
        return;

    lastFallStatsLogMs_ = nowMs;
    hasFallStatsLogTime_ = true;

    const size_t preStartAge =
        DM_FALL_POST_SAMPLES + DM_FALL_IMPACT_SAMPLES;
    float preMinG = std::numeric_limits<float>::infinity();
    size_t preLowGCount = 0U;
    for (size_t offset = 0U; offset < DM_FALL_PRE_SAMPLES; ++offset)
    {
        const size_t index =
            fallBufferIndexFromAge(preStartAge + offset);
        const float magnitudeG = fallMagnitudeBuffer_[index];
        preMinG = std::min(preMinG, magnitudeG);
        if (magnitudeG < DM_FALL_PRE_LOW_G_THRESHOLD_G)
            ++preLowGCount;
    }
    const float preLowGRatio =
        DM_FALL_PRE_SAMPLES > 0U
            ? static_cast<float>(preLowGCount) /
                  static_cast<float>(DM_FALL_PRE_SAMPLES)
            : 0.0F;

    LOG_INFO(
        "{DM@FALL} PRE  [%s] mean=%.2f std=%.2f rms=%.2f min=%.2f low=%.0f%% (%u/%u)",
        prePass ? "PASS" : "NO",
        static_cast<double>(pre.meanG),
        static_cast<double>(pre.stdG),
        static_cast<double>(pre.rmsG),
        static_cast<double>(preMinG),
        static_cast<double>(preLowGRatio * 100.0F),
        static_cast<unsigned>(preLowGCount),
        static_cast<unsigned>(DM_FALL_PRE_SAMPLES));

    LOG_INFO(
        "{DM@FALL} IMP  [%s] mean=%.2f std=%.2f rms=%.2f",
        impactPass ? "PASS" : "NO",
        static_cast<double>(impact.meanG),
        static_cast<double>(impact.stdG),
        static_cast<double>(impact.rmsG));

    LOG_INFO(
        "{DM@FALL} POST [%s] mean=%.2f std=%.2f rms=%.2f",
        postPass ? "PASS" : "NO",
        static_cast<double>(post.meanG),
        static_cast<double>(post.stdG),
        static_cast<double>(post.rmsG));
}

void DistanceMonitorModule::handleLocalFallDetected(
    DmNodeState &localState,
    uint32_t nowMs)
{
    if (localFallDetected_)
        return;

    localFallDetected_ = true;
    localFallDetectedMs_ = nowMs;

    LOG_WARN(
        "{DM@FALL} DETECTED role=%s silent=%s",
        localState.isBase ? "BASE" : "TRACKER",
        DM_ALARM_AUDIO_SILENT ? "YES" : "NO");

    if (localState.isBase)
    {
        if (!DM_ALARM_AUDIO_SILENT && !audio_.startSos())
            LOG_ERROR("{DM@Alarm} Cause=FALL buzzer unavailable");
        return;
    }

    triggerLocalSos(DmSosCause::FallDetected);
}

void DistanceMonitorModule::updateFallDetection(
    DmNodeState &localState,
    uint32_t nowMs,
    float rawNormG)
{
    if (localFallDetected_ &&
        !localState.isBase &&
        dmElapsedMs(nowMs, localFallDetectedMs_) >=
            DM_SOS_REARM_MS)
    {
        localFallDetected_ = false;
    }

    if (fallBufferCount_ == 0U)
    {
        fallBufferHead_ = 0U;
    }
    else
    {
        fallBufferHead_ =
            (fallBufferHead_ + 1U) % DM_MAX_ROLL_BUFFER;
    }

    fallMagnitudeBuffer_[fallBufferHead_] = rawNormG;

    if (fallBufferCount_ < DM_MAX_ROLL_BUFFER)
        ++fallBufferCount_;

    if (fallBufferCount_ < DM_MAX_ROLL_BUFFER)
        return;

    // Logical time is measured backward from the dynamic ring head:
    //
    // now                                                   past
    // 0 s                                                    3 s
    // |---------------------------------------------------------|
    // [      POST      ][ IMPACT ][           PRE               ]
    // ^                  ^        ^
    // age 0              age POST age POST+IMPACT
    const size_t postStartAge = 0U;
    const size_t impactStartAge = DM_FALL_POST_SAMPLES;
    const size_t preStartAge =
        DM_FALL_POST_SAMPLES + DM_FALL_IMPACT_SAMPLES;

    const DmFallPhaseStats post = analyzeFallPhase(
        postStartAge,
        DM_FALL_POST_SAMPLES);

    const DmFallPhaseStats impact = analyzeFallPhase(
        impactStartAge,
        DM_FALL_IMPACT_SAMPLES);

    const DmFallPhaseStats pre = analyzeFallPhase(
        preStartAge,
        DM_FALL_PRE_SAMPLES);

    // PRE is intentionally event-based rather than mean-based. A fall must
    // contain both a clear low-g dip and a sufficient proportion of the PRE
    // window below the broader low-g threshold. The proportion is expressed as
    // a ratio of PRE, never as a hard-coded sample count.
    float preMinG = std::numeric_limits<float>::infinity();
    size_t preLowGCount = 0U;
    for (size_t offset = 0U; offset < DM_FALL_PRE_SAMPLES; ++offset)
    {
        const size_t index =
            fallBufferIndexFromAge(preStartAge + offset);
        const float magnitudeG = fallMagnitudeBuffer_[index];
        preMinG = std::min(preMinG, magnitudeG);
        if (magnitudeG < DM_FALL_PRE_LOW_G_THRESHOLD_G)
            ++preLowGCount;
    }
    const float preLowGRatio =
        DM_FALL_PRE_SAMPLES > 0U
            ? static_cast<float>(preLowGCount) /
                  static_cast<float>(DM_FALL_PRE_SAMPLES)
            : 0.0F;

    const bool prePass =
        preMinG < DM_FALL_PRE_MIN_G_THRESHOLD_G &&
        preLowGRatio >= DM_FALL_PRE_MIN_LOW_G_RATIO;

    const bool impactPass =
        impact.rmsG >= DM_FALL_IMPACT_MIN_RMS_G;

    const bool postPass =
        post.stdG <= DM_FALL_POST_MAX_STD_G;

    // POST is naturally true at rest. Only emit detailed FALL statistics when
    // PRE or IMPACT is interesting, otherwise the stationary state floods logs.
    if (impactPass || prePass)
    {
        logFallAnalysis(
            nowMs,
            pre,
            impact,
            post,
            prePass,
            impactPass,
            postPass);
    }

    if (prePass && impactPass && postPass)
        handleLocalFallDetected(localState, nowMs);
}

void DistanceMonitorModule::sampleLocalImu(
    DmNodeState &localState,
    uint32_t nowMs)
{
    bool motionDetected = false;
    float rawNormG = 1.0F;

    const bool imuOk =
        readImuSample(motionDetected, rawNormG);

    if (!imuOk)
    {
#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C && defined(HAS_QMA6100P)
        const NodeNum localNodeNum =
            nodeDB != nullptr ? nodeDB->getNodeNum() : 0U;
        const bool calibrating =
            localNodeNum != 0U &&
            dmImuCalibrationInProgress(localNodeNum);
#else
        const bool calibrating = false;
#endif

        if (!calibrating)
        {
            if (imuAvailable_ || !imuWarningLogged_)
                LOG_WARN("{DM@IMU} unavailable");
            imuWarningLogged_ = true;
        }
        else
        {
            // Calibration requires static plateaus. Keep the node in MOVING
            // state and keep FALL completely disabled until valid=true.
            imuWarningLogged_ = false;
        }

        imuAvailable_ = false;
        gravityReferenceValid_ = false;
        fallBufferCount_ = 0U;
        fallBufferHead_ = 0U;
        lastFallPassMask_ = 0U;
        hasFallStatsLogTime_ = false;

        localMoving_ = true;
        localState.moving = true;
        return;
    }

    if (!imuAvailable_)
        LOG_INFO("{DM@IMU} ready calibration=node-table");

    imuAvailable_ = true;
    imuWarningLogged_ = false;

    if (motionDetected)
    {
        lastMotionMs_ = nowMs;
        localMoving_ = true;
    }
    else if (dmElapsedMs(nowMs, lastMotionMs_) >=
             DM_STATIONARY_CONFIRM_MS)
    {
        localMoving_ = false;
    }

    localState.moving = localMoving_;

    // Same FALL detector on base and tracker, but only after calibration is
    // valid for this physical node.
    updateFallDetection(localState, nowMs, rawNormG);
}

void DistanceMonitorModule::updateLocalPosition(
    DmNodeState &localState,
    uint32_t nowMs)
{
    // Reassert every control tick. ensureGpsAlwaysOn() also owns the periodic
    // functional diagnostic through lastGpsFunctionalCheckMs_.
    ensureGpsAlwaysOn(nowMs, false);

    if (readNewUsableGpsPosition())
        captureGpsFix(nowMs);

    if (localPositionKind_ == DmPositionKind::FreshFix &&
        localFixAgeSeconds(nowMs) >= freshFixMaxAgeSeconds())
    {
        localPositionKind_ = DmPositionKind::NoFix;
        LOG_INFO("{DM@GPS} no fix");
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
    pendingSos_.targetNode =
        runtimeConfig_.members[baseIndex].nodeNum;
    pendingSos_.sequence = sequence;
    pendingSos_.cause = cause;
    pendingSos_.lastTxMs = nowMs;
    lastSosTriggerMs_ = nowMs;

    LOG_WARN(
        "{DM@Alarm} Cause=%s silent=%s",
        dmSosCauseName(cause),
        DM_ALARM_AUDIO_SILENT ? "YES" : "NO");

    sendSos(
        pendingSos_.targetNode,
        pendingSos_.sequence,
        pendingSos_.cause);
}
