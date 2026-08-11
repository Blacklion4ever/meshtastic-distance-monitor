#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "FSCommon.h"
#include "SPILock.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "configuration.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

DistanceMonitorModule *distanceMonitorModule = nullptr;

namespace
{
bool deadlinePending(uint32_t nowMs, uint32_t deadlineMs)
{
    return deadlineMs != 0U &&
           static_cast<int32_t>(nowMs - deadlineMs) < 0;
}

static constexpr uint32_t DM_RSSI_FILE_MAGIC = 0x52525344U;
static constexpr uint16_t DM_RSSI_FILE_VERSION = 1U;

struct DmRssiPersistStats
{
    uint32_t count;
    float mean;
    float m2;
};

struct DmRssiPersistBin
{
    DmRssiPersistStats bt;
    DmRssiPersistStats tb;
};

struct DmRssiPersistRecord
{
    uint32_t magic;
    uint16_t version;
    uint16_t binCount;
    uint32_t nodeNum;
    uint32_t generation;
    uint8_t flags;
    uint8_t reserved[3];
    float bestBtDbm;
    float bestTbDbm;
    DmRssiPersistBin bins[DM_RSSI_CALIBRATION_BIN_COUNT];
    uint32_t crc32;
};

uint32_t dmCrc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^
                  (0xEDB88320U & static_cast<uint32_t>(
                       -(static_cast<int32_t>(crc & 1U))));
        }
    }
    return ~crc;
}

void dmRssiFilePath(char *buffer, size_t size, uint32_t nodeNum, char slot)
{
    std::snprintf(buffer, size, "/prefs/dm_rssi_%08lx_%c.bin",
                  static_cast<unsigned long>(nodeNum), slot);
}

float effectiveAlertRatioFromBand(DmDistanceBand band)
{
    switch (band)
    {
    case DmDistanceBand::Warning:
        return 0.80F;
    case DmDistanceBand::Beyond:
        return 1.00F;
    case DmDistanceBand::Near:
    case DmDistanceBand::Mid:
    case DmDistanceBand::Unknown:
    default:
        return 0.0F;
    }
}
}

DistanceMonitorModule::DistanceMonitorModule()
    : SinglePortModule("distance_monitor", meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("DistanceMonitor")
{
    loadDefaultConfiguration();
    initializeIfNeeded();
}

void DistanceMonitorModule::setup()
{
    initializeIfNeeded();
}

void DistanceMonitorModule::initializeIfNeeded()
{
    if (moduleInitialized_)
    {
        return;
    }

    bootMs_ = millis();
    uint32_t localNodeNum = nodeDB != nullptr ? nodeDB->getNodeNum() : 0U;

    bootSessionId_ = static_cast<uint32_t>(micros()) ^
                     (bootMs_ << 11U) ^ localNodeNum ^ 0xD157A9C5U;
    if (bootSessionId_ == 0U)
    {
        bootSessionId_ = 1U;
    }

    const uint8_t maximumReport =
        dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);
    localAppliedIntervalSec_ = maximumReport;
    localDesiredGpsIntervalSec_ = maximumReport;

    applyRuntimeProfile();
    audio_.setup();

    LOG_INFO(
        "Distance Monitor initialized: firmware=%s protocol=%u members=%u "
        "imu=%lums control=%lums dmax=%.1fm report=%u..%us beacon=%lus session=%08lx",
        DM_FIRMWARE_VERSION,
        static_cast<unsigned>(DM_PROTOCOL_VERSION),
        static_cast<unsigned>(runtimeConfig_.memberCount),
        static_cast<unsigned long>(DM_TICK_INTERVAL_MS),
        static_cast<unsigned long>(DM_CONTROL_INTERVAL_MS),
        static_cast<double>(runtimeConfig_.maxDistanceMeters),
        static_cast<unsigned>(DM_MIN_REPORT_INTERVAL_S),
        static_cast<unsigned>(maximumReport),
        static_cast<unsigned long>(DM_BASE_BEACON_INTERVAL_MS / 1000U),
        static_cast<unsigned long>(bootSessionId_));

    moduleInitialized_ = true;
    if (!audio_.isAvailable())
    {
        LOG_ERROR("Distance Monitor initialization failed: buzzer unavailable");
    }
    setIntervalFromNow(DM_TICK_INTERVAL_MS);
}

int32_t DistanceMonitorModule::runOnce()
{
    initializeIfNeeded();

    if (nodeDB == nullptr || runtimeConfig_.memberCount == 0U)
    {
        return DM_TICK_INTERVAL_MS;
    }

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
    {
        return DM_TICK_INTERVAL_MS;
    }

    const uint32_t nowMs = millis();
    DmNodeState &localState = nodeStates_[localIndex];
    localState.isLocal = true;

    if (!localPositionStarted_)
    {
        startLocalPositionManager(nowMs);
    }

    sampleLocalImu(localState, nowMs);
    if (localState.isBase)
    {
        processSearchMode(localIndex, nowMs);
    }
    else
    {
        processTrackerSearchMode(localIndex, nowMs);
    }

    if (hasControlTickTime_ &&
        dmElapsedMs(nowMs, lastControlTickMs_) < DM_CONTROL_INTERVAL_MS)
    {
        return DM_TICK_INTERVAL_MS;
    }
    lastControlTickMs_ = nowMs;
    hasControlTickTime_ = true;

    localState.batteryPercent = currentBatteryPercent();
    updateLocalPosition(localState, nowMs);
    loadRssiCalibrationIfNeeded(localIndex);

    processPairing(localIndex, nowMs);
    if (localState.isBase)
    {
        processBase(localIndex, nowMs);
    }
    else
    {
        processTracker(localIndex, nowMs);
    }

    if (!hasSummaryLogTime_ ||
        dmElapsedMs(nowMs, lastSummaryLogMs_) >= DM_SUMMARY_INTERVAL_MS)
    {
        lastSummaryLogMs_ = nowMs;
        hasSummaryLogTime_ = true;
        logSummary(localIndex, nowMs);
    }

    return DM_TICK_INTERVAL_MS;
}

void DistanceMonitorModule::loadDefaultConfiguration()
{
    runtimeConfig_ = DmRuntimeConfig{};
    runtimeConfig_.maxDistanceMeters = DM_MAX_DISTANCE_M;
    runtimeConfig_.maxDop = DM_MAX_DOP;
    runtimeConfig_.memberCount = DM_DEFAULT_MEMBER_COUNT;

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        runtimeConfig_.members[index].nodeNum =
            DM_DEFAULT_MEMBERS[index].nodeNum;
        runtimeConfig_.members[index].isBase =
            DM_DEFAULT_MEMBERS[index].isBase;

        nodeStates_[index] = DmNodeState{};
        nodeStates_[index].nodeNum =
            DM_DEFAULT_MEMBERS[index].nodeNum;
        nodeStates_[index].isBase =
            DM_DEFAULT_MEMBERS[index].isBase;
        nodeStates_[index].batteryPercent =
            DM_BATTERY_UNKNOWN;
    }
}

void DistanceMonitorModule::applyRuntimeProfile()
{
    config.device.role = meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
    config.device.buzzer_mode = meshtastic_Config_DeviceConfig_BuzzerMode_DISABLED;
    config.device.disable_triple_click = true;
    config.device.led_heartbeat_disabled = false;

    config.position.position_broadcast_smart_enabled = false;
    config.position.position_broadcast_secs = DM_GPS_SLEEP_UPDATE_INTERVAL_S;

    moduleConfig.telemetry.device_telemetry_enabled = false;
    moduleConfig.mqtt.enabled = false;

    moduleConfig.external_notification.enabled = false;
}

bool DistanceMonitorModule::findMemberIndex(
    uint32_t nodeNum,
    size_t &index) const
{
    for (size_t candidate = 0U;
         candidate < runtimeConfig_.memberCount;
         ++candidate)
    {
        if (runtimeConfig_.members[candidate].nodeNum == nodeNum)
        {
            index = candidate;
            return true;
        }
    }

    return false;
}

bool DistanceMonitorModule::findLocalIndex(size_t &index) const
{
    if (nodeDB == nullptr)
    {
        return false;
    }

    return findMemberIndex(nodeDB->getNodeNum(), index);
}

bool DistanceMonitorModule::findBaseIndex(size_t &index) const
{
    for (size_t candidate = 0U;
         candidate < runtimeConfig_.memberCount;
         ++candidate)
    {
        if (runtimeConfig_.members[candidate].isBase)
        {
            index = candidate;
            return true;
        }
    }

    return false;
}

uint32_t DistanceMonitorModule::uptimeSeconds(uint32_t nowMs) const
{
    return dmElapsedMs(nowMs, bootMs_) / 1000U;
}

uint8_t DistanceMonitorModule::currentBatteryPercent() const
{
    if (powerStatus == nullptr || !powerStatus->getHasBattery())
    {
        return DM_BATTERY_UNKNOWN;
    }

    return std::min<uint8_t>(
        100U,
        powerStatus->getBatteryChargePercent());
}

void DistanceMonitorModule::processPairing(
    size_t localIndex,
    uint32_t nowMs)
{
    DmNodeState &localState = nodeStates_[localIndex];

    if (!localState.isBase)
    {
        return;
    }

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex || nodeStates_[index].isBase)
        {
            continue;
        }

        DmNodeState &remote = nodeStates_[index];
        if (remote.paired)
        {
            continue;
        }

        if (!remote.hasRemoteSession || !remote.hasRemoteUptime)
        {
            if (!remote.hasHandshakeTxTime ||
                dmElapsedMs(nowMs, remote.lastHandshakeTxMs) >=
                    DM_HANDSHAKE_RETRY_MS)
            {
                if (sendAliveRequest(remote.nodeNum))
                {
                    remote.lastHandshakeTxMs = nowMs;
                    remote.hasHandshakeTxTime = true;
                    remote.radioState = DmRadioState::Pairing;
                }
            }
            continue;
        }

        if (uptimeSeconds(nowMs) * 1000U <
            DM_PAIR_CONFIRM_DELAY_MS)
        {
            continue;
        }

        if (remote.hasHandshakeTxTime &&
            dmElapsedMs(nowMs, remote.lastHandshakeTxMs) <
                DM_HANDSHAKE_RETRY_MS)
        {
            continue;
        }

        const bool trackerJustBooted =
            remote.remoteUptimeSeconds <=
            DM_PAIR_REMOTE_BOOT_WINDOW_S;

        if (sendPairConfirm(
                remote.nodeNum,
                remote.remoteSessionId,
                trackerJustBooted))
        {

            remote.pairedLocalSessionId = bootSessionId_;
            remote.pairedRemoteSessionId = remote.remoteSessionId;
            remote.lastHandshakeTxMs = nowMs;
            remote.hasHandshakeTxTime = true;
            remote.radioState = DmRadioState::Pairing;
        }
    }
}

void DistanceMonitorModule::processBase(
    size_t localIndex,
    uint32_t nowMs)
{
    bool hasActiveSos = false;
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index != localIndex && nodeStates_[index].sosActive)
        {
            hasActiveSos = true;
            break;
        }
    }

    if (!searchModeActive_ &&
        !hasActiveSos &&
        (!hasBaseBeaconTxTime_ ||
         dmElapsedMs(nowMs, lastBaseBeaconTxMs_) >= DM_BASE_BEACON_INTERVAL_MS))
    {
        if (sendBaseBeacon())
        {
            lastBaseBeaconTxMs_ = nowMs;
            hasBaseBeaconTxTime_ = true;
        }
    }

    for (size_t remoteIndex = 0U; remoteIndex < runtimeConfig_.memberCount; ++remoteIndex)
    {
        if (remoteIndex == localIndex || nodeStates_[remoteIndex].isBase)
            continue;
        processRemoteMember(localIndex, remoteIndex, nowMs);
    }

    updateBaseGpsDemand(localIndex);
    processNotifications(nowMs);
    updateBaseAudio(localIndex, nowMs);
}

void DistanceMonitorModule::processTracker(
    size_t,
    uint32_t nowMs)
{
    size_t baseIndex = 0U;
    if (!findBaseIndex(baseIndex))
    {
        return;
    }

    if (!trackerSearchActive_)
    {
        const uint32_t reportPeriodMs =
            static_cast<uint32_t>(
                std::max<uint8_t>(
                    localAppliedIntervalSec_,
                    DM_MIN_REPORT_INTERVAL_S)) *
            1000U;

        if (forcePositionReport_ ||
            !hasPositionReportTxTime_ ||
            dmElapsedMs(nowMs, lastPositionReportTxMs_) >= reportPeriodMs)
        {
            size_t localIndex = 0U;
            if (findLocalIndex(localIndex) &&
                sendPositionReport(
                    runtimeConfig_.members[baseIndex].nodeNum,
                    nodeStates_[localIndex],
                    nowMs))
            {
                lastPositionReportTxMs_ = nowMs;
                hasPositionReportTxTime_ = true;
                forcePositionReport_ = false;
            }
        }
    }

    processPendingSos(nowMs);
}

void DistanceMonitorModule::processRemoteMember(
    size_t localIndex,
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &remote = nodeStates_[remoteIndex];

    processLinkState(remoteIndex, nowMs);
    if (remote.radioState == DmRadioState::Lost ||
        !remote.hasPositionReportRxTime)
    {
        return;
    }

    evaluateRemoteDistance(localIndex, remoteIndex, nowMs);

    if (remote.lastPositionReportSequence == 0U ||
        remote.lastIntervalEvaluationSequence ==
            remote.lastPositionReportSequence)
    {
        return;
    }

    uint8_t desired = remote.appliedIntervalSec;
    if (desired < DM_MIN_REPORT_INTERVAL_S)
    {
        desired =
            dmComputeMaxReportIntervalSec(
                runtimeConfig_.maxDistanceMeters);
    }

    if (remote.distanceSource == DmDistanceSource::Gps)
    {
        desired = dmComputeReportIntervalSec(
            remote.lastValidDistanceRatio,
            runtimeConfig_.maxDistanceMeters);
    }
    else if (remote.distanceSource == DmDistanceSource::Rssi)
    {
        switch (remote.rssiEstimate.band)
        {
        case DmDistanceBand::Beyond:
            desired = DM_MIN_REPORT_INTERVAL_S;
            break;
        case DmDistanceBand::Warning:
            desired = dmComputeReportIntervalSec(
                0.80F,
                runtimeConfig_.maxDistanceMeters);
            break;
        case DmDistanceBand::Near:
        case DmDistanceBand::Mid:
        case DmDistanceBand::Unknown:
        default:
            break;
        }
    }

    remote.desiredIntervalSec = desired;
    remote.lastIntervalEvaluationSequence =
        remote.lastPositionReportSequence;

    if (remote.appliedIntervalSec != desired &&
        remote.hasRemoteSession)
    {
        sendSetPositionInterval(
            remote.nodeNum,
            remote.remoteSessionId,
            desired);
    }
}

void DistanceMonitorModule::processLinkState(
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &remote = nodeStates_[remoteIndex];
    const bool useSearchTraffic =
        searchModeActive_ &&
        remoteIndex == searchTargetIndex_ &&
        remote.hasAnyPacketRxTime;

    if (!useSearchTraffic && !remote.hasPositionReportRxTime)
    {
        return;
    }

    const uint32_t lastRxMs = useSearchTraffic
                                  ? remote.lastAnyPacketRxMs
                                  : remote.lastPositionReportRxMs;
    const uint32_t ageMs = dmElapsedMs(nowMs, lastRxMs);
    const uint32_t timeoutMs = linkTimeoutMs();
    const uint32_t suspectMs =
        timeoutMs > DM_LINK_TIMEOUT_MARGIN_MS
            ? timeoutMs - DM_LINK_TIMEOUT_MARGIN_MS
            : timeoutMs;

    if (ageMs <= suspectMs)
    {
        if (remote.radioState != DmRadioState::Pairing)
        {
            remote.radioState = DmRadioState::Alive;
        }
        return;
    }

    if (ageMs <= timeoutMs)
    {
        remote.radioState = DmRadioState::Suspected;
    }
    else
    {
        remote.radioState = DmRadioState::Lost;

        if (!isRadioFault(remote.faultCause))
        {
            remote.faultCause = diagnoseRadioLoss(remoteIndex);
            remote.faultStartedMs = nowMs;
            remote.hasFaultAudioTime = false;

            if (remote.faultCause == DmFaultCause::RadioComSaturation)
            {
                remote.comSaturationSilentUntilMs =
                    nowMs + DM_COM_SATURATION_SILENT_MS;
            }

            LOG_WARN(
                "Distance Monitor link lost: node=!%08lx cause=%s age=%lus last_ratio=%.2f battery=%u",
                static_cast<unsigned long>(remote.nodeNum),
                dmFaultCauseName(remote.faultCause),
                static_cast<unsigned long>(ageMs / 1000U),
                static_cast<double>(remote.lastValidDistanceRatio),
                static_cast<unsigned>(remote.batteryPercent));
        }
    }
}

void DistanceMonitorModule::processPendingSos(uint32_t nowMs)
{
    if (!pendingSos_.active)
    {
        return;
    }

    if (dmElapsedMs(nowMs, pendingSos_.lastTxMs) <
        DM_RESEND_TIMEOUT_MS)
    {
        return;
    }

    if (sendSos(
            pendingSos_.targetNode,
            pendingSos_.sequence,
            pendingSos_.cause))
    {
        pendingSos_.lastTxMs = nowMs;

        LOG_WARN(
            "Distance Monitor SOS retry: seq=%lu cause=%s",
            static_cast<unsigned long>(pendingSos_.sequence),
            dmSosCauseName(pendingSos_.cause));
    }
}

void DistanceMonitorModule::processNotifications(uint32_t nowMs)
{
    uint32_t pendingSequence = 0U;
    bool hasPending = false;
    bool allAcknowledged = true;

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        DmNodeState &remote = nodeStates_[index];
        if (remote.isBase ||
            remote.pendingNotificationSequence == 0U)
        {
            continue;
        }

        hasPending = true;
        if (pendingSequence == 0U)
        {
            pendingSequence =
                remote.pendingNotificationSequence;
        }

        if (remote.pendingNotificationSequence !=
                pendingSequence ||
            !remote.notificationAcked)
        {
            allAcknowledged = false;
        }

        if (dmElapsedMs(
                nowMs,
                remote.pendingNotificationSinceMs) >
            DM_NOTIFICATION_ACK_WINDOW_MS)
        {
            allAcknowledged = false;
        }
    }

    if (!hasPending)
    {
        return;
    }

    if (allAcknowledged)
    {
        audio_.playConfirmationBop();

        LOG_INFO(
            "Distance Monitor notification confirmed: seq=%lu",
            static_cast<unsigned long>(pendingSequence));

        for (size_t index = 0U;
             index < runtimeConfig_.memberCount;
             ++index)
        {
            DmNodeState &remote = nodeStates_[index];
            if (remote.pendingNotificationSequence ==
                pendingSequence)
            {
                remote.pendingNotificationSequence = 0U;
                remote.notificationAcked = false;
            }
        }
        return;
    }

    bool expired = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        const DmNodeState &remote = nodeStates_[index];
        if (remote.pendingNotificationSequence ==
                pendingSequence &&
            dmElapsedMs(
                nowMs,
                remote.pendingNotificationSinceMs) >
                DM_NOTIFICATION_ACK_WINDOW_MS)
        {
            expired = true;
            break;
        }
    }

    if (expired)
    {
        LOG_WARN(
            "Distance Monitor notification not confirmed: seq=%lu",
            static_cast<unsigned long>(pendingSequence));

        for (size_t index = 0U;
             index < runtimeConfig_.memberCount;
             ++index)
        {
            DmNodeState &remote = nodeStates_[index];
            if (remote.pendingNotificationSequence ==
                pendingSequence)
            {
                remote.pendingNotificationSequence = 0U;
                remote.notificationAcked = false;
            }
        }
    }
}

void DistanceMonitorModule::evaluateRemoteDistance(
    size_t localIndex,
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &local = nodeStates_[localIndex];
    DmNodeState &remote = nodeStates_[remoteIndex];

    if (evaluateGpsDistance(local, remote, nowMs))
    {
        updateRssiCalibration(
            remoteIndex,
            remote,
            static_cast<float>(remote.lastValidDistanceMeters),
            nowMs);
        return;
    }

    evaluateRssiDistance(local, remoteIndex, remote, nowMs);
}

bool DistanceMonitorModule::evaluateGpsDistance(
    const DmNodeState &localState,
    DmNodeState &remoteState,
    uint32_t nowMs)
{
    if (localState.positionKind == DmPositionKind::NoFix ||
        remoteState.positionKind == DmPositionKind::NoFix)
    {
        return false;
    }

    const double localLatitude =
        static_cast<double>(localState.latitudeI) * 1.0e-7;
    const double localLongitude =
        static_cast<double>(localState.longitudeI) * 1.0e-7;
    const double remoteLatitude =
        static_cast<double>(remoteState.latitudeI) * 1.0e-7;
    const double remoteLongitude =
        static_cast<double>(remoteState.longitudeI) * 1.0e-7;

    const double distance = dmCalculateDistanceMeters(
        localLatitude,
        localLongitude,
        remoteLatitude,
        remoteLongitude);

    const float ratio =
        runtimeConfig_.maxDistanceMeters > 0.0F
            ? static_cast<float>(
                  distance /
                  runtimeConfig_.maxDistanceMeters)
            : 0.0F;

    remoteState.distanceSource = DmDistanceSource::Gps;
    remoteState.distanceMeters = distance;
    remoteState.distanceRatio = ratio;
    remoteState.hasLastValidDistance = true;
    remoteState.lastValidDistanceMeters = distance;
    remoteState.lastValidDistanceRatio = ratio;
    remoteState.lastValidDistanceMs = nowMs;
    remoteState.distanceEstimateFault = false;

    updateDistanceAlertState(remoteState, ratio);
    return true;
}

DmRssiTableEstimate DistanceMonitorModule::currentRssiEstimate(
    size_t remoteIndex,
    const DmNodeState &remoteState,
    const DmNodeState &localState,
    uint32_t nowMs) const
{
    const DmRssiFilter &trackerToBase = directInboundRssi_[remoteIndex];

    const bool btValid = remoteState.remoteBaseRssiValid;
    const uint32_t btAgeMs = remoteState.hasPositionReportRxTime
                                 ? dmElapsedMs(nowMs, remoteState.lastPositionReportRxMs)
                                 : UINT32_MAX;

    const bool tbValid = trackerToBase.isInitialized() &&
                         dmElapsedMs(nowMs, trackerToBase.lastUpdateMs()) <= linkTimeoutMs();
    const uint32_t tbAgeMs = tbValid
                                 ? dmElapsedMs(nowMs, trackerToBase.lastUpdateMs())
                                 : UINT32_MAX;

    float trend = 0.0F;
    if (btValid && tbValid)
    {
        const float btQ = 1.0F / std::max(
            remoteState.remoteBaseRssiStdDb * remoteState.remoteBaseRssiStdDb, 1.0F);
        const float tbQ = 1.0F / std::max(
            trackerToBase.stdDb() * trackerToBase.stdDb(), 1.0F);
        trend = (btQ * remoteState.remoteBaseRssiTrendDbPerSec +
                 tbQ * trackerToBase.trendDbPerSec()) /
                (btQ + tbQ);
    }
    else if (btValid)
        trend = remoteState.remoteBaseRssiTrendDbPerSec;
    else if (tbValid)
        trend = trackerToBase.trendDbPerSec();

    return rssiProfiles_[remoteIndex].estimate(
        btValid,
        remoteState.remoteBaseRssiMeanDbm,
        std::max(remoteState.remoteBaseRssiStdDb, DM_RSSI_MIN_STD_DB),
        btAgeMs,
        tbValid,
        trackerToBase.meanDbm(),
        std::max(trackerToBase.stdDb(), DM_RSSI_MIN_STD_DB),
        tbAgeMs,
        trend,
        localState.moving || remoteState.moving);
}

bool DistanceMonitorModule::evaluateRssiDistance(
    const DmNodeState &localState,
    size_t remoteIndex,
    DmNodeState &remoteState,
    uint32_t nowMs)
{
    const DmRssiFilter &trackerToBase = directInboundRssi_[remoteIndex];
    const bool btValid = remoteState.remoteBaseRssiValid;
    const bool tbValid = trackerToBase.isUsable(nowMs, linkTimeoutMs());

    if (!btValid && !tbValid)
    {
        remoteState.distanceSource = DmDistanceSource::None;
        remoteState.rssiEstimate = DmDistanceBandEstimate{};
        remoteState.distanceEstimateFault = false;
        updateDistanceAlertState(remoteState, 0.0F);
        return false;
    }

    if (!searchModeActive_ &&
        rssiProfiles_[remoteIndex].observeSignal(
            btValid,
            remoteState.remoteBaseRssiMeanDbm,
            tbValid,
            trackerToBase.lastSampleDbm()))
    {
        rssiCalibrationDirty_[remoteIndex] = true;
    }

    const DmRssiTableEstimate table =
        currentRssiEstimate(remoteIndex, remoteState, localState, nowMs);

    remoteState.distanceSource = DmDistanceSource::Rssi;
    remoteState.distanceMeters = 0.0;
    remoteState.distanceRatio = 0.0F;
    remoteState.distanceEstimateFault = false;
    remoteState.rssiEstimate = DmDistanceBandEstimate{};
    remoteState.rssiEstimate.band = table.band;
    remoteState.rssiEstimate.confidence = table.confidence;
    remoteState.rssiEstimate.fusedRssiDbm = table.fusedRssiDbm;
    remoteState.rssiEstimate.fusedTrendDbPerSec = table.fusedTrendDbPerSec;

    if (!table.calibrated || table.band == DmDistanceBand::Unknown)
    {
        updateDistanceAlertState(remoteState, 0.0F);
        return false;
    }

    if (table.positiveAlert)
    {
        const float ratio = table.band == DmDistanceBand::Beyond ? 1.0F : 0.80F;
        LOG_WARN(
            "Distance Monitor RSSI positive alert: node=!%08lx band=%s conf=%.2f p_alert=%.2f",
            static_cast<unsigned long>(remoteState.nodeNum),
            dmDistanceBandName(table.band),
            static_cast<double>(table.confidence),
            static_cast<double>(table.alertProbability));
        updateDistanceAlertState(remoteState, ratio);
        return true;
    }

    updateDistanceAlertState(remoteState, 0.0F);
    return true;
}

void DistanceMonitorModule::updateDistanceAlertState(
    DmNodeState &remoteState,
    float currentRatio)
{
    const float previousActive =
        remoteState.activeDistanceAlertRatio;

    remoteState.currentDistanceAlertRatio =
        currentRatio >= DM_DISTANCE_ALERT_START_RATIO
            ? currentRatio
            : 0.0F;

    if (remoteState.distanceSource ==
        DmDistanceSource::Gps)
    {
        if (currentRatio >= 1.0F)
        {
            remoteState.criticalDistanceLatch = true;
            remoteState.criticalDistanceLatchRatio =
                std::max(
                    remoteState.criticalDistanceLatchRatio,
                    currentRatio);
        }
        else if (currentRatio <
                 DM_DISTANCE_ALERT_START_RATIO)
        {

            remoteState.criticalDistanceLatch = false;
            remoteState.criticalDistanceLatchRatio = 0.0F;
        }
    }

    float active =
        remoteState.currentDistanceAlertRatio;

    if (remoteState.criticalDistanceLatch &&
        remoteState.distanceSource !=
            DmDistanceSource::Gps &&
        active < DM_DISTANCE_ALERT_START_RATIO)
    {
        active =
            std::max(
                1.0F,
                remoteState.criticalDistanceLatchRatio);
    }

    remoteState.activeDistanceAlertRatio = active;

    if (previousActive <
            DM_DISTANCE_ALERT_START_RATIO &&
        active >= DM_DISTANCE_ALERT_START_RATIO)
    {
        remoteState.hasDistanceBipTime = false;
    }
}

void DistanceMonitorModule::updateRssiCalibration(
    size_t remoteIndex,
    DmNodeState &remoteState,
    float distanceMeters,
    uint32_t nowMs)
{

    if (searchModeActive_ ||
        remoteIndex >= runtimeConfig_.memberCount ||
        remoteState.lastPositionReportSequence == 0U ||
        lastCalibrationSequence_[remoteIndex] == remoteState.lastPositionReportSequence)
    {
        return;
    }

    const DmRssiFilter &trackerToBase = directInboundRssi_[remoteIndex];
    const bool btValid = remoteState.remoteBaseRssiValid;
    const bool tbValid = trackerToBase.isUsable(nowMs, linkTimeoutMs());

    if (!btValid && !tbValid)
    {
        return;
    }

    if (rssiProfiles_[remoteIndex].update(
            distanceMeters,
            btValid,
            remoteState.remoteBaseRssiMeanDbm,
            tbValid,
            trackerToBase.lastSampleDbm()))
    {
        lastCalibrationSequence_[remoteIndex] = remoteState.lastPositionReportSequence;
        rssiCalibrationDirty_[remoteIndex] = true;
    }
}

DmFaultCause DistanceMonitorModule::diagnoseRadioLoss(
    size_t remoteIndex) const
{
    const DmNodeState &remote =
        nodeStates_[remoteIndex];

    if (remote.shutdownNoticeReceived)
    {
        return DmFaultCause::RemoteShutdown;
    }

    if (remote.batteryPercent != DM_BATTERY_UNKNOWN &&
        remote.batteryPercent <=
            DM_LOW_BATTERY_PERCENT)
    {
        return DmFaultCause::RadioLowBattery;
    }

    const DmRssiFilter &trackerToBase =
        directInboundRssi_[remoteIndex];

    bool rssiAvailable = trackerToBase.isInitialized();
    float fusedRssi = trackerToBase.meanDbm();
    float fusedTrend = trackerToBase.trendDbPerSec();

    if (remote.remoteBaseRssiValid && rssiAvailable)
    {
        fusedRssi = dmFuseRssi(
            remote.remoteBaseRssiMeanDbm,
            trackerToBase.meanDbm());
        fusedTrend = dmFuseRssiTrend(
            remote.remoteBaseRssiTrendDbPerSec,
            trackerToBase.trendDbPerSec());
    }

    if (remote.hasLastValidDistance &&
        remote.lastValidDistanceRatio >
            DM_RANGE_LOSS_MIN_RATIO &&
        rssiAvailable &&
        fusedTrend <=
            DM_RSSI_DEGRADING_TREND_DB_PER_S)
    {
        return DmFaultCause::RadioRangeLoss;
    }

    if (remote.hasLastValidDistance &&
        remote.lastValidDistanceRatio <
            DM_COM_SATURATION_MAX_RATIO &&
        rssiAvailable &&
        fusedRssi >= DM_RSSI_GOOD_DBM &&
        fusedTrend >=
            DM_RSSI_STABLE_TREND_DB_PER_S)
    {
        return DmFaultCause::RadioComSaturation;
    }

    return DmFaultCause::RadioUnexpectedLoss;
}

bool DistanceMonitorModule::isRadioFault(
    DmFaultCause cause) const
{
    switch (cause)
    {
    case DmFaultCause::RadioLowBattery:
    case DmFaultCause::RadioRangeLoss:
    case DmFaultCause::RadioComSaturation:
    case DmFaultCause::RadioUnexpectedLoss:
    case DmFaultCause::RemoteShutdown:
    case DmFaultCause::Unpaired:
        return true;
    case DmFaultCause::DistanceEstimateUnavailable:
    case DmFaultCause::None:
    default:
        return false;
    }
}

bool DistanceMonitorModule::isFaultAudible(
    const DmNodeState &state,
    uint32_t nowMs) const
{
    if (!isRadioFault(state.faultCause))
    {
        return false;
    }
    if (deadlinePending(nowMs, state.faultSnoozedUntilMs))
    {
        return false;
    }
    if (state.faultCause == DmFaultCause::RadioComSaturation &&
        deadlinePending(nowMs, state.comSaturationSilentUntilMs))
    {
        return false;
    }
    return true;
}

uint32_t DistanceMonitorModule::linkTimeoutMs() const
{
    return static_cast<uint32_t>(
               dmComputeMaxReportIntervalSec(
                   runtimeConfig_.maxDistanceMeters)) *
               1000U +
           DM_LINK_TIMEOUT_MARGIN_MS;
}

uint32_t DistanceMonitorModule::rssiBeaconMaxAgeMs() const
{
    return DM_BASE_BEACON_INTERVAL_MS *
           DM_RSSI_VALID_BEACON_MULTIPLIER;
}

void DistanceMonitorModule::updateBaseGpsDemand(size_t localIndex)
{
    uint8_t required =
        dmComputeMaxReportIntervalSec(
            runtimeConfig_.maxDistanceMeters);

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex ||
            nodeStates_[index].isBase)
        {
            continue;
        }

        const DmNodeState &remote =
            nodeStates_[index];
        uint8_t candidate =
            remote.desiredIntervalSec > 0U
                ? remote.desiredIntervalSec
                : remote.appliedIntervalSec;

        if (candidate >= DM_MIN_REPORT_INTERVAL_S)
        {
            required =
                std::min<uint8_t>(
                    required,
                    candidate);
        }
    }

    if (required != localDesiredGpsIntervalSec_)
    {
        localDesiredGpsIntervalSec_ = required;

        if (!gpsSleeping_ &&
            highSpeedWatchUntilMs_ == 0U)
        {
            applyGpsInterval(required);
        }
    }
}

void DistanceMonitorModule::updateBaseAudio(
    size_t localIndex,
    uint32_t nowMs)
{
    bool hasSos = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index != localIndex &&
            nodeStates_[index].sosActive)
        {
            hasSos = true;
            break;
        }
    }

    if (hasSos)
    {
        audio_.startSos();
        return;
    }

    audio_.stopSos();

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        DmNodeState &remote = nodeStates_[index];
        if (!isFaultAudible(remote, nowMs))
        {
            continue;
        }

        if (!remote.hasFaultAudioTime ||
            dmElapsedMs(
                nowMs,
                remote.lastFaultAudioMs) >=
                DM_FAULT_AUDIO_PERIOD_MS)
        {
            remote.lastFaultAudioMs = nowMs;
            remote.hasFaultAudioTime = true;
            audio_.playFaultBops();
            return;
        }
    }

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        DmNodeState &remote = nodeStates_[index];
        if (remote.activeDistanceAlertRatio <
            DM_DISTANCE_ALERT_START_RATIO)
        {
            continue;
        }

        if (deadlinePending(
                nowMs,
                remote.distanceSnoozedUntilMs))
        {
            continue;
        }

        const uint32_t periodMs =
            dmDistanceBipIntervalMs(
                remote.activeDistanceAlertRatio);
        if (periodMs == 0U)
        {
            continue;
        }

        if (!remote.hasDistanceBipTime ||
            dmElapsedMs(
                nowMs,
                remote.lastDistanceBipMs) >=
                periodMs)
        {
            remote.lastDistanceBipMs = nowMs;
            remote.hasDistanceBipTime = true;
            audio_.playDistanceBip();
            return;
        }
    }
}

void DistanceMonitorModule::logRssiCalibrationTable(size_t memberIndex) const
{
    if (memberIndex >= runtimeConfig_.memberCount || nodeStates_[memberIndex].isBase)
        return;

    const DmRssiCalibrationProfile &profile = rssiProfiles_[memberIndex];
    LOG_INFO("RSSI calibration: node=!%08lx samples=%lu bestBT=%s%.1f bestTB=%s%.1f",
             static_cast<unsigned long>(nodeStates_[memberIndex].nodeNum),
             static_cast<unsigned long>(profile.totalSamples()),
             profile.bestBaseToTrackerValid() ? "" : "NA/",
             static_cast<double>(profile.bestBaseToTrackerDbm()),
             profile.bestTrackerToBaseValid() ? "" : "NA/",
             static_cast<double>(profile.bestTrackerToBaseDbm()));
    LOG_INFO("RSSI calibration table: range(m) | BT mean std n | TB mean std n");

    for (size_t i = 0U; i < profile.binCount(); ++i)
    {
        const DmRssiCalibrationBin &b = profile.bin(i);
        if (b.baseToTracker.count() > 0U && b.trackerToBase.count() > 0U)
        {
            LOG_INFO("RSSI cal %-7s | BT %6.1f %4.1f %lu | TB %6.1f %4.1f %lu",
                     dmRssiBinLabel(i),
                     static_cast<double>(b.baseToTracker.mean()),
                     static_cast<double>(b.baseToTracker.stdDev()),
                     static_cast<unsigned long>(b.baseToTracker.count()),
                     static_cast<double>(b.trackerToBase.mean()),
                     static_cast<double>(b.trackerToBase.stdDev()),
                     static_cast<unsigned long>(b.trackerToBase.count()));
        }
        else if (b.baseToTracker.count() > 0U)
        {
            LOG_INFO("RSSI cal %-7s | BT %6.1f %4.1f %lu | TB -- -- 0",
                     dmRssiBinLabel(i),
                     static_cast<double>(b.baseToTracker.mean()),
                     static_cast<double>(b.baseToTracker.stdDev()),
                     static_cast<unsigned long>(b.baseToTracker.count()));
        }
        else if (b.trackerToBase.count() > 0U)
        {
            LOG_INFO("RSSI cal %-7s | BT -- -- 0 | TB %6.1f %4.1f %lu",
                     dmRssiBinLabel(i),
                     static_cast<double>(b.trackerToBase.mean()),
                     static_cast<double>(b.trackerToBase.stdDev()),
                     static_cast<unsigned long>(b.trackerToBase.count()));
        }
        else
        {
            LOG_INFO("RSSI cal %-7s | BT -- -- 0 | TB -- -- 0", dmRssiBinLabel(i));
        }
    }
}

bool DistanceMonitorModule::loadRssiCalibrationProfile(size_t memberIndex)
{
#ifdef FSCom
    if (memberIndex >= runtimeConfig_.memberCount || nodeStates_[memberIndex].isBase)
        return false;

    const uint32_t nodeNum = nodeStates_[memberIndex].nodeNum;
    DmRssiPersistRecord best = {};
    bool haveBest = false;

    for (const char slot : {'a', 'b'})
    {
        char path[64] = {};
        dmRssiFilePath(path, sizeof(path), nodeNum, slot);
        DmRssiPersistRecord candidate = {};
        bool readOk = false;
        {
            concurrency::LockGuard g(spiLock);
            File f = FSCom.open(path, FILE_O_READ);
            if (f)
            {
                readOk = f.read(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate)) == sizeof(candidate);
                f.close();
            }
        }
        if (!readOk)
            continue;

        const uint32_t storedCrc = candidate.crc32;
        candidate.crc32 = 0U;
        const uint32_t computedCrc = dmCrc32(
            reinterpret_cast<const uint8_t *>(&candidate), sizeof(candidate));
        candidate.crc32 = storedCrc;

        const bool valid = candidate.magic == DM_RSSI_FILE_MAGIC &&
                           candidate.version == DM_RSSI_FILE_VERSION &&
                           candidate.binCount == DM_RSSI_CALIBRATION_BIN_COUNT &&
                           candidate.nodeNum == nodeNum &&
                           storedCrc == computedCrc;
        if (valid && (!haveBest || candidate.generation > best.generation))
        {
            best = candidate;
            haveBest = true;
        }
    }

    if (!haveBest)
    {
        rssiProfiles_[memberIndex].reset();
        rssiCalibrationGeneration_[memberIndex] = 0U;
        rssiCalibrationSavedSamples_[memberIndex] = 0U;
        LOG_INFO("RSSI calibration: node=!%08lx no valid flash table; starting empty",
                 static_cast<unsigned long>(nodeNum));
        return false;
    }

    DmRssiCalibrationProfile &profile = rssiProfiles_[memberIndex];
    profile.reset();
    profile.restoreBest(
        (best.flags & 0x01U) != 0U, best.bestBtDbm,
        (best.flags & 0x02U) != 0U, best.bestTbDbm);

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        profile.bin(i).baseToTracker.restore(
            best.bins[i].bt.count, best.bins[i].bt.mean, best.bins[i].bt.m2);
        profile.bin(i).trackerToBase.restore(
            best.bins[i].tb.count, best.bins[i].tb.mean, best.bins[i].tb.m2);
    }

    rssiCalibrationGeneration_[memberIndex] = best.generation;
    rssiCalibrationSavedSamples_[memberIndex] = profile.totalSamples();
    LOG_INFO("RSSI calibration loaded: node=!%08lx generation=%lu samples=%lu",
             static_cast<unsigned long>(nodeNum),
             static_cast<unsigned long>(best.generation),
             static_cast<unsigned long>(profile.totalSamples()));
    return true;
#else
    (void)memberIndex;
    return false;
#endif
}

bool DistanceMonitorModule::saveRssiCalibrationProfile(size_t memberIndex)
{
#ifdef FSCom
    if (memberIndex >= runtimeConfig_.memberCount || nodeStates_[memberIndex].isBase)
        return false;

    DmRssiPersistRecord record = {};
    record.magic = DM_RSSI_FILE_MAGIC;
    record.version = DM_RSSI_FILE_VERSION;
    record.binCount = DM_RSSI_CALIBRATION_BIN_COUNT;
    record.nodeNum = nodeStates_[memberIndex].nodeNum;
    record.generation = rssiCalibrationGeneration_[memberIndex] + 1U;

    const DmRssiCalibrationProfile &profile = rssiProfiles_[memberIndex];
    if (profile.bestBaseToTrackerValid())
    {
        record.flags |= 0x01U;
        record.bestBtDbm = profile.bestBaseToTrackerDbm();
    }
    if (profile.bestTrackerToBaseValid())
    {
        record.flags |= 0x02U;
        record.bestTbDbm = profile.bestTrackerToBaseDbm();
    }

    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        const DmRssiCalibrationBin &b = profile.bin(i);
        record.bins[i].bt = {b.baseToTracker.count(), b.baseToTracker.mean(), b.baseToTracker.m2()};
        record.bins[i].tb = {b.trackerToBase.count(), b.trackerToBase.mean(), b.trackerToBase.m2()};
    }

    record.crc32 = 0U;
    record.crc32 = dmCrc32(reinterpret_cast<const uint8_t *>(&record), sizeof(record));

    const char slot = (record.generation & 1U) != 0U ? 'a' : 'b';
    char path[64] = {};
    dmRssiFilePath(path, sizeof(path), record.nodeNum, slot);

    bool ok = false;
    {
        concurrency::LockGuard g(spiLock);
        File f = FSCom.open(path, FILE_O_WRITE);
        if (f)
        {
            ok = f.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record)) == sizeof(record);
            f.flush();
            f.close();
        }
    }

    if (!ok)
    {
        LOG_ERROR("RSSI calibration save failed: node=!%08lx slot=%c",
                  static_cast<unsigned long>(record.nodeNum), slot);
        return false;
    }

    rssiCalibrationGeneration_[memberIndex] = record.generation;
    rssiCalibrationSavedSamples_[memberIndex] = profile.totalSamples();
    rssiCalibrationDirty_[memberIndex] = false;
    LOG_INFO("RSSI calibration saved: node=!%08lx generation=%lu samples=%lu slot=%c",
             static_cast<unsigned long>(record.nodeNum),
             static_cast<unsigned long>(record.generation),
             static_cast<unsigned long>(profile.totalSamples()), slot);
    return true;
#else
    (void)memberIndex;
    return false;
#endif
}

void DistanceMonitorModule::loadRssiCalibrationIfNeeded(size_t localIndex)
{
    if (rssiCalibrationLoaded_)
        return;
    rssiCalibrationLoaded_ = true;

    if (!nodeStates_[localIndex].isBase)
        return;

    for (size_t i = 0U; i < runtimeConfig_.memberCount; ++i)
    {
        if (i == localIndex || nodeStates_[i].isBase)
            continue;
        loadRssiCalibrationProfile(i);
        logRssiCalibrationTable(i);
    }
}

void DistanceMonitorModule::saveAllRssiCalibration(bool force)
{
    for (size_t i = 0U; i < runtimeConfig_.memberCount; ++i)
    {
        if (nodeStates_[i].isBase)
            continue;
        if (force || rssiCalibrationDirty_[i])
            saveRssiCalibrationProfile(i);
    }
}

bool DistanceMonitorModule::isLocalBase() const
{
    size_t index = 0U;
    return findLocalIndex(index) && nodeStates_[index].isBase;
}

bool DistanceMonitorModule::handleSearchToggle()
{
    initializeIfNeeded();
    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex) || !nodeStates_[localIndex].isBase)
        return false;

    if (searchModeActive_)
    {
        if (searchTargetIndex_ < runtimeConfig_.memberCount)
        {
            sendSearchStop(nodeStates_[searchTargetIndex_].nodeNum);
            nodeStates_[searchTargetIndex_].lastIntervalEvaluationSequence = 0U;
        }

        searchModeActive_ = false;
        audio_.playSearchExit();
        LOG_INFO("Distance Monitor SEARCH: OFF");
        searchTargetIndex_ = DM_MAX_MEMBERS;
        hasSearchPulseTime_ = false;
        hasSearchStartTxTime_ = false;
        searchRssiValid_ = false;
        searchLastSampleMs_ = 0U;
        searchFilteredDbm_ = 0.0F;
        searchPreviousFilteredDbm_ = 0.0F;
        searchTrendDbPerSec_ = 0.0F;
        return true;
    }

    size_t target = DM_MAX_MEMBERS;
    for (size_t i = 0U; i < runtimeConfig_.memberCount; ++i)
    {
        if (i != localIndex && !nodeStates_[i].isBase && nodeStates_[i].paired)
        {
            target = i;
            break;
        }
    }

    if (target == DM_MAX_MEMBERS)
    {
        LOG_WARN("Distance Monitor SEARCH: no paired tracker");
        return true;
    }

    searchModeActive_ = true;
    searchTargetIndex_ = target;
    hasSearchPulseTime_ = false;
    hasSearchStartTxTime_ = false;
    searchRssiValid_ = false;
    searchLastSampleMs_ = 0U;
    searchFilteredDbm_ = 0.0F;
    searchPreviousFilteredDbm_ = 0.0F;
    searchTrendDbPerSec_ = 0.0F;

    DmNodeState &remote = nodeStates_[target];
    if (sendSearchStart(remote.nodeNum))
    {
        lastSearchStartTxMs_ = millis();
        hasSearchStartTxTime_ = true;
    }

    audio_.playSearchEnter();
    LOG_INFO(
        "Distance Monitor SEARCH: ON target=!%08lx beacon=%lums span=%.1f..%.1fdBm",
        static_cast<unsigned long>(remote.nodeNum),
        static_cast<unsigned long>(DM_SEARCH_BEACON_INTERVAL_MS),
        static_cast<double>(DM_SEARCH_RSSI_WORST_DBM),
        static_cast<double>(DM_SEARCH_RSSI_BEST_DBM));
    return true;
}

void DistanceMonitorModule::processSearchMode(size_t, uint32_t nowMs)
{
    if (!searchModeActive_ || searchTargetIndex_ >= runtimeConfig_.memberCount)
        return;

    DmNodeState &remote = nodeStates_[searchTargetIndex_];
    if (!remote.paired || remote.sosActive || audio_.isSosActive())
        return;

    if (!hasSearchStartTxTime_ ||
        dmElapsedMs(nowMs, lastSearchStartTxMs_) >= DM_SEARCH_START_REFRESH_MS)
    {
        if (sendSearchStart(remote.nodeNum))
        {
            lastSearchStartTxMs_ = nowMs;
            hasSearchStartTxTime_ = true;
        }
    }

    if (searchRssiValid_ &&
        dmElapsedMs(nowMs, searchLastSampleMs_) > DM_SEARCH_BEACON_TIMEOUT_MS)
    {
        searchRssiValid_ = false;
        searchTrendDbPerSec_ = 0.0F;
    }

    float proximity = 0.0F;
    if (searchRssiValid_)
    {
        const uint32_t ageMs = dmElapsedMs(nowMs, searchLastSampleMs_);
        const uint32_t interpMs = std::min(ageMs, DM_SEARCH_TREND_INTERPOLATION_MS);
        float predictedDelta = searchTrendDbPerSec_ *
                               static_cast<float>(interpMs) / 1000.0F;
        predictedDelta = std::max(
            -DM_SEARCH_TREND_INTERPOLATION_CLAMP_DB,
            std::min(DM_SEARCH_TREND_INTERPOLATION_CLAMP_DB, predictedDelta));
        const float predictedRssi = searchFilteredDbm_ + predictedDelta;
        const float denominator = std::max(
            1.0F, DM_SEARCH_RSSI_BEST_DBM - DM_SEARCH_RSSI_WORST_DBM);
        proximity = (predictedRssi - DM_SEARCH_RSSI_WORST_DBM) / denominator;
        proximity = std::max(0.0F, std::min(1.0F, proximity));
    }

    if ((!hasSearchPulseTime_ ||
         dmElapsedMs(nowMs, lastSearchPulseMs_) >= DM_SEARCH_PULSE_INTERVAL_MS) &&
        !audio_.isBusyAboveSearch())
    {
        audio_.playSearchPulse(proximity);
        lastSearchPulseMs_ = nowMs;
        hasSearchPulseTime_ = true;
    }
}

void DistanceMonitorModule::processTrackerSearchMode(size_t, uint32_t nowMs)
{
    if (!trackerSearchActive_)
        return;

    if (static_cast<int32_t>(nowMs - trackerSearchLeaseUntilMs_) >= 0)
    {
        trackerSearchActive_ = false;
        trackerSearchBaseNode_ = 0U;
        hasTrackerSearchBeaconTxTime_ = false;
        forcePositionReport_ = true;
        return;
    }

    if (pendingSos_.active)
        return;

    if (!hasTrackerSearchBeaconTxTime_ ||
        dmElapsedMs(nowMs, lastTrackerSearchBeaconTxMs_) >= DM_SEARCH_BEACON_INTERVAL_MS)
    {
        if (sendSearchBeacon(trackerSearchBaseNode_))
        {
            lastTrackerSearchBeaconTxMs_ = nowMs;
            hasTrackerSearchBeaconTxTime_ = true;
        }
    }
}

void DistanceMonitorModule::updateSearchRssi(float rawRssiDbm, uint32_t nowMs)
{
    if (!searchRssiValid_)
    {
        searchFilteredDbm_ = rawRssiDbm;
        searchPreviousFilteredDbm_ = rawRssiDbm;
        searchTrendDbPerSec_ = 0.0F;
        searchRssiValid_ = true;
        searchLastSampleMs_ = nowMs;
        return;
    }

    const float previous = searchFilteredDbm_;
    const uint32_t dtMs = dmElapsedMs(nowMs, searchLastSampleMs_);
    searchPreviousFilteredDbm_ = previous;
    searchFilteredDbm_ =
        DM_SEARCH_FILTER_ALPHA * rawRssiDbm +
        (1.0F - DM_SEARCH_FILTER_ALPHA) * previous;

    if (dtMs > 0U)
    {
        searchTrendDbPerSec_ =
            (searchFilteredDbm_ - previous) * 1000.0F /
            static_cast<float>(dtMs);
        searchTrendDbPerSec_ = std::max(
            -DM_SEARCH_TREND_CLAMP_DB_PER_S,
            std::min(DM_SEARCH_TREND_CLAMP_DB_PER_S,
                     searchTrendDbPerSec_));
    }

    searchLastSampleMs_ = nowMs;
}

bool DistanceMonitorModule::handleSingleButtonPress()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
    {
        return false;
    }

    if (!nodeStates_[localIndex].isBase)
    {
        return true;
    }

    const uint32_t nowMs = millis();

    bool hadSos = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        DmNodeState &remote = nodeStates_[index];
        if (remote.sosActive)
        {
            remote.sosActive = false;
            hadSos = true;
        }
    }

    if (hadSos)
    {
        audio_.stopSos();
        LOG_INFO("Distance Monitor SOS acknowledged by adult");
        return true;
    }

    bool hadFault = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        DmNodeState &remote = nodeStates_[index];
        if (!isFaultAudible(remote, nowMs))
        {
            continue;
        }

        const uint32_t snooze =
            remote.faultCause ==
                    DmFaultCause::RadioComSaturation
                ? DM_COM_SATURATION_SNOOZE_MS
                : DM_FAULT_SNOOZE_MS;

        remote.faultSnoozedUntilMs =
            nowMs + snooze;
        hadFault = true;
    }

    if (hadFault)
    {
        LOG_INFO("Distance Monitor fault snoozed by adult");
        return true;
    }

    bool hadDistanceAlert = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        DmNodeState &remote = nodeStates_[index];
        if (remote.activeDistanceAlertRatio <
            DM_DISTANCE_ALERT_START_RATIO)
        {
            continue;
        }

        hadDistanceAlert = true;

        if (remote.criticalDistanceLatch &&
            remote.currentDistanceAlertRatio <
                DM_DISTANCE_ALERT_START_RATIO)
        {

            remote.criticalDistanceLatch = false;
            remote.criticalDistanceLatchRatio = 0.0F;
            remote.activeDistanceAlertRatio = 0.0F;
        }
        else
        {
            remote.distanceSnoozedUntilMs =
                nowMs + DM_DISTANCE_SNOOZE_MS;
        }
    }

    if (hadDistanceAlert)
    {
        LOG_INFO(
            "Distance Monitor distance alert acknowledged by adult");
    }

    return true;
}

bool DistanceMonitorModule::handleDoubleButtonPress()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
    {
        return false;
    }

    if (!nodeStates_[localIndex].isBase)
    {
        triggerLocalSos(DmSosCause::ManualButton);
        return true;
    }

    const uint32_t sequence =
        allocateSequenceNumber();
    const uint32_t nowMs = millis();
    bool attempted = false;

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        DmNodeState &remote = nodeStates_[index];
        if (index == localIndex || remote.isBase)
        {
            continue;
        }

        attempted = true;
        remote.pendingNotificationSequence =
            sequence;
        remote.pendingNotificationSinceMs = nowMs;
        remote.notificationAcked = false;

        sendNotification(
            remote.nodeNum,
            sequence);
    }

    if (attempted)
    {
        LOG_INFO(
            "Distance Monitor notification TX: seq=%lu",
            static_cast<unsigned long>(sequence));
    }

    return true;
}

uint32_t DistanceMonitorModule::prepareLocalShutdown()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
    {
        return 0U;
    }

    if (nodeStates_[localIndex].isBase)
    {
        if (searchModeActive_ && searchTargetIndex_ < runtimeConfig_.memberCount)
        {
            sendSearchStop(nodeStates_[searchTargetIndex_].nodeNum);
            searchModeActive_ = false;
        }
        saveAllRssiCalibration(true);
        return DM_SHUTDOWN_TX_GRACE_MS;
    }

    size_t baseIndex = 0U;
    if (!findBaseIndex(baseIndex))
    {
        return 0U;
    }

    sendShutdownNotice(runtimeConfig_.members[baseIndex].nodeNum);
    LOG_WARN("Distance Monitor local shutdown notice sent");
    return DM_SHUTDOWN_TX_GRACE_MS;
}

void DistanceMonitorModule::logSummary(
    size_t localIndex,
    uint32_t nowMs) const
{
    const DmNodeState &local = nodeStates_[localIndex];

    const char *localStateName = "STATIONARY";
    if (highSpeedSosLatched_)
        localStateName = "IN_VEHICLE";
    else if (fallFreefallArmed_)
        localStateName = "FALLING";
    else if (localMoving_)
        localStateName = "MOVING";

    LOG_INFO(
        "Self: node=!%08lx role=%s pos=%s battery=%u%% state=%s",
        static_cast<unsigned long>(local.nodeNum),
        local.isBase ? "base" : "tracker",
        dmPositionKindName(local.positionKind),
        static_cast<unsigned>(local.batteryPercent),
        localStateName);

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
            continue;

        const DmNodeState &remote = nodeStates_[index];
        const uint32_t reportAgeSeconds = remote.hasPositionReportRxTime
            ? dmElapsedMs(nowMs, remote.lastPositionReportRxMs) / 1000U
            : 0U;

        if (remote.distanceSource == DmDistanceSource::Gps)
        {
            float ratio = remote.distanceRatio;
            ratio = std::max(0.0F, std::min(1.0F, ratio));
            const unsigned pct = static_cast<unsigned>(ratio * 100.0F + 0.5F);
            LOG_INFO(
                "Member: node=!%08lx paired=%s radio=%s report_age=%lus pos=%s dist=%.1fm (%u%%)",
                static_cast<unsigned long>(remote.nodeNum),
                remote.paired ? "yes" : "no",
                dmRadioStateName(remote.radioState),
                static_cast<unsigned long>(reportAgeSeconds),
                dmPositionKindName(remote.positionKind),
                remote.distanceMeters,
                pct);
        }
        else if (remote.distanceSource == DmDistanceSource::Rssi)
        {
            LOG_INFO(
                "Member: node=!%08lx paired=%s radio=%s report_age=%lus pos=%s band=%s conf=%.2f",
                static_cast<unsigned long>(remote.nodeNum),
                remote.paired ? "yes" : "no",
                dmRadioStateName(remote.radioState),
                static_cast<unsigned long>(reportAgeSeconds),
                dmPositionKindName(remote.positionKind),
                dmDistanceBandName(remote.rssiEstimate.band),
                static_cast<double>(remote.rssiEstimate.confidence));
        }
        else
        {
            LOG_INFO(
                "Member: node=!%08lx paired=%s radio=%s report_age=%lus pos=%s distance=UNAVAILABLE",
                static_cast<unsigned long>(remote.nodeNum),
                remote.paired ? "yes" : "no",
                dmRadioStateName(remote.radioState),
                static_cast<unsigned long>(reportAgeSeconds),
                dmPositionKindName(remote.positionKind));
        }
    }
}
