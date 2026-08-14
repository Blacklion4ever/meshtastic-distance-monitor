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
    return deadlineMs != 0U && static_cast<int32_t>(nowMs - deadlineMs) < 0;
}

static constexpr uint32_t DM_RSSI_FILE_MAGIC = 0x52525344U;
static constexpr uint16_t DM_RSSI_FILE_VERSION = 2U;

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
    std::snprintf(
        buffer,
        size,
        "/prefs/dm_rssi_%08lx_%c.bin",
        static_cast<unsigned long>(nodeNum),
        slot);
}


const char *localMotionName(
    bool moving,
    bool fallArmed,
    bool highSpeedLatched)
{
    if (highSpeedLatched)
        return "IN_VEHICLE";
    if (fallArmed)
        return "FALL";
    return moving ? "MOVING" : "STATIONARY";
}

const char *remoteMotionName(const DmNodeState &state)
{
    if (state.sosActive)
    {
        if (state.activeSosCause == DmSosCause::FallDetected)
            return "FALL";
        if (state.activeSosCause == DmSosCause::HighSpeedMovement)
            return "IN_VEHICLE";
    }
    return state.moving ? "MOVING" : "STATIONARY";
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
        return;

    bootMs_ = millis();
    const uint32_t localNodeNum = nodeDB != nullptr ? nodeDB->getNodeNum() : 0U;
    bootSessionId_ = static_cast<uint32_t>(micros()) ^
                     (bootMs_ << 11U) ^ localNodeNum ^ 0xD157A9C5U;
    if (bootSessionId_ == 0U)
        bootSessionId_ = 1U;

    localAppliedIntervalSec_ =
        dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);
    applyRuntimeProfile();
    audio_.setup();
    moduleInitialized_ = true;

    LOG_INFO(
        "{Init} firmware=%s protocol=%u",
        DM_FIRMWARE_VERSION,
        static_cast<unsigned>(DM_PROTOCOL_VERSION));
    if (!audio_.isAvailable())
        LOG_ERROR("{Alarm} Cause=BuzzerUnavailable");

    setIntervalFromNow(DM_TICK_INTERVAL_MS);
}

int32_t DistanceMonitorModule::runOnce()
{
    initializeIfNeeded();

    if (nodeDB == nullptr || runtimeConfig_.memberCount == 0U)
        return DM_TICK_INTERVAL_MS;

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return DM_TICK_INTERVAL_MS;

    const uint32_t nowMs = millis();
    DmNodeState &localState = nodeStates_[localIndex];

    if (!localPositionStarted_)
        startLocalPositionManager(nowMs);

    sampleLocalImu(localState, nowMs);
    if (localState.isBase)
        processSearchMode(localIndex, nowMs);

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
        processBase(localIndex, nowMs);
    else
        processTracker(localIndex, nowMs);

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

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        runtimeConfig_.members[index].nodeNum = DM_DEFAULT_MEMBERS[index].nodeNum;
        runtimeConfig_.members[index].isBase = DM_DEFAULT_MEMBERS[index].isBase;
        nodeStates_[index] = DmNodeState{};
        nodeStates_[index].nodeNum = DM_DEFAULT_MEMBERS[index].nodeNum;
        nodeStates_[index].isBase = DM_DEFAULT_MEMBERS[index].isBase;
        nodeStates_[index].batteryPercent = DM_BATTERY_UNKNOWN;
    }
}

void DistanceMonitorModule::applyRuntimeProfile()
{
    config.device.role = meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
    config.device.buzzer_mode = meshtastic_Config_DeviceConfig_BuzzerMode_DISABLED;
    config.device.disable_triple_click = true;
    config.device.led_heartbeat_disabled = false;
    config.position.position_broadcast_smart_enabled = false;
    config.position.position_broadcast_secs = DM_NATIVE_POSITION_BROADCAST_INTERVAL_S;
    moduleConfig.telemetry.device_telemetry_enabled = false;
    moduleConfig.mqtt.enabled = false;
    moduleConfig.external_notification.enabled = false;
}

bool DistanceMonitorModule::findMemberIndex(uint32_t nodeNum, size_t &index) const
{
    for (size_t candidate = 0U; candidate < runtimeConfig_.memberCount; ++candidate)
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
        return false;
    return findMemberIndex(nodeDB->getNodeNum(), index);
}

bool DistanceMonitorModule::findBaseIndex(size_t &index) const
{
    for (size_t candidate = 0U; candidate < runtimeConfig_.memberCount; ++candidate)
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
        return DM_BATTERY_UNKNOWN;

    return std::min<uint8_t>(100U, powerStatus->getBatteryChargePercent());
}

// The base owns pairing and periodically reasserts it so both devices recover
// from a missed confirmation or a tracker-side RAM state loss.
void DistanceMonitorModule::processPairing(size_t localIndex, uint32_t nowMs)
{
    DmNodeState &localState = nodeStates_[localIndex];
    if (!localState.isBase)
        return;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex || nodeStates_[index].isBase)
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (remote.shutdownNoticeReceived)
            continue;

        if (remote.paired)
        {
            if (remote.hasRemoteSession &&
                (!remote.hasHandshakeTxTime ||
                 dmElapsedMs(nowMs, remote.lastHandshakeTxMs) >=
                     DM_PAIR_REASSERT_INTERVAL_MS))
            {
                if (sendPairConfirm(remote.nodeNum, remote.remoteSessionId, false))
                {
                    remote.pairedLocalSessionId = bootSessionId_;
                    remote.pairedRemoteSessionId = remote.remoteSessionId;
                    remote.lastHandshakeTxMs = nowMs;
                    remote.hasHandshakeTxTime = true;
                }
            }
            continue;
        }

        if (!remote.hasRemoteSession || !remote.hasRemoteUptime)
        {
            if (!remote.hasHandshakeTxTime ||
                dmElapsedMs(nowMs, remote.lastHandshakeTxMs) >= DM_HANDSHAKE_RETRY_MS)
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

        if (uptimeSeconds(nowMs) * 1000U < DM_PAIR_CONFIRM_DELAY_MS)
            continue;

        if (remote.hasHandshakeTxTime &&
            dmElapsedMs(nowMs, remote.lastHandshakeTxMs) < DM_HANDSHAKE_RETRY_MS)
        {
            continue;
        }

        const bool trackerJustBooted =
            remote.remoteUptimeSeconds <= DM_PAIR_REMOTE_BOOT_WINDOW_S;
        if (sendPairConfirm(remote.nodeNum, remote.remoteSessionId, trackerJustBooted))
        {
            remote.pairedLocalSessionId = bootSessionId_;
            remote.pairedRemoteSessionId = remote.remoteSessionId;
            remote.lastHandshakeTxMs = nowMs;
            remote.hasHandshakeTxTime = true;
            remote.radioState = DmRadioState::Pairing;
        }
    }
}

void DistanceMonitorModule::processBase(size_t localIndex, uint32_t nowMs)
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

    if (!hasActiveSos &&
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

    processNotifications(nowMs);
    updateBaseAudio(localIndex, nowMs);
}

void DistanceMonitorModule::processTracker(size_t, uint32_t nowMs)
{
    size_t baseIndex = 0U;
    if (!findBaseIndex(baseIndex))
        return;

    const uint32_t reportPeriodMs =
        static_cast<uint32_t>(
            std::max<uint8_t>(localAppliedIntervalSec_, DM_MIN_REPORT_INTERVAL_S)) *
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

    processPendingSos(nowMs);
}

void DistanceMonitorModule::processRemoteMember(
    size_t localIndex,
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &remote = nodeStates_[remoteIndex];
    processLinkState(remoteIndex, nowMs);

    if (remote.radioState == DmRadioState::Lost || !remote.hasPositionReportRxTime)
        return;

    evaluateRemoteDistance(localIndex, remoteIndex, nowMs);

    if (remote.lastPositionReportSequence == 0U ||
        remote.lastIntervalEvaluationSequence == remote.lastPositionReportSequence)
    {
        return;
    }

    uint8_t desired = remote.appliedIntervalSec;
    if (desired < DM_MIN_REPORT_INTERVAL_S)
        desired = dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);

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
            desired = dmComputeReportIntervalSec(0.80F, runtimeConfig_.maxDistanceMeters);
            break;
        case DmDistanceBand::Near:
        case DmDistanceBand::Mid:
        case DmDistanceBand::Unknown:
        default:
            break;
        }
    }

    remote.lastIntervalEvaluationSequence = remote.lastPositionReportSequence;

    if (remote.appliedIntervalSec != desired && remote.hasRemoteSession)
        sendSetPositionInterval(remote.nodeNum, remote.remoteSessionId, desired);
}

// Link alarms are valid only after pairing. Before pairing, missing reports are
// part of the handshake state and must not be presented as a radio failure.
void DistanceMonitorModule::processLinkState(size_t remoteIndex, uint32_t nowMs)
{
    DmNodeState &remote = nodeStates_[remoteIndex];
    if (!remote.hasPositionReportRxTime)
        return;

    const uint32_t ageMs = dmElapsedMs(nowMs, remote.lastPositionReportRxMs);
    const uint32_t timeoutMs = linkTimeoutMs();
    const uint32_t suspectMs =
        timeoutMs > DM_LINK_TIMEOUT_MARGIN_MS
            ? timeoutMs - DM_LINK_TIMEOUT_MARGIN_MS
            : timeoutMs;

    if (ageMs <= suspectMs)
    {
        if (remote.radioState != DmRadioState::Pairing)
            remote.radioState = DmRadioState::Alive;
        return;
    }

    if (ageMs <= timeoutMs)
    {
        remote.radioState = DmRadioState::Suspected;
        return;
    }

    remote.radioState = DmRadioState::Lost;
    if (!remote.paired)
        return;

    if (!isRadioFault(remote.faultCause))
    {
        remote.faultCause = diagnoseRadioLoss(remoteIndex);
        remote.hasFaultAudioTime = false;

        if (remote.faultCause == DmFaultCause::RadioComSaturation)
            remote.comSaturationSilentUntilMs = nowMs + DM_COM_SATURATION_SILENT_MS;

        LOG_WARN(
            "{Alarm} Cause=%s id=!%08lx age=%lus",
            dmFaultCauseName(remote.faultCause),
            static_cast<unsigned long>(remote.nodeNum),
            static_cast<unsigned long>(ageMs / 1000U));
    }
}

void DistanceMonitorModule::processPendingSos(uint32_t nowMs)
{
    if (!pendingSos_.active ||
        dmElapsedMs(nowMs, pendingSos_.lastTxMs) < DM_RESEND_TIMEOUT_MS)
    {
        return;
    }

    if (sendSos(pendingSos_.targetNode, pendingSos_.sequence, pendingSos_.cause))
        pendingSos_.lastTxMs = nowMs;
}

void DistanceMonitorModule::processNotifications(uint32_t nowMs)
{
    uint32_t pendingSequence = 0U;
    bool hasPending = false;
    bool allAcknowledged = true;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        DmNodeState &remote = nodeStates_[index];
        if (remote.isBase || remote.pendingNotificationSequence == 0U)
            continue;

        hasPending = true;
        if (pendingSequence == 0U)
            pendingSequence = remote.pendingNotificationSequence;

        if (remote.pendingNotificationSequence != pendingSequence || !remote.notificationAcked)
            allAcknowledged = false;

        if (dmElapsedMs(nowMs, remote.pendingNotificationSinceMs) >
            DM_NOTIFICATION_ACK_WINDOW_MS)
        {
            allAcknowledged = false;
        }
    }

    if (!hasPending)
        return;

    if (allAcknowledged)
    {
        audio_.playConfirmationBop();
        LOG_INFO(
            "{RX} notification ACK complete seq=%lu",
            static_cast<unsigned long>(pendingSequence));

        for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
        {
            DmNodeState &remote = nodeStates_[index];
            if (remote.pendingNotificationSequence == pendingSequence)
            {
                remote.pendingNotificationSequence = 0U;
                remote.notificationAcked = false;
            }
        }
        return;
    }

    bool expired = false;
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        const DmNodeState &remote = nodeStates_[index];
        if (remote.pendingNotificationSequence == pendingSequence &&
            dmElapsedMs(nowMs, remote.pendingNotificationSinceMs) >
                DM_NOTIFICATION_ACK_WINDOW_MS)
        {
            expired = true;
            break;
        }
    }

    if (!expired)
        return;

    LOG_WARN(
        "{Alarm} Cause=NotificationAckTimeout seq=%lu",
        static_cast<unsigned long>(pendingSequence));
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        DmNodeState &remote = nodeStates_[index];
        if (remote.pendingNotificationSequence == pendingSequence)
        {
            remote.pendingNotificationSequence = 0U;
            remote.notificationAcked = false;
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

    const double localLatitude = static_cast<double>(localState.latitudeI) * 1.0e-7;
    const double localLongitude = static_cast<double>(localState.longitudeI) * 1.0e-7;
    const double remoteLatitude = static_cast<double>(remoteState.latitudeI) * 1.0e-7;
    const double remoteLongitude = static_cast<double>(remoteState.longitudeI) * 1.0e-7;
    const double distance = dmCalculateDistanceMeters(
        localLatitude,
        localLongitude,
        remoteLatitude,
        remoteLongitude);
    const float ratio =
        runtimeConfig_.maxDistanceMeters > 0.0F
            ? static_cast<float>(distance / runtimeConfig_.maxDistanceMeters)
            : 0.0F;

    remoteState.distanceSource = DmDistanceSource::Gps;
    remoteState.distanceMeters = distance;
    remoteState.distanceRatio = ratio;
    remoteState.hasLastValidDistance = true;
    remoteState.lastValidDistanceMeters = distance;
    remoteState.lastValidDistanceRatio = ratio;
    remoteState.lastValidDistanceMs = nowMs;

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
    const bool tbValid =
        trackerToBase.isInitialized() &&
        dmElapsedMs(nowMs, trackerToBase.lastUpdateMs()) <= linkTimeoutMs();
    const uint32_t tbAgeMs = tbValid
                                 ? dmElapsedMs(nowMs, trackerToBase.lastUpdateMs())
                                 : UINT32_MAX;

    float trend = 0.0F;
    if (btValid && tbValid)
    {
        const float btQ = 1.0F /
                          std::max(
                              remoteState.remoteBaseRssiStdDb *
                                  remoteState.remoteBaseRssiStdDb,
                              1.0F);
        const float tbQ = 1.0F /
                          std::max(
                              trackerToBase.stdDb() * trackerToBase.stdDb(),
                              1.0F);
        trend = (btQ * remoteState.remoteBaseRssiTrendDbPerSec +
                 tbQ * trackerToBase.trendDbPerSec()) /
                (btQ + tbQ);
    }
    else if (btValid)
    {
        trend = remoteState.remoteBaseRssiTrendDbPerSec;
    }
    else if (tbValid)
    {
        trend = trackerToBase.trendDbPerSec();
    }

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
        updateDistanceAlertState(remoteState, 0.0F);
        return false;
    }

    const DmRssiTableEstimate table =
        currentRssiEstimate(remoteIndex, remoteState, localState, nowMs);
    remoteState.distanceSource = DmDistanceSource::Rssi;
    remoteState.distanceMeters = 0.0;
    remoteState.distanceRatio = 0.0F;
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
    const float previousActive = remoteState.activeDistanceAlertRatio;

    remoteState.currentDistanceAlertRatio =
        currentRatio >= DM_DISTANCE_ALERT_START_RATIO ? currentRatio : 0.0F;

    if (remoteState.distanceSource == DmDistanceSource::Gps)
    {
        if (currentRatio >= 1.0F)
        {
            remoteState.criticalDistanceLatch = true;
            remoteState.criticalDistanceLatchRatio =
                std::max(remoteState.criticalDistanceLatchRatio, currentRatio);
        }
        else if (currentRatio < DM_DISTANCE_ALERT_START_RATIO)
        {
            remoteState.criticalDistanceLatch = false;
            remoteState.criticalDistanceLatchRatio = 0.0F;
        }
    }

    float active = remoteState.currentDistanceAlertRatio;
    if (remoteState.criticalDistanceLatch &&
        remoteState.distanceSource != DmDistanceSource::Gps &&
        active < DM_DISTANCE_ALERT_START_RATIO)
    {
        active = std::max(1.0F, remoteState.criticalDistanceLatchRatio);
    }

    remoteState.activeDistanceAlertRatio = active;
    if (previousActive < DM_DISTANCE_ALERT_START_RATIO &&
        active >= DM_DISTANCE_ALERT_START_RATIO)
    {
        remoteState.hasDistanceBipTime = false;
        LOG_WARN(
            "{Alarm} Cause=Distance id=!%08lx source=%s ratio=%u%%",
            static_cast<unsigned long>(remoteState.nodeNum),
            dmDistanceSourceName(remoteState.distanceSource),
            static_cast<unsigned>(std::max(0.0F, active) * 100.0F + 0.5F));
    }
}

void DistanceMonitorModule::updateRssiCalibration(
    size_t remoteIndex,
    DmNodeState &remoteState,
    float distanceMeters,
    uint32_t nowMs)
{
    if (remoteIndex >= runtimeConfig_.memberCount ||
        remoteState.lastPositionReportSequence == 0U ||
        lastCalibrationSequence_[remoteIndex] == remoteState.lastPositionReportSequence)
    {
        return;
    }

    const DmRssiFilter &trackerToBase = directInboundRssi_[remoteIndex];
    const bool btValid = remoteState.remoteBaseRssiValid;
    const bool tbValid = trackerToBase.isUsable(nowMs, linkTimeoutMs());
    if (!btValid && !tbValid)
        return;

    if (rssiProfiles_[remoteIndex].update(
            distanceMeters,
            btValid,
            remoteState.remoteBaseRssiMeanDbm,
            tbValid,
            trackerToBase.lastSampleDbm()))
    {
        lastCalibrationSequence_[remoteIndex] = remoteState.lastPositionReportSequence;
    }
}

DmFaultCause DistanceMonitorModule::diagnoseRadioLoss(size_t remoteIndex) const
{
    const DmNodeState &remote = nodeStates_[remoteIndex];
    if (remote.shutdownNoticeReceived)
        return DmFaultCause::RemoteShutdown;

    if (remote.batteryPercent != DM_BATTERY_UNKNOWN &&
        remote.batteryPercent <= DM_LOW_BATTERY_PERCENT)
    {
        return DmFaultCause::RadioLowBattery;
    }

    const DmRssiFilter &trackerToBase = directInboundRssi_[remoteIndex];
    const bool rssiAvailable = trackerToBase.isInitialized();
    float fusedRssi = trackerToBase.meanDbm();
    float fusedTrend = trackerToBase.trendDbPerSec();

    if (remote.remoteBaseRssiValid && rssiAvailable)
    {
        fusedRssi = dmFuseRssi(remote.remoteBaseRssiMeanDbm, trackerToBase.meanDbm());
        fusedTrend = dmFuseRssiTrend(
            remote.remoteBaseRssiTrendDbPerSec,
            trackerToBase.trendDbPerSec());
    }

    if (remote.hasLastValidDistance &&
        remote.lastValidDistanceRatio > DM_RANGE_LOSS_MIN_RATIO &&
        rssiAvailable &&
        fusedTrend <= DM_RSSI_DEGRADING_TREND_DB_PER_S)
    {
        return DmFaultCause::RadioRangeLoss;
    }

    if (remote.hasLastValidDistance &&
        remote.lastValidDistanceRatio < DM_COM_SATURATION_MAX_RATIO &&
        rssiAvailable &&
        fusedRssi >= DM_RSSI_GOOD_DBM &&
        fusedTrend >= DM_RSSI_STABLE_TREND_DB_PER_S)
    {
        return DmFaultCause::RadioComSaturation;
    }

    return DmFaultCause::RadioUnexpectedLoss;
}

bool DistanceMonitorModule::isRadioFault(DmFaultCause cause) const
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
        return false;
    if (deadlinePending(nowMs, state.faultSnoozedUntilMs))
        return false;
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
               dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters)) *
               1000U +
           DM_LINK_TIMEOUT_MARGIN_MS;
}

uint32_t DistanceMonitorModule::rssiBeaconMaxAgeMs() const
{
    return DM_BASE_BEACON_INTERVAL_MS * DM_RSSI_VALID_BEACON_MULTIPLIER;
}

void DistanceMonitorModule::updateBaseAudio(size_t localIndex, uint32_t nowMs)
{
    bool hasSos = false;
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index != localIndex && nodeStates_[index].sosActive)
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
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (!isFaultAudible(remote, nowMs))
            continue;

        if (!remote.hasFaultAudioTime ||
            dmElapsedMs(nowMs, remote.lastFaultAudioMs) >= DM_FAULT_AUDIO_PERIOD_MS)
        {
            remote.lastFaultAudioMs = nowMs;
            remote.hasFaultAudioTime = true;
            audio_.playFaultBops();
            return;
        }
    }

    // SEARCH owns normal distance feedback while it is active. SOS and radio
    // faults above remain higher priority.
    if (searchModeActive_)
        return;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (remote.activeDistanceAlertRatio < DM_DISTANCE_ALERT_START_RATIO)
            continue;
        if (deadlinePending(nowMs, remote.distanceSnoozedUntilMs))
            continue;

        const uint32_t periodMs = dmDistanceBipIntervalMs(remote.activeDistanceAlertRatio);
        if (periodMs == 0U)
            continue;

        if (!remote.hasDistanceBipTime ||
            dmElapsedMs(nowMs, remote.lastDistanceBipMs) >= periodMs)
        {
            remote.lastDistanceBipMs = nowMs;
            remote.hasDistanceBipTime = true;
            audio_.playDistanceBip();
            return;
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
                readOk =
                    f.read(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate)) ==
                    sizeof(candidate);
                f.close();
            }
        }

        if (!readOk)
            continue;

        const uint32_t storedCrc = candidate.crc32;
        candidate.crc32 = 0U;
        const uint32_t computedCrc =
            dmCrc32(reinterpret_cast<const uint8_t *>(&candidate), sizeof(candidate));
        candidate.crc32 = storedCrc;

        const bool valid =
            candidate.magic == DM_RSSI_FILE_MAGIC &&
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
        LOG_DEBUG(
            "{RSSI} id=!%08lx calibration=empty",
            static_cast<unsigned long>(nodeNum));
        return false;
    }

    DmRssiCalibrationProfile &profile = rssiProfiles_[memberIndex];
    profile.reset();
    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        profile.bin(i).baseToTracker.restore(
            best.bins[i].bt.count,
            best.bins[i].bt.mean,
            best.bins[i].bt.m2);
        profile.bin(i).trackerToBase.restore(
            best.bins[i].tb.count,
            best.bins[i].tb.mean,
            best.bins[i].tb.m2);
    }

    rssiCalibrationGeneration_[memberIndex] = best.generation;
    LOG_DEBUG(
        "{RSSI} id=!%08lx calibration=loaded samples=%lu",
        static_cast<unsigned long>(nodeNum),
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
    for (size_t i = 0U; i < DM_RSSI_CALIBRATION_BIN_COUNT; ++i)
    {
        const DmRssiCalibrationBin &bin = profile.bin(i);
        record.bins[i].bt = {
            bin.baseToTracker.count(),
            bin.baseToTracker.mean(),
            bin.baseToTracker.m2()};
        record.bins[i].tb = {
            bin.trackerToBase.count(),
            bin.trackerToBase.mean(),
            bin.trackerToBase.m2()};
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
            ok = f.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record)) ==
                 sizeof(record);
            f.flush();
            f.close();
        }
    }

    if (!ok)
    {
        LOG_ERROR(
            "{RSSI} id=!%08lx calibration save failed",
            static_cast<unsigned long>(record.nodeNum));
        return false;
    }

    rssiCalibrationGeneration_[memberIndex] = record.generation;
    LOG_DEBUG(
        "{RSSI} id=!%08lx calibration=saved samples=%lu",
        static_cast<unsigned long>(record.nodeNum),
        static_cast<unsigned long>(profile.totalSamples()));
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
    }
}

void DistanceMonitorModule::saveAllRssiCalibration()
{
    for (size_t i = 0U; i < runtimeConfig_.memberCount; ++i)
    {
        if (!nodeStates_[i].isBase)
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
        searchModeActive_ = false;
        searchTargetIndex_ = DM_MAX_MEMBERS;
        hasSearchPulseTime_ = false;
        audio_.playSearchExit();
        LOG_INFO("{Search} off");
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
        LOG_WARN("{Search} no paired tracker");
        return true;
    }

    searchModeActive_ = true;
    searchTargetIndex_ = target;
    hasSearchPulseTime_ = false;
    audio_.playSearchEnter();
    LOG_INFO(
        "{Search} on id=!%08lx",
        static_cast<unsigned long>(nodeStates_[target].nodeNum));
    return true;
}

// SEARCH uses only GNSS distance. During a short NO_FIX it may reuse the last
// still-fresh GNSS distance, but it never falls back to RSSI proximity.
void DistanceMonitorModule::processSearchMode(size_t, uint32_t nowMs)
{
    if (!searchModeActive_ || searchTargetIndex_ >= runtimeConfig_.memberCount)
        return;

    const DmNodeState &remote = nodeStates_[searchTargetIndex_];
    if (!remote.paired || remote.sosActive || audio_.isSosActive())
        return;

    float proximity = 0.0F;
    bool hasUsableGnssDistance = false;
    float ratio = 0.0F;

    if (remote.distanceSource == DmDistanceSource::Gps)
    {
        ratio = remote.distanceRatio;
        hasUsableGnssDistance = true;
    }
    else if (remote.hasLastValidDistance &&
             dmElapsedMs(nowMs, remote.lastValidDistanceMs) <= linkTimeoutMs())
    {
        ratio = remote.lastValidDistanceRatio;
        hasUsableGnssDistance = true;
    }

    if (hasUsableGnssDistance)
    {
        ratio = std::max(0.0F, ratio);
        if (ratio <= DM_SEARCH_CONTACT_RATIO)
        {
            proximity = 1.0F;
        }
        else if (ratio < DM_SEARCH_DETECTION_MAX_RATIO)
        {
            proximity =
                (DM_SEARCH_DETECTION_MAX_RATIO - ratio) /
                (DM_SEARCH_DETECTION_MAX_RATIO - DM_SEARCH_CONTACT_RATIO);
        }
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

bool DistanceMonitorModule::handleSingleButtonPress()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return false;
    if (!nodeStates_[localIndex].isBase)
        return true;

    const uint32_t nowMs = millis();
    bool hadSos = false;
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
            continue;
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
        LOG_INFO("{Alarm} SOS acknowledged");
        return true;
    }

    bool hadFault = false;
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (!isFaultAudible(remote, nowMs))
            continue;

        const uint32_t snooze =
            remote.faultCause == DmFaultCause::RadioComSaturation
                ? DM_COM_SATURATION_SNOOZE_MS
                : DM_FAULT_SNOOZE_MS;
        remote.faultSnoozedUntilMs = nowMs + snooze;
        hadFault = true;
    }

    if (hadFault)
    {
        LOG_INFO("{Alarm} fault snoozed");
        return true;
    }

    bool hadDistanceAlert = false;
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (remote.activeDistanceAlertRatio < DM_DISTANCE_ALERT_START_RATIO)
            continue;

        hadDistanceAlert = true;
        if (remote.criticalDistanceLatch &&
            remote.currentDistanceAlertRatio < DM_DISTANCE_ALERT_START_RATIO)
        {
            remote.criticalDistanceLatch = false;
            remote.criticalDistanceLatchRatio = 0.0F;
            remote.activeDistanceAlertRatio = 0.0F;
        }
        else
        {
            remote.distanceSnoozedUntilMs = nowMs + DM_DISTANCE_SNOOZE_MS;
        }
    }

    if (hadDistanceAlert)
        LOG_INFO("{Alarm} Distance acknowledged");

    return true;
}

bool DistanceMonitorModule::handleDoubleButtonPress()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return false;

    if (!nodeStates_[localIndex].isBase)
    {
        triggerLocalSos(DmSosCause::ManualButton);
        return true;
    }

    const uint32_t sequence = allocateSequenceNumber();
    const uint32_t nowMs = millis();
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        DmNodeState &remote = nodeStates_[index];
        if (index == localIndex || remote.isBase)
            continue;

        remote.pendingNotificationSequence = sequence;
        remote.pendingNotificationSinceMs = nowMs;
        remote.notificationAcked = false;
        sendNotification(remote.nodeNum, sequence);
    }

    return true;
}

uint32_t DistanceMonitorModule::prepareLocalShutdown()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return 0U;

    if (nodeStates_[localIndex].isBase)
    {
        searchModeActive_ = false;
        searchTargetIndex_ = DM_MAX_MEMBERS;
        hasSearchPulseTime_ = false;
        saveAllRssiCalibration();
        return DM_SHUTDOWN_TX_GRACE_MS;
    }

    size_t baseIndex = 0U;
    if (!findBaseIndex(baseIndex))
        return 0U;

    sendShutdownNotice(runtimeConfig_.members[baseIndex].nodeNum);
    return DM_SHUTDOWN_TX_GRACE_MS;
}

void DistanceMonitorModule::logSummary(size_t localIndex, uint32_t) const
{
    const DmNodeState &local = nodeStates_[localIndex];
    char localBattery[8] = {};
    dmFormatBattery(local.batteryPercent, localBattery, sizeof(localBattery));

    LOG_INFO(
        "{%s} id=!%08lx pos=%s bat=%s state=%s",
        local.isBase ? "Node_base" : "Node_track",
        static_cast<unsigned long>(local.nodeNum),
        dmPositionKindName(local.positionKind),
        localBattery,
        localMotionName(localMoving_, fallFreefallArmed_, highSpeedSosLatched_));

    if (!local.isBase)
        return;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex || nodeStates_[index].isBase)
            continue;

        const DmNodeState &remote = nodeStates_[index];
        char battery[8] = {};
        dmFormatBattery(remote.batteryPercent, battery, sizeof(battery));

        if (remote.distanceSource == DmDistanceSource::Gps)
        {
            const unsigned pct = static_cast<unsigned>(
                std::max(0.0F, remote.distanceRatio) * 100.0F + 0.5F);
            LOG_INFO(
                "{Node_track} id=!%08lx pos=%s bat=%s state=%s dist=%.0fm (%u%%)",
                static_cast<unsigned long>(remote.nodeNum),
                dmPositionKindName(remote.positionKind),
                battery,
                remoteMotionName(remote),
                remote.distanceMeters,
                pct);
        }
        else if (remote.distanceSource == DmDistanceSource::Rssi)
        {
            const unsigned pct = static_cast<unsigned>(
                std::max(0.0F, std::min(1.0F, remote.rssiEstimate.confidence)) *
                    100.0F +
                0.5F);
            LOG_INFO(
                "{Node_track} id=!%08lx pos=%s bat=%s state=%s dist=%s (%u%%)",
                static_cast<unsigned long>(remote.nodeNum),
                dmPositionKindName(remote.positionKind),
                battery,
                remoteMotionName(remote),
                dmDistanceBandName(remote.rssiEstimate.band),
                pct);
        }
        else
        {
            LOG_INFO(
                "{Node_track} id=!%08lx pos=%s bat=%s state=%s dist=Unknown",
                static_cast<unsigned long>(remote.nodeNum),
                dmPositionKindName(remote.positionKind),
                battery,
                remoteMotionName(remote));
        }
    }
}
