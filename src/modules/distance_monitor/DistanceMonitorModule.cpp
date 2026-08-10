#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "configuration.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

DistanceMonitorModule *distanceMonitorModule = nullptr;

namespace
{
bool deadlinePending(uint32_t nowMs, uint32_t deadlineMs)
{
    return deadlineMs != 0U &&
           static_cast<int32_t>(nowMs - deadlineMs) < 0;
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
} // namespace

DistanceMonitorModule::DistanceMonitorModule()
    : SinglePortModule("distance_monitor", meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("DistanceMonitor")
{
    loadDefaultConfiguration();

    // Meshtastic core modules are instantiated from setupModules(), but their
    // MeshModule::setup() hook is not called automatically. Initialize here so
    // boot/session/audio state exists before the first packet can be handled.
    initializeIfNeeded();
}

void DistanceMonitorModule::setup()
{
    // Keep this hook for compatibility with integrations that explicitly call
    // MeshModule::setup(). Initialization is deliberately idempotent.
    initializeIfNeeded();
}

void DistanceMonitorModule::initializeIfNeeded()
{
    if (moduleInitialized_)
    {
        return;
    }

    bootMs_ = millis();

    uint32_t localNodeNum = 0U;
    if (nodeDB != nullptr)
    {
        localNodeNum = nodeDB->getNodeNum();
    }

    bootSessionId_ =
        static_cast<uint32_t>(micros()) ^
        (bootMs_ << 11U) ^
        localNodeNum ^
        0xD157A9C5U;
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
        "tick=%lums dmax=%.1fm report=%u..%us beacon=%lus session=%08lx",
        DM_FIRMWARE_VERSION,
        static_cast<unsigned>(DM_PROTOCOL_VERSION),
        static_cast<unsigned>(runtimeConfig_.memberCount),
        static_cast<unsigned long>(DM_TICK_INTERVAL_MS),
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
        LOG_WARN(
            "Distance Monitor: local node !%08lx is not configured",
            static_cast<unsigned long>(nodeDB->getNodeNum()));
        return DM_TICK_INTERVAL_MS;
    }

    const uint32_t nowMs = millis();
    DmNodeState &localState = nodeStates_[localIndex];
    localState.isLocal = true;
    localState.batteryPercent = currentBatteryPercent();

    if (!localPositionStarted_)
    {
        startLocalPositionManager(nowMs);
    }
    updateLocalPosition(localState, nowMs);

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
        dmElapsedMs(nowMs, lastSummaryLogMs_) >=
            DM_SUMMARY_INTERVAL_MS)
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
    // Keep standard Meshtastic feedback from competing with Distance Monitor tones.
    config.device.role =
        meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
    config.device.buzzer_mode =
        meshtastic_Config_DeviceConfig_BuzzerMode_DISABLED;
    config.device.disable_triple_click = true;
    config.device.led_heartbeat_disabled = false;

    // Native position broadcasts are reduced; Distance Monitor owns its private reports.
    config.position.position_broadcast_smart_enabled = false;
    config.position.position_broadcast_secs =
        DM_GPS_SLEEP_UPDATE_INTERVAL_S;

    // These are runtime-only changes. The user's persistent configuration is not rewritten.
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
            // Expected session IDs are stored before confirmation arrives.
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

    // RSSI beacons are useful background traffic, but never compete with an
    // active SOS. At the default radio preset a beacon occupies substantial
    // airtime, so the normal cadence is intentionally conservative.
    if (!hasActiveSos &&
        (!hasBaseBeaconTxTime_ ||
         dmElapsedMs(nowMs, lastBaseBeaconTxMs_) >=
             DM_BASE_BEACON_INTERVAL_MS))
    {
        if (sendBaseBeacon())
        {
            lastBaseBeaconTxMs_ = nowMs;
            hasBaseBeaconTxTime_ = true;
        }
    }

    for (size_t remoteIndex = 0U;
         remoteIndex < runtimeConfig_.memberCount;
         ++remoteIndex)
    {
        if (remoteIndex == localIndex ||
            nodeStates_[remoteIndex].isBase)
        {
            continue;
        }

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

    const uint32_t reportPeriodMs =
        static_cast<uint32_t>(
            std::max<uint8_t>(
                localAppliedIntervalSec_,
                DM_MIN_REPORT_INTERVAL_S)) *
        1000U;

    if (forcePositionReport_ ||
        !hasPositionReportTxTime_ ||
        dmElapsedMs(nowMs, lastPositionReportTxMs_) >=
            reportPeriodMs)
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
    if (remote.radioState == DmRadioState::Lost ||
        !remote.hasPositionReportRxTime)
    {
        return;
    }

    evaluateRemoteDistance(localIndex, remoteIndex, nowMs);

    // Interval commands are evaluated only when a new report is received.
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
    if (!remote.hasPositionReportRxTime)
    {
        return;
    }

    const uint32_t ageMs =
        dmElapsedMs(nowMs, remote.lastPositionReportRxMs);
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
            remote.faultCause =
                diagnoseRadioLoss(remoteIndex);
            remote.faultStartedMs = nowMs;
            remote.hasFaultAudioTime = false;

            if (remote.faultCause ==
                DmFaultCause::RadioComSaturation)
            {
                remote.comSaturationSilentUntilMs =
                    nowMs +
                    DM_COM_SATURATION_SILENT_MS;
            }

            LOG_WARN(
                "Distance Monitor link lost: node=!%08lx cause=%s "
                "age=%lus last_ratio=%.2f battery=%u",
                static_cast<unsigned long>(remote.nodeNum),
                dmFaultCauseName(remote.faultCause),
                static_cast<unsigned long>(ageMs / 1000U),
                static_cast<double>(remote.lastValidDistanceRatio),
                static_cast<unsigned>(remote.batteryPercent));
        }
    }

    // A live tracker publishes POSITION_REPORT even with NO_FIX, so recovery
    // is detected passively by the next report. Probing a bad link would only
    // add traffic and can keep a congested radio in a failure loop.
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
            remote.lastValidDistanceRatio,
            nowMs);
        return;
    }

    evaluateRssiDistance(
        local,
        remoteIndex,
        remote,
        nowMs);
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

bool DistanceMonitorModule::evaluateRssiDistance(
    const DmNodeState &localState,
    size_t remoteIndex,
    DmNodeState &remoteState,
    uint32_t nowMs)
{
    const DmRssiFilter &trackerToBase =
        directInboundRssi_[remoteIndex];

    if (!remoteState.remoteBaseRssiValid ||
        !trackerToBase.isUsable(nowMs, linkTimeoutMs()))
    {
        const bool wasFaulted =
            remoteState.distanceEstimateFault;

        remoteState.distanceSource =
            DmDistanceSource::None;
        remoteState.rssiEstimate =
            DmDistanceBandEstimate{};
        remoteState.distanceEstimateFault = true;
        remoteState.currentDistanceAlertRatio = 0.0F;

        if (!wasFaulted)
        {
            remoteState.hasFaultAudioTime = false;
            LOG_WARN(
                "Distance Monitor fallback unavailable: node=!%08lx "
                "reason=RSSI_DATA_UNAVAILABLE",
                static_cast<unsigned long>(remoteState.nodeNum));
        }
        return false;
    }

    const float fusedRssi = dmFuseRssi(
        remoteState.remoteBaseRssiMeanDbm,
        trackerToBase.meanDbm());
    const float fusedTrend = dmFuseRssiTrend(
        remoteState.remoteBaseRssiTrendDbPerSec,
        trackerToBase.trendDbPerSec());
    const bool moving =
        localState.moving || remoteState.moving;

    const DmDistanceBandEstimate estimate =
        rssiCalibration_.estimate(
            fusedRssi,
            fusedTrend,
            moving);

    remoteState.distanceSource = DmDistanceSource::Rssi;
    remoteState.distanceMeters = 0.0;
    remoteState.distanceRatio = 0.0F;
    remoteState.rssiEstimate = estimate;

    if (estimate.band == DmDistanceBand::Unknown)
    {
        const bool wasFaulted =
            remoteState.distanceEstimateFault;

        remoteState.distanceEstimateFault = true;
        remoteState.currentDistanceAlertRatio = 0.0F;

        if (!wasFaulted)
        {
            remoteState.hasFaultAudioTime = false;
            LOG_WARN(
                "Distance Monitor fallback UNKNOWN: node=!%08lx "
                "rssi=%.1f trend=%.2f confidence=%.2f",
                static_cast<unsigned long>(remoteState.nodeNum),
                static_cast<double>(estimate.fusedRssiDbm),
                static_cast<double>(estimate.fusedTrendDbPerSec),
                static_cast<double>(estimate.confidence));
        }
        return false;
    }

    if (remoteState.distanceEstimateFault)
    {
        LOG_INFO(
            "Distance Monitor fallback recovered: node=!%08lx band=%s",
            static_cast<unsigned long>(remoteState.nodeNum),
            dmDistanceBandName(estimate.band));
    }

    remoteState.distanceEstimateFault = false;
    updateDistanceAlertState(
        remoteState,
        effectiveAlertRatioFromBand(estimate.band));
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
            // A fresh GPS distance below 80% is reliable enough to clear the latch.
            remoteState.criticalDistanceLatch = false;
            remoteState.criticalDistanceLatchRatio = 0.0F;
        }
    }

    float active =
        remoteState.currentDistanceAlertRatio;

    // If GPS previously crossed Dmax, a weaker RSSI fallback cannot silently clear it.
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
    float distanceRatio,
    uint32_t nowMs)
{
    const DmRssiFilter &trackerToBase =
        directInboundRssi_[remoteIndex];

    if (!remoteState.remoteBaseRssiValid ||
        !trackerToBase.isUsable(nowMs, linkTimeoutMs()))
    {
        return;
    }

    const float fusedRssi =
        dmFuseRssi(
            remoteState.remoteBaseRssiMeanDbm,
            trackerToBase.meanDbm());

    rssiCalibration_.update(
        dmDistanceBandFromRatio(distanceRatio),
        fusedRssi);
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
    const bool hasRadioFault =
        isRadioFault(state.faultCause);

    if (!hasRadioFault &&
        !state.distanceEstimateFault)
    {
        return false;
    }

    if (deadlinePending(
            nowMs,
            state.faultSnoozedUntilMs))
    {
        return false;
    }

    if (state.faultCause ==
            DmFaultCause::RadioComSaturation &&
        deadlinePending(
            nowMs,
            state.comSaturationSilentUntilMs))
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

bool DistanceMonitorModule::handleSingleButtonPress()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
    {
        return false;
    }

    // Distance Monitor owns the configured tracker's single press and keeps it silent.
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
            // The only remaining alarm is the remembered GPS >100% state.
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
    size_t baseIndex = 0U;

    if (!findLocalIndex(localIndex))
    {
        return 0U;
    }

    if (nodeStates_[localIndex].isBase ||
        !findBaseIndex(baseIndex))
    {
        return 0U;
    }

    sendShutdownNotice(
        runtimeConfig_.members[baseIndex].nodeNum);

    LOG_WARN(
        "Distance Monitor local shutdown notice sent");

    return DM_SHUTDOWN_TX_GRACE_MS;
}

void DistanceMonitorModule::logSummary(
    size_t localIndex,
    uint32_t nowMs) const
{
    const DmNodeState &local =
        nodeStates_[localIndex];

    const char *localStateName = "STATIONARY";
    if (highSpeedSosLatched_)
    {
        localStateName = "IN_VEHICLE";
    }
    else if (fallFreefallArmed_)
    {
        localStateName = "FALLING";
    }
    else if (localMoving_)
    {
        localStateName = "MOVING";
    }

    LOG_INFO(
        "Self: node=!%08lx role=%s pos=%s battery=%u%% state=%s",
        static_cast<unsigned long>(local.nodeNum),
        local.isBase ? "base" : "tracker",
        dmPositionKindName(local.positionKind),
        static_cast<unsigned>(local.batteryPercent),
        localStateName);

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        const DmNodeState &remote =
            nodeStates_[index];

        uint32_t reportAgeSeconds = 0U;
        if (remote.hasPositionReportRxTime)
        {
            reportAgeSeconds =
                dmElapsedMs(
                    nowMs,
                    remote.lastPositionReportRxMs) /
                1000U;
        }

        if (remote.distanceSource == DmDistanceSource::Gps)
        {
            const unsigned distancePercent =
                static_cast<unsigned>(
                    std::lround(
                        std::max(0.0F, remote.distanceRatio) *
                        100.0F));

            LOG_INFO(
                "Member: node=!%08lx paired=%s radio=%s report_age=%lus "
                "pos=%s dist=%.1fm (%u%%)",
                static_cast<unsigned long>(remote.nodeNum),
                remote.paired ? "yes" : "no",
                dmRadioStateName(remote.radioState),
                static_cast<unsigned long>(reportAgeSeconds),
                dmPositionKindName(remote.positionKind),
                remote.distanceMeters,
                distancePercent);
        }
        else if (remote.distanceSource == DmDistanceSource::Rssi)
        {
            LOG_INFO(
                "Member: node=!%08lx paired=%s radio=%s report_age=%lus "
                "pos=%s band=%s conf=%.2f",
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
                "Member: node=!%08lx paired=%s radio=%s report_age=%lus "
                "pos=%s distance=UNAVAILABLE",
                static_cast<unsigned long>(remote.nodeNum),
                remote.paired ? "yes" : "no",
                dmRadioStateName(remote.radioState),
                static_cast<unsigned long>(reportAgeSeconds),
                dmPositionKindName(remote.positionKind));
        }
    }
}
