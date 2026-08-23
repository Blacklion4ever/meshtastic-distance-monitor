#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "configuration.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

DistanceMonitorModule *distanceMonitorModule = nullptr;

namespace
{
bool deadlinePending(uint32_t nowMs, uint32_t deadlineMs)
{
    return deadlineMs != 0U &&
           static_cast<int32_t>(nowMs - deadlineMs) < 0;
}

const char *localMotionName(
    bool moving,
    bool fallDetected,
    bool highSpeedLatched)
{
    if (highSpeedLatched)
        return "IN_VEHICLE";
    if (fallDetected)
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
} // namespace

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
    if (service != nullptr)
        service->setPhoneForwardingSuppressedPort(meshtastic_PortNum_PRIVATE_APP);
}

void DistanceMonitorModule::setupStatusLed()
{
    if (!DM_STATUS_LED_ENABLED)
        return;

#if defined(LED_POWER)
    statusLedPin_ = static_cast<int>(LED_POWER);
#elif defined(PIN_LED1)
    statusLedPin_ = static_cast<int>(PIN_LED1);
#else
    statusLedPin_ = -1;
#endif

    if (statusLedPin_ < 0)
        return;

    pinMode(statusLedPin_, OUTPUT);
    setStatusLed(false);
    lastLedHeartbeatMs_ = millis();
    hasLedHeartbeatTime_ = true;
}

void DistanceMonitorModule::setStatusLed(bool on)
{
    if (statusLedPin_ < 0)
        return;

#if defined(LED_STATE_ON)
    const int onLevel = LED_STATE_ON;
#else
    const int onLevel = HIGH;
#endif

    uint8_t brightness = DM_LED_BASE_BRIGHTNESS;
    size_t localIndex = 0U;
    if (findLocalIndex(localIndex) && !nodeStates_[localIndex].isBase)
        brightness = DM_LED_TRACKER_BRIGHTNESS;

#if defined(ARCH_NRF52)
    // nRF52 Arduino exposes PWM through analogWrite(). Only use PWM when
    // dimming is actually requested; otherwise preserve the normal digital LED.
    if (brightness < 255U)
    {
        const uint32_t duty =
            onLevel == HIGH
                ? (on ? brightness : 0U)
                : (on ? 255U - brightness : 255U);
        analogWrite(static_cast<uint32_t>(statusLedPin_), duty);
        return;
    }
#endif

    digitalWrite(
        statusLedPin_,
        on ? onLevel : (onLevel == HIGH ? LOW : HIGH));
}

void DistanceMonitorModule::triggerStatusLedDoubleBlink()
{
    if (statusLedPin_ < 0)
        return;

    const uint32_t nowMs = millis();
    ledEventPhase_ = 1U;
    setStatusLed(true);
    ledTransitionMs_ = nowMs + DM_LED_EVENT_ON_MS;
}

void DistanceMonitorModule::serviceStatusLed(uint32_t nowMs)
{
    if (statusLedPin_ < 0)
        return;

    const bool transitionDue =
        ledTransitionMs_ != 0U &&
        static_cast<int32_t>(nowMs - ledTransitionMs_) >= 0;

    if (ledEventPhase_ != 0U && transitionDue)
    {
        switch (ledEventPhase_)
        {
        case 1U:
            setStatusLed(false);
            ledEventPhase_ = 2U;
            ledTransitionMs_ = nowMs + DM_LED_EVENT_OFF_MS;
            return;
        case 2U:
            setStatusLed(true);
            ledEventPhase_ = 3U;
            ledTransitionMs_ = nowMs + DM_LED_EVENT_ON_MS;
            return;
        case 3U:
            setStatusLed(false);
            ledEventPhase_ = 0U;
            ledTransitionMs_ = 0U;
            lastLedHeartbeatMs_ = nowMs;
            hasLedHeartbeatTime_ = true;
            return;
        case 4U:
            setStatusLed(false);
            ledEventPhase_ = 0U;
            ledTransitionMs_ = 0U;
            return;
        default:
            setStatusLed(false);
            ledEventPhase_ = 0U;
            ledTransitionMs_ = 0U;
            return;
        }
    }

    if (ledEventPhase_ != 0U)
        return;

    if (!hasLedHeartbeatTime_ ||
        dmElapsedMs(nowMs, lastLedHeartbeatMs_) >=
            DM_LED_HEARTBEAT_INTERVAL_MS)
    {
        lastLedHeartbeatMs_ = nowMs;
        hasLedHeartbeatTime_ = true;
        setStatusLed(true);
        ledEventPhase_ = 4U;
        ledTransitionMs_ = nowMs + DM_LED_HEARTBEAT_ON_MS;
    }
}

void DistanceMonitorModule::initializeIfNeeded()
{
    if (moduleInitialized_)
        return;

    bootMs_ = millis();
    const uint32_t localNodeNum =
        nodeDB != nullptr ? nodeDB->getNodeNum() : 0U;

    // A 16-bit boot session saves two bytes from every session-bearing packet.
    // Mix several boot-variant values before truncating so successive restarts
    // of the same node are very unlikely to reuse the previous session.
    uint32_t sessionSeed =
        static_cast<uint32_t>(micros()) ^
        (bootMs_ << 11U) ^
        localNodeNum ^
        0xD157A9C5U;
    sessionSeed ^= sessionSeed >> 16U;
    bootSessionId_ = static_cast<DmSessionId>(sessionSeed & 0xFFFFU);
    if (bootSessionId_ == 0U)
        bootSessionId_ = 1U;

    localAppliedIntervalSec_ =
        dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);

    applyRuntimeProfile();
    audio_.setup();
    setupStatusLed();
    moduleInitialized_ = true;

    LOG_INFO(
        "{DM@Init} schema=%u firmware=%s protocol=%u session=%04x audio=%s silent=%s",
        static_cast<unsigned>(DM_LOG_SCHEMA),
        DM_FIRMWARE_VERSION,
        static_cast<unsigned>(DM_PROTOCOL_VERSION),
        static_cast<unsigned>(bootSessionId_),
        DM_ALARM_AUDIO_SILENT ? "SILENT" : "AUDIBLE",
        DM_ALARM_AUDIO_SILENT ? "YES" : "NO");

    // The buzzer is still useful in silent alarm mode for pairing and
    // notifications, so report a missing buzzer in either mode.
    if (!audio_.isAvailable())
        LOG_ERROR("{DM@Alarm} Cause=BuzzerUnavailable silent=%s",
                  DM_ALARM_AUDIO_SILENT ? "YES" : "NO");

    setIntervalFromNow(DM_TICK_INTERVAL_MS);
}

int32_t DistanceMonitorModule::runOnce()
{
    initializeIfNeeded();

    const uint32_t nowMs = millis();
    serviceStatusLed(nowMs);

    if (nodeDB == nullptr || runtimeConfig_.memberCount == 0U)
        return DM_TICK_INTERVAL_MS;

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return DM_TICK_INTERVAL_MS;

    DmNodeState &localState = nodeStates_[localIndex];

    if (!localPositionStarted_)
        startLocalPositionManager(nowMs);

    sampleLocalImu(localState, nowMs);

    if (hasControlTickTime_ &&
        dmElapsedMs(nowMs, lastControlTickMs_) < DM_CONTROL_INTERVAL_MS)
    {
        return DM_TICK_INTERVAL_MS;
    }

    lastControlTickMs_ = nowMs;
    hasControlTickTime_ = true;
    localState.batteryPercent = currentBatteryPercent();
    updateLocalPosition(localState, nowMs);
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
    runtimeConfig_.memberCount = DM_DEFAULT_MEMBER_COUNT;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        runtimeConfig_.members[index].nodeNum =
            DM_DEFAULT_MEMBERS[index].nodeNum;
        runtimeConfig_.members[index].isBase =
            DM_DEFAULT_MEMBERS[index].isBase;

        nodeStates_[index] = DmNodeState{};
        nodeStates_[index].nodeNum = DM_DEFAULT_MEMBERS[index].nodeNum;
        nodeStates_[index].isBase = DM_DEFAULT_MEMBERS[index].isBase;
        nodeStates_[index].batteryPercent = DM_BATTERY_UNKNOWN;
        nodeStates_[index].positionAccuracyMeters =
            std::numeric_limits<double>::infinity();
    }
}

void DistanceMonitorModule::applyRuntimeProfile()
{
    config.device.role = meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
    config.device.buzzer_mode =
        meshtastic_Config_DeviceConfig_BuzzerMode_DISABLED;
    config.device.disable_triple_click = true;
    config.device.led_heartbeat_disabled = true;

    config.position.position_broadcast_smart_enabled = false;
    config.position.position_broadcast_secs =
        DM_NATIVE_POSITION_BROADCAST_INTERVAL_S;

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
        return false;

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
        return DM_BATTERY_UNKNOWN;

    return std::min<uint8_t>(
        100U,
        powerStatus->getBatteryChargePercent());
}

// The base owns pairing and periodically reasserts it so both ends recover
// from a missed confirmation or a tracker-side RAM state loss.
void DistanceMonitorModule::processPairing(
    size_t localIndex,
    uint32_t nowMs)
{
    DmNodeState &localState = nodeStates_[localIndex];
    if (!localState.isBase)
        return;

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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
                if (sendPairConfirm(
                        remote.nodeNum,
                        remote.remoteSessionId,
                        false))
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

        if (uptimeSeconds(nowMs) * 1000U < DM_PAIR_CONFIRM_DELAY_MS)
            continue;

        if (remote.hasHandshakeTxTime &&
            dmElapsedMs(nowMs, remote.lastHandshakeTxMs) <
                DM_HANDSHAKE_RETRY_MS)
        {
            continue;
        }

        const bool trackerJustBooted =
            remote.remoteUptimeSeconds <= DM_PAIR_REMOTE_BOOT_WINDOW_S;

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
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index != localIndex && nodeStates_[index].sosActive)
        {
            hasActiveSos = true;
            break;
        }
    }

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
            dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);
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
        case DmDistanceBand::VeryFar:
            desired = DM_MIN_REPORT_INTERVAL_S;
            break;
        case DmDistanceBand::Warning:
            desired = dmComputeReportIntervalSec(
                DM_RSSI_WARNING_ALERT_RATIO,
                runtimeConfig_.maxDistanceMeters);
            break;
        case DmDistanceBand::Medium:
            desired = dmComputeReportIntervalSec(
                DM_RSSI_WARNING_ALERT_RATIO*0.5F,
                runtimeConfig_.maxDistanceMeters);
            break;
        case DmDistanceBand::Near:
            desired = dmComputeReportIntervalSec(
                DM_RSSI_WARNING_ALERT_RATIO*0.25F,
                runtimeConfig_.maxDistanceMeters);
            break;
        case DmDistanceBand::Unknown:
        default:
            desired = DM_MIN_REPORT_INTERVAL_S;
            break;
        }
    }

    remote.lastIntervalEvaluationSequence =
        remote.lastPositionReportSequence;

    if (remote.appliedIntervalSec != desired && remote.hasRemoteSession)
    {
        sendSetPositionInterval(
            remote.nodeNum,
            remote.remoteSessionId,
            desired);
    }
}

// Link alarms are valid only after pairing. Before pairing, missing reports are
// part of the handshake state and must not be presented as a radio failure.
void DistanceMonitorModule::processLinkState(
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &remote = nodeStates_[remoteIndex];
    if (!remote.hasPositionReportRxTime)
        return;

    const uint32_t ageMs =
        dmElapsedMs(nowMs, remote.lastPositionReportRxMs);
    const uint32_t timeoutMs = linkTimeoutMs(remote.appliedIntervalSec);
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
        {
            remote.comSaturationSilentUntilMs =
                nowMs + DM_COM_SATURATION_SILENT_MS;
        }

        LOG_WARN(
            "{DM@Alarm} Cause=%s id=!%08lx age=%lus silent=%s",
            dmFaultCauseName(remote.faultCause),
            static_cast<unsigned long>(remote.nodeNum),
            static_cast<unsigned long>(ageMs / 1000U),
            DM_ALARM_AUDIO_SILENT ? "YES" : "NO");
    }
}

void DistanceMonitorModule::processPendingSos(uint32_t nowMs)
{
    if (!pendingSos_.active ||
        dmElapsedMs(nowMs, pendingSos_.lastTxMs) < DM_RESEND_TIMEOUT_MS)
    {
        return;
    }

    if (sendSos(
            pendingSos_.targetNode,
            pendingSos_.sequence,
            pendingSos_.cause))
    {
        pendingSos_.lastTxMs = nowMs;
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
        if (remote.isBase || remote.pendingNotificationSequence == 0U)
            continue;

        hasPending = true;
        if (pendingSequence == 0U)
            pendingSequence = remote.pendingNotificationSequence;

        if (remote.pendingNotificationSequence != pendingSequence ||
            !remote.notificationAcked)
        {
            allAcknowledged = false;
        }

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
        // User-command confirmation remains audible in silent alarm mode.
        audio_.playConfirmationBop();
        LOG_INFO(
            "{DM@RX} notification ACK complete seq=%lu",
            static_cast<unsigned long>(pendingSequence));

        for (size_t index = 0U;
             index < runtimeConfig_.memberCount;
             ++index)
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
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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
        "{DM@Alarm} Cause=NotificationAckTimeout seq=%lu silent=%s",
        static_cast<unsigned long>(pendingSequence),
        DM_ALARM_AUDIO_SILENT ? "YES" : "NO");

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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

    // GPS has priority only when its combined DOP-derived accuracy is useful
    // relative to Dmax. Otherwise RSSI becomes the safety decision source.
    if (evaluateGpsDistance(local, remote, nowMs))
        return;

    evaluateRssiDistance(remoteIndex, remote, nowMs);
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

    const double combinedAccuracyM =
        localState.positionAccuracyMeters +
        remoteState.positionAccuracyMeters;
    const double maximumUsableAccuracyM =
        static_cast<double>(runtimeConfig_.maxDistanceMeters) *
        static_cast<double>(DM_GPS_MAX_COMBINED_ACCURACY_RATIO);

    if (!std::isfinite(combinedAccuracyM) ||
        combinedAccuracyM > maximumUsableAccuracyM)
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

    const double rawDistanceM = dmCalculateDistanceMeters(
        localLatitude,
        localLongitude,
        remoteLatitude,
        remoteLongitude);

    if (combinedAccuracyM >= rawDistanceM)
        return false;

    // Safety best case: use the minimum plausible separation so GNSS error
    // cannot create an unnecessary distance alarm. If uncertainty itself is
    // too large, the function already returned false and RSSI is used instead.
    const double safeDistanceM =
        std::max(0.0, rawDistanceM - combinedAccuracyM);
    const float ratio =
        runtimeConfig_.maxDistanceMeters > 0.0F
            ? static_cast<float>(
                  safeDistanceM /
                  static_cast<double>(runtimeConfig_.maxDistanceMeters))
            : 0.0F;

    remoteState.distanceSource = DmDistanceSource::Gps;
    remoteState.distanceMeters = safeDistanceM;
    remoteState.rawDistanceMeters = rawDistanceM;
    remoteState.combinedAccuracyMeters = combinedAccuracyM;
    remoteState.distanceRatio = ratio;
    remoteState.rssiEstimate = DmDistanceBandEstimate{};

    // Keep the last valid GPS distance for radio-loss diagnosis.
    remoteState.hasLastValidDistance = true;
    remoteState.lastValidDistanceMeters = safeDistanceM;
    remoteState.lastValidDistanceRatio = ratio;
    remoteState.lastValidDistanceMs = nowMs;

    updateDistanceAlertState(remoteState, ratio);
    return true;
}

bool DistanceMonitorModule::evaluateRssiDistance(
    size_t remoteIndex,
    DmNodeState &remoteState,
    uint32_t nowMs)
{
    const DmRssiFilter &trackerToBase = directInboundRssi_[remoteIndex];
    const bool btValid = remoteState.remoteBaseRssiValid;
    const bool tbValid =
        trackerToBase.isInitialized() &&
        dmElapsedMs(nowMs, trackerToBase.lastUpdateMs()) <=
            linkTimeoutMs(remoteState.appliedIntervalSec);

    float bestRssiDbm = 0.0F;
    if (!dmSelectBestRssi(
            btValid,
            remoteState.remoteBaseRssiDbm,
            tbValid,
            trackerToBase.meanDbm(),
            bestRssiDbm))
    {
        remoteState.distanceSource = DmDistanceSource::None;
        remoteState.distanceMeters = 0.0;
        remoteState.rawDistanceMeters = 0.0;
        remoteState.combinedAccuracyMeters = 0.0;
        remoteState.distanceRatio = 0.0F;
        remoteState.rssiEstimate = DmDistanceBandEstimate{};
        updateDistanceAlertState(remoteState, 0.0F);
        return false;
    }

    const DmDistanceBand band = dmDistanceBandFromRssi(bestRssiDbm);
    const float alertRatio = dmRssiAlertRatio(band);

    remoteState.distanceSource = DmDistanceSource::Rssi;
    remoteState.distanceMeters = 0.0;
    remoteState.rawDistanceMeters = 0.0;
    remoteState.combinedAccuracyMeters = 0.0;
    remoteState.distanceRatio = alertRatio;
    remoteState.rssiEstimate.band = band;
    remoteState.rssiEstimate.bestRssiDbm = bestRssiDbm;

    updateDistanceAlertState(remoteState, alertRatio);
    return true;
}

void DistanceMonitorModule::updateDistanceAlertState(
    DmNodeState &remoteState,
    float currentRatio)
{
    const float previousActive = remoteState.activeDistanceAlertRatio;
    remoteState.currentDistanceAlertRatio = currentRatio;

    // Preserve the original GPS critical latch behavior: once the measured
    // distance has genuinely crossed Dmax, a transient GPS improvement cannot
    // immediately silence the critical alarm without user acknowledgement.
    if (remoteState.distanceSource == DmDistanceSource::Gps &&
        currentRatio >= 1.0F)
    {
        remoteState.criticalDistanceLatch = true;
        remoteState.criticalDistanceLatchRatio =
            std::max(remoteState.criticalDistanceLatchRatio, currentRatio);
    }

    float active = currentRatio;
    if (remoteState.criticalDistanceLatch)
    {
        active = std::max(active, remoteState.criticalDistanceLatchRatio);
    }

    remoteState.activeDistanceAlertRatio = active;

    if (previousActive < DM_DISTANCE_ALERT_START_RATIO &&
        active >= DM_DISTANCE_ALERT_START_RATIO)
    {
        remoteState.hasDistanceBipTime = false;

        if (remoteState.distanceSource == DmDistanceSource::Rssi)
        {
            LOG_WARN(
                "{DM@Alarm} Cause=Distance id=!%08lx source=RSSI band=%s RSSI=%.0fdBm ratio=%u%% silent=%s",
                static_cast<unsigned long>(remoteState.nodeNum),
                dmDistanceBandName(remoteState.rssiEstimate.band),
                static_cast<double>(remoteState.rssiEstimate.bestRssiDbm),
                static_cast<unsigned>(active * 100.0F + 0.5F),
                DM_ALARM_AUDIO_SILENT ? "YES" : "NO");
        }
        else
        {
            LOG_WARN(
                "{DM@Alarm} Cause=Distance id=!%08lx source=%s dist=%.0fm raw=%.0fm acc=%.0fm ratio=%u%% silent=%s",
                static_cast<unsigned long>(remoteState.nodeNum),
                dmDistanceSourceName(remoteState.distanceSource),
                remoteState.distanceMeters,
                remoteState.rawDistanceMeters,
                remoteState.combinedAccuracyMeters,
                static_cast<unsigned>(active * 100.0F + 0.5F),
                DM_ALARM_AUDIO_SILENT ? "YES" : "NO");
        }
    }
}

DmFaultCause DistanceMonitorModule::diagnoseRadioLoss(
    size_t remoteIndex) const
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
    const bool tbAvailable = trackerToBase.isInitialized();
    const bool btAvailable = remote.remoteBaseRssiValid;

    float fusedRssi = 0.0F;
    const bool rssiAvailable = dmSelectBestRssi(
        btAvailable,
        remote.remoteBaseRssiDbm,
        tbAvailable,
        trackerToBase.meanDbm(),
        fusedRssi);

    const float fusedTrend =
        tbAvailable ? trackerToBase.trendDbPerSec() : 0.0F;

    if (remote.hasLastValidDistance &&
        remote.lastValidDistanceRatio > DM_RANGE_LOSS_MIN_RATIO &&
        tbAvailable &&
        fusedTrend <= DM_RSSI_DEGRADING_TREND_DB_PER_S)
    {
        return DmFaultCause::RadioRangeLoss;
    }

    if (remote.hasLastValidDistance &&
        remote.lastValidDistanceRatio < DM_COM_SATURATION_MAX_RATIO &&
        rssiAvailable &&
        fusedRssi >= DM_RSSI_GOOD_DBM &&
        (!tbAvailable || fusedTrend >= DM_RSSI_STABLE_TREND_DB_PER_S))
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

uint32_t DistanceMonitorModule::linkTimeoutMs(uint8_t intervalSec) const
{
    const uint32_t effectiveIntervalSec =
        intervalSec >= DM_MIN_REPORT_INTERVAL_S
            ? intervalSec
            : dmComputeMaxReportIntervalSec(
                  runtimeConfig_.maxDistanceMeters);

    return 2U * effectiveIntervalSec * 1000U +
           DM_LINK_TIMEOUT_MARGIN_MS;
}

uint32_t DistanceMonitorModule::rssiBeaconMaxAgeMs() const
{
    return DM_BASE_BEACON_INTERVAL_MS *
           DM_RSSI_VALID_BEACON_MULTIPLIER;
}

void DistanceMonitorModule::updateBaseAudio(
    size_t localIndex,
    uint32_t nowMs)
{
    // Silent mode suppresses alarm audio only. Pairing, command confirmation
    // and tracker notification sounds are emitted at their call sites.
    if (DM_ALARM_AUDIO_SILENT)
    {
        audio_.stopSos();
        return;
    }

    bool hasSos = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
    {
        if (index == localIndex)
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (!isFaultAudible(remote, nowMs))
            continue;

        if (!remote.hasFaultAudioTime ||
            dmElapsedMs(nowMs, remote.lastFaultAudioMs) >=
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
            continue;

        DmNodeState &remote = nodeStates_[index];
        if (remote.activeDistanceAlertRatio < DM_DISTANCE_ALERT_START_RATIO)
            continue;
        if (deadlinePending(nowMs, remote.distanceSnoozedUntilMs))
            continue;

        const uint32_t periodMs =
            dmDistanceBipIntervalMs(remote.activeDistanceAlertRatio);
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

bool DistanceMonitorModule::isLocalBase() const
{
    size_t index = 0U;
    return findLocalIndex(index) && nodeStates_[index].isBase;
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

    if (localFallDetected_)
    {
        localFallDetected_ = false;
        hadSos = true;
    }

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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
        LOG_INFO("{DM@Alarm} SOS acknowledged");
        return true;
    }

    bool hadFault = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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
        LOG_INFO("{DM@Alarm} fault snoozed");
        return true;
    }

    bool hadDistanceAlert = false;
    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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
            remote.distanceSnoozedUntilMs =
                nowMs + DM_DISTANCE_SNOOZE_MS;
        }
    }

    if (hadDistanceAlert)
        LOG_INFO("{DM@Alarm} Distance acknowledged");

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

    for (size_t index = 0U;
         index < runtimeConfig_.memberCount;
         ++index)
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

void DistanceMonitorModule::handleShutdownThresholdReached()
{
    initializeIfNeeded();
    audio_.playShutdownReadyBip();
    LOG_INFO("{DM@Button} shutdown threshold reached; release to power off");
}

uint32_t DistanceMonitorModule::prepareLocalShutdown()
{
    initializeIfNeeded();

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex))
        return 0U;

    if (nodeStates_[localIndex].isBase)
        return 0U;

    size_t baseIndex = 0U;
    if (!findBaseIndex(baseIndex))
        return 0U;

    sendShutdownNotice(runtimeConfig_.members[baseIndex].nodeNum);
    return DM_SHUTDOWN_TX_GRACE_MS;
}

void DistanceMonitorModule::logSummary(
    size_t localIndex,
    uint32_t nowMs) const
{
    const DmNodeState &local = nodeStates_[localIndex];
    char localBattery[8] = {};
    dmFormatBattery(local.batteryPercent, localBattery, sizeof(localBattery));

    const bool localHasKnownPosition =
        local.latitudeI != 0 && local.longitudeI != 0;
    const double localLat = static_cast<double>(local.latitudeI) * 1.0e-7;
    const double localLon = static_cast<double>(local.longitudeI) * 1.0e-7;

    if (local.positionKind == DmPositionKind::FreshFix)
    {
        LOG_INFO(
            "{DM@%s} schema=%u id=!%08lx pos=FIX lat=%.7f lon=%.7f pos_age_s=%lu dop=%u acc=%.0fm bat=%s motion=%s",
            local.isBase ? "Node_base" : "Node_track",
            static_cast<unsigned>(DM_LOG_SCHEMA),
            static_cast<unsigned long>(local.nodeNum),
            localLat,
            localLon,
            static_cast<unsigned long>(localFixAgeSeconds(nowMs)),
            static_cast<unsigned>(local.positionDop),
            local.positionAccuracyMeters,
            localBattery,
            localMotionName(localMoving_, localFallDetected_, highSpeedSosLatched_));
    }
    else if (localHasKnownPosition)
    {
        LOG_INFO(
            "{DM@%s} schema=%u id=!%08lx pos=NO_FIX last_lat=%.7f last_lon=%.7f last_pos_age_s=%lu bat=%s motion=%s",
            local.isBase ? "Node_base" : "Node_track",
            static_cast<unsigned>(DM_LOG_SCHEMA),
            static_cast<unsigned long>(local.nodeNum),
            localLat,
            localLon,
            static_cast<unsigned long>(dmElapsedMs(nowMs, localFixMs_) / 1000U),
            localBattery,
            localMotionName(localMoving_, localFallDetected_, highSpeedSosLatched_));
    }
    else
    {
        LOG_INFO(
            "{DM@%s} schema=%u id=!%08lx pos=NO_FIX bat=%s motion=%s",
            local.isBase ? "Node_base" : "Node_track",
            static_cast<unsigned>(DM_LOG_SCHEMA),
            static_cast<unsigned long>(local.nodeNum),
            localBattery,
            localMotionName(localMoving_, localFallDetected_, highSpeedSosLatched_));
    }

    if (!local.isBase)
        return;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex || nodeStates_[index].isBase)
            continue;

        const DmNodeState &remote = nodeStates_[index];
        if (!remote.paired)
        {
            LOG_INFO(
                "{DM@Node_track} schema=%u id=!%08lx paired=0 link=UNPAIRED pos=NO_FIX dist=Unknown",
                static_cast<unsigned>(DM_LOG_SCHEMA),
                static_cast<unsigned long>(remote.nodeNum));
            continue;
        }

        char battery[8] = {};
        dmFormatBattery(remote.batteryPercent, battery, sizeof(battery));
        const uint32_t reportAgeS = remote.hasPositionReportRxTime
            ? dmElapsedMs(nowMs, remote.lastPositionReportRxMs) / 1000U
            : 0U;
        const uint32_t lastFixAgeS = remote.hasLastFixRxTime
            ? dmElapsedMs(nowMs, remote.lastFixRxMs) / 1000U
            : 0U;
        const bool remoteHasKnownPosition =
            remote.latitudeI != 0 && remote.longitudeI != 0;
        const double remoteLat = static_cast<double>(remote.latitudeI) * 1.0e-7;
        const double remoteLon = static_cast<double>(remote.longitudeI) * 1.0e-7;

        if (remote.distanceSource == DmDistanceSource::Gps)
        {
            const unsigned pct = static_cast<unsigned>(
                std::max(0.0F, remote.distanceRatio) * 100.0F + 0.5F);

            LOG_INFO(
                "{DM@Node_track} schema=%u id=!%08lx paired=1 link=ALIVE rpt_age_s=%lu pos=FIX lat=%.7f lon=%.7f pos_age_s=%lu dop=%u acc=%.0fm bat=%s motion=%s src=GPS dist_m=%.0f raw_m=%.0f total_acc_m=%.0f ratio=%u%% alarm=%u sos=%s fault=%s",
                static_cast<unsigned>(DM_LOG_SCHEMA),
                static_cast<unsigned long>(remote.nodeNum),
                static_cast<unsigned long>(reportAgeS),
                remoteLat,
                remoteLon,
                static_cast<unsigned long>(lastFixAgeS),
                static_cast<unsigned>(remote.positionDop),
                remote.positionAccuracyMeters,
                battery,
                remoteMotionName(remote),
                remote.distanceMeters,
                remote.rawDistanceMeters,
                remote.combinedAccuracyMeters,
                pct,
                remote.activeDistanceAlertRatio >= DM_DISTANCE_ALERT_START_RATIO ? 1U : 0U,
                remote.sosActive ? dmSosCauseName(remote.activeSosCause) : "NONE",
                dmFaultCauseName(remote.faultCause));
        }
        else if (remote.distanceSource == DmDistanceSource::Rssi)
        {
            if (remoteHasKnownPosition)
            {
                LOG_INFO(
                    "{DM@Node_track} schema=%u id=!%08lx paired=1 link=ALIVE rpt_age_s=%lu pos=%s last_lat=%.7f last_lon=%.7f last_pos_age_s=%lu bat=%s motion=%s src=RSSI band=%s rssi=%.0f alarm=%u sos=%s fault=%s",
                    static_cast<unsigned>(DM_LOG_SCHEMA),
                    static_cast<unsigned long>(remote.nodeNum),
                    static_cast<unsigned long>(reportAgeS),
                    dmPositionKindName(remote.positionKind),
                    remoteLat,
                    remoteLon,
                    static_cast<unsigned long>(lastFixAgeS),
                    battery,
                    remoteMotionName(remote),
                    dmDistanceBandName(remote.rssiEstimate.band),
                    static_cast<double>(remote.rssiEstimate.bestRssiDbm),
                    remote.activeDistanceAlertRatio >= DM_DISTANCE_ALERT_START_RATIO ? 1U : 0U,
                    remote.sosActive ? dmSosCauseName(remote.activeSosCause) : "NONE",
                    dmFaultCauseName(remote.faultCause));
            }
            else
            {
                LOG_INFO(
                    "{DM@Node_track} schema=%u id=!%08lx paired=1 link=ALIVE rpt_age_s=%lu pos=%s bat=%s motion=%s src=RSSI band=%s rssi=%.0f alarm=%u sos=%s fault=%s",
                    static_cast<unsigned>(DM_LOG_SCHEMA),
                    static_cast<unsigned long>(remote.nodeNum),
                    static_cast<unsigned long>(reportAgeS),
                    dmPositionKindName(remote.positionKind),
                    battery,
                    remoteMotionName(remote),
                    dmDistanceBandName(remote.rssiEstimate.band),
                    static_cast<double>(remote.rssiEstimate.bestRssiDbm),
                    remote.activeDistanceAlertRatio >= DM_DISTANCE_ALERT_START_RATIO ? 1U : 0U,
                    remote.sosActive ? dmSosCauseName(remote.activeSosCause) : "NONE",
                    dmFaultCauseName(remote.faultCause));
            }
        }
        else
        {
            LOG_INFO(
                "{DM@Node_track} schema=%u id=!%08lx paired=1 link=%s rpt_age_s=%lu pos=%s bat=%s motion=%s src=NONE band=Unknown alarm=%u sos=%s fault=%s",
                static_cast<unsigned>(DM_LOG_SCHEMA),
                static_cast<unsigned long>(remote.nodeNum),
                remote.radioState == DmRadioState::Alive ? "ALIVE" : "UNKNOWN",
                static_cast<unsigned long>(reportAgeS),
                dmPositionKindName(remote.positionKind),
                battery,
                remoteMotionName(remote),
                remote.activeDistanceAlertRatio >= DM_DISTANCE_ALERT_START_RATIO ? 1U : 0U,
                remote.sosActive ? dmSosCauseName(remote.activeSosCause) : "NONE",
                dmFaultCauseName(remote.faultCause));
        }
    }
}
