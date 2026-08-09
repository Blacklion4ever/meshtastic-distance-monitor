#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "NodeDB.h"
#include "RTC.h"

#include <Arduino.h>
#include <cstdio>

DistanceMonitorModule::DistanceMonitorModule()
    : SinglePortModule("distance_monitor", meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("DistanceMonitor")
{
    loadDefaultConfiguration();
}

void DistanceMonitorModule::setup()
{
    LOG_INFO(
        "Distance Monitor initialized: firmware=%s protocol=%u members=%u tick=%lums poll=%lus",
        DM_FIRMWARE_VERSION,
        static_cast<unsigned>(DM_PROTOCOL_VERSION),
        static_cast<unsigned>(runtimeConfig_.memberCount),
        static_cast<unsigned long>(DM_TICK_INTERVAL_MS),
        static_cast<unsigned long>(DM_POSITION_REQUEST_INTERVAL_MS / 1000U));

    setIntervalFromNow(DM_TICK_INTERVAL_MS);
}

void DistanceMonitorModule::acknowledgeLocalAlarms()
{
    if (nodeDB == nullptr)
    {
        return;
    }

    size_t localIndex = 0U;
    if (!findMemberIndex(nodeDB->getNodeNum(), localIndex) || !nodeStates_[localIndex].isBase)
    {
        return;
    }

    const uint32_t nowMs = millis();
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        if (index == localIndex)
        {
            continue;
        }

        DmNodeState &state = nodeStates_[index];
        if (state.alarmState != DmAlarmState::Safe)
        {
            state.alarmAcknowledged = true;
            state.alarmAcknowledgedMs = nowMs;
        }
    }
}

bool DistanceMonitorModule::sendLocalSos()
{
    if (nodeDB == nullptr)
    {
        return false;
    }

    size_t localIndex = 0U;
    const uint32_t localNodeNum = nodeDB->getNodeNum();
    if (!findMemberIndex(localNodeNum, localIndex))
    {
        return false;
    }

    return sendSosToMembers(localNodeNum);
}

int32_t DistanceMonitorModule::runOnce()
{
    if (nodeDB == nullptr || runtimeConfig_.memberCount == 0U)
    {
        return DM_TICK_INTERVAL_MS;
    }

    const uint32_t localNodeNum = nodeDB->getNodeNum();
    size_t localIndex = 0U;
    if (localNodeNum == 0U || !findMemberIndex(localNodeNum, localIndex))
    {
        LOG_WARN("Distance Monitor: local node !%08lx is not configured",
                 static_cast<unsigned long>(localNodeNum));
        return DM_TICK_INTERVAL_MS;
    }

    const uint32_t nowMs = millis();
    const uint32_t now = getTime();
    const bool absoluteTimeValid = now >= DM_MIN_VALID_UNIX_TIME_S;

    DmNodeState &localState = nodeStates_[localIndex];
    localState.isLocal = true;
    localState.isBase = runtimeConfig_.members[localIndex].isBase;

    if (!localPositionStarted_)
    {
        startLocalPositionManager(nowMs);
    }
    updateLocalPosition(localState, nowMs);

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        refreshMemberState(index, localNodeNum, now, absoluteTimeValid, nowMs);
    }

    if (localState.isBase)
    {
        logLocalPositionSample(localState, nowMs);
    }

    for (size_t remoteIndex = 0U; remoteIndex < runtimeConfig_.memberCount; ++remoteIndex)
    {
        if (remoteIndex != localIndex)
        {
            processRemoteMember(localIndex, remoteIndex, nowMs);
        }
    }

    if (!hasSummaryLogTime_ || dmElapsedMs(nowMs, lastSummaryLogMs_) >= DM_SUMMARY_INTERVAL_MS)
    {
        lastSummaryLogMs_ = nowMs;
        hasSummaryLogTime_ = true;
        logMemberSummary(nowMs);
    }

    return DM_TICK_INTERVAL_MS;
}

void DistanceMonitorModule::loadDefaultConfiguration()
{
    runtimeConfig_ = DmRuntimeConfig{};
    runtimeConfig_.maxDistanceMeters = DM_MAX_DISTANCE_M;
    runtimeConfig_.warningRatio = DM_WARNING_RATIO;
    runtimeConfig_.freshPositionMaxAgeSeconds = DM_FRESH_POSITION_MAX_AGE_S;
    runtimeConfig_.cachedPositionMaxAgeSeconds = DM_CACHED_POSITION_MAX_AGE_S;
    runtimeConfig_.maxDop = DM_MAX_DOP;
    runtimeConfig_.memberCount = DM_DEFAULT_MEMBER_COUNT;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        runtimeConfig_.members[index].nodeNum = DM_DEFAULT_MEMBERS[index].nodeNum;
        runtimeConfig_.members[index].isBase = DM_DEFAULT_MEMBERS[index].isBase;
        nodeStates_[index].nodeNum = DM_DEFAULT_MEMBERS[index].nodeNum;
        nodeStates_[index].isBase = DM_DEFAULT_MEMBERS[index].isBase;
    }
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

void DistanceMonitorModule::refreshMemberState(
    size_t index,
    uint32_t localNodeNum,
    uint32_t now,
    bool absoluteTimeValid,
    uint32_t nowMs)
{
    DmNodeState &state = nodeStates_[index];
    const DmMemberConfig &member = runtimeConfig_.members[index];

    state.nodeNum = member.nodeNum;
    state.isBase = member.isBase;
    state.isLocal = member.nodeNum == localNodeNum;

    evaluateRadioState(
        state,
        nodeDB->getMeshNode(member.nodeNum),
        now,
        absoluteTimeValid,
        nowMs);

    if (state.isLocal)
    {
        state.distanceState = DmDistanceState::Local;
        state.distanceMeters = 0.0;
    }
}

void DistanceMonitorModule::evaluateRadioState(
    DmNodeState &state,
    const meshtastic_NodeInfoLite *node,
    uint32_t now,
    bool absoluteTimeValid,
    uint32_t nowMs) const
{
    if (state.isLocal)
    {
        state.lastHeardAgeSeconds = 0U;
        state.hasLastHeardAge = true;
        state.radioState = DmRadioState::Alive;
        return;
    }

    const uint32_t lastHeard = node != nullptr ? node->last_heard : 0U;
    state.hasLastHeardAge = false;

    if (absoluteTimeValid && lastHeard != 0U && now >= lastHeard)
    {
        state.lastHeardAgeSeconds = now - lastHeard;
        state.hasLastHeardAge = true;
    }
    else if (state.hasPacketRxTime)
    {
        state.lastHeardAgeSeconds = dmElapsedMs(nowMs, state.lastPacketRxMs) / 1000U;
        state.hasLastHeardAge = true;
    }

    if (!state.hasLastHeardAge)
    {
        state.radioState = DmRadioState::Unknown;
    }
    else if (state.lastHeardAgeSeconds <= DM_LAST_HEARD_STALE_S)
    {
        state.radioState = DmRadioState::Alive;
    }
    else if (state.lastHeardAgeSeconds <= DM_LAST_HEARD_LOST_S)
    {
        state.radioState = DmRadioState::Stale;
    }
    else
    {
        state.radioState = DmRadioState::Lost;
    }
}

uint32_t DistanceMonitorModule::positionAgeSeconds(
    const DmNodeState &state,
    uint32_t nowMs) const
{
    if (state.positionKind == DmPositionKind::NoFix)
    {
        return 0U;
    }

    return state.positionAgeAtRxSeconds +
           dmElapsedMs(nowMs, state.positionRxMs) / 1000U;
}

bool DistanceMonitorModule::isPositionUsable(
    const DmNodeState &state,
    uint32_t nowMs) const
{
    if (state.positionKind == DmPositionKind::NoFix)
    {
        return false;
    }

    const uint32_t maxAge =
        state.positionKind == DmPositionKind::CachedStationary
            ? runtimeConfig_.cachedPositionMaxAgeSeconds
            : runtimeConfig_.freshPositionMaxAgeSeconds;

    return positionAgeSeconds(state, nowMs) < maxAge;
}

void DistanceMonitorModule::processRemoteMember(
    size_t localIndex,
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &local = nodeStates_[localIndex];
    DmNodeState &remote = nodeStates_[remoteIndex];

    if (local.isBase)
    {
        updateAlarmAcknowledgement(remote, nowMs);
    }

    if (remote.radioState != DmRadioState::Alive &&
        (!remote.hasAliveRequestTime ||
         dmElapsedMs(nowMs, remote.lastAliveRequestMs) >= DM_ALIVE_REQUEST_COOLDOWN_MS))
    {
        if (requestAlive(remote.nodeNum))
        {
            remote.lastAliveRequestMs = nowMs;
            remote.hasAliveRequestTime = true;
        }
    }

    if (local.isBase &&
        (!remote.hasPositionRequestTime ||
         dmElapsedMs(nowMs, remote.lastPositionRequestMs) >= DM_POSITION_REQUEST_INTERVAL_MS))
    {
        if (requestPosition(remote.nodeNum))
        {
            remote.lastPositionRequestMs = nowMs;
            remote.hasPositionRequestTime = true;
        }
    }

    if (isPositionUsable(local, nowMs) && isPositionUsable(remote, nowMs))
    {
        remote.distanceMeters = dmCalculateDistanceMeters(
            static_cast<double>(local.latitudeI) * 1e-7,
            static_cast<double>(local.longitudeI) * 1e-7,
            static_cast<double>(remote.latitudeI) * 1e-7,
            static_cast<double>(remote.longitudeI) * 1e-7);

        remote.distanceState = DmDistanceState::Valid;
        remote.lastValidDistanceMeters = remote.distanceMeters;
        remote.lastValidDistanceMs = nowMs;
        remote.hasLastValidDistance = true;

        if (local.isBase)
        {
            updateAlarmState(remote);
        }
        return;
    }

    if (remote.hasLastValidDistance && remote.radioState != DmRadioState::Lost)
    {
        const bool wasClose =
            remote.lastValidDistanceMeters < runtimeConfig_.maxDistanceMeters * DM_CLOSE_DISTANCE_RATIO;
        const uint32_t graceMs = wasClose ? DM_CLOSE_DISTANCE_GRACE_MS : DM_DISTANCE_GRACE_MS;

        if (dmElapsedMs(nowMs, remote.lastValidDistanceMs) <= graceMs)
        {
            remote.distanceMeters = remote.lastValidDistanceMeters;
            remote.distanceState = DmDistanceState::Grace;
            return;
        }
    }

    remote.distanceState = DmDistanceState::Unknown;
    if (local.isBase)
    {
        remote.alarmState = DmAlarmState::TrackingLost;
    }
}

void DistanceMonitorModule::updateAlarmState(DmNodeState &state)
{
    const double warning = runtimeConfig_.maxDistanceMeters * runtimeConfig_.warningRatio;
    const double alarm = runtimeConfig_.maxDistanceMeters;

    if (state.distanceMeters < warning)
    {
        state.alarmState = DmAlarmState::Safe;
        state.alarmAcknowledged = false;
    }
    else if (state.distanceMeters > alarm)
    {
        state.alarmState = DmAlarmState::Alarm;
        state.alarmAcknowledged = false;
    }
    else if (state.alarmState != DmAlarmState::Alarm)
    {
        state.alarmState = DmAlarmState::Warning;
        state.alarmAcknowledged = false;
    }
}

void DistanceMonitorModule::updateAlarmAcknowledgement(
    DmNodeState &state,
    uint32_t nowMs) const
{
    if (!state.alarmAcknowledged)
    {
        return;
    }

    if (state.alarmState == DmAlarmState::Safe ||
        dmElapsedMs(nowMs, state.alarmAcknowledgedMs) >= DM_ALARM_REARM_MS)
    {
        state.alarmAcknowledged = false;
    }
}

void DistanceMonitorModule::logMemberSummary(uint32_t nowMs) const
{
    LOG_INFO("[");

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        const DmNodeState &state = nodeStates_[index];

        char radioAge[16];
        char positionAge[16];
        char positionStatus[24];
        char distance[24];
        char rssi[40];
        char alarm[28];

        if (state.isLocal)
        {
            std::snprintf(radioAge, sizeof(radioAge), "self");
        }
        else if (state.hasLastHeardAge)
        {
            dmFormatAge(state.lastHeardAgeSeconds, radioAge, sizeof(radioAge));
        }
        else
        {
            std::snprintf(radioAge, sizeof(radioAge), "n/a");
        }

        if (state.positionKind != DmPositionKind::NoFix)
        {
            dmFormatAge(positionAgeSeconds(state, nowMs), positionAge, sizeof(positionAge));
            if (isPositionUsable(state, nowMs))
            {
                std::snprintf(positionStatus, sizeof(positionStatus), "%s", dmPositionKindName(state.positionKind));
            }
            else
            {
                std::snprintf(positionStatus, sizeof(positionStatus), "STALE/%s", dmPositionKindName(state.positionKind));
            }
        }
        else
        {
            std::snprintf(positionAge, sizeof(positionAge), "n/a");
            std::snprintf(positionStatus, sizeof(positionStatus), "NO_FIX");
        }

        switch (state.distanceState)
        {
        case DmDistanceState::Local:
            std::snprintf(distance, sizeof(distance), "self");
            break;
        case DmDistanceState::Valid:
        case DmDistanceState::Grace:
            std::snprintf(distance, sizeof(distance), "%.1fm", state.distanceMeters);
            break;
        default:
            std::snprintf(distance, sizeof(distance), "n/a");
            break;
        }

        if (state.radioMetricsValid)
        {
            std::snprintf(
                rssi,
                sizeof(rssi),
                "%.1fdBm (%+.1f)",
                static_cast<double>(state.filteredRssi),
                static_cast<double>(state.rssiTrend));
        }
        else
        {
            std::snprintf(rssi, sizeof(rssi), "n/a");
        }

        if (state.isLocal)
        {
            std::snprintf(alarm, sizeof(alarm), "LOCAL");
        }
        else if (state.alarmAcknowledged)
        {
            std::snprintf(alarm, sizeof(alarm), "%s/ACK", dmAlarmStateName(state.alarmState));
        }
        else
        {
            std::snprintf(alarm, sizeof(alarm), "%s", dmAlarmStateName(state.alarmState));
        }

        LOG_INFO(
            "%s Node ID: !%08lx | radio: %s {%s} | gnss: %s {%s} | distance: %s {%s/%s} | RSSI: %s",
            state.isBase ? "(*)" : "   ",
            static_cast<unsigned long>(state.nodeNum),
            radioAge,
            dmRadioStateName(state.radioState),
            positionAge,
            positionStatus,
            distance,
            dmDistanceStateName(state.distanceState),
            alarm,
            rssi);
    }

    LOG_INFO("]");
}
