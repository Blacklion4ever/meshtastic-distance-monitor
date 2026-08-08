#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "configuration.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include <pb_decode.h>

DistanceMonitorModule::DistanceMonitorModule()
    : SinglePortModule(
          "distance_monitor",
          meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("DistanceMonitor")
{
    /*
     * Initialize the complete runtime member configuration immediately.
     *
     * The OSThread already exists after the base-class constructor runs, so
     * runOnce() must never depend on MeshModule::setup() having initialized
     * the member list first.
     */
    loadDefaultConfiguration();
}

void DistanceMonitorModule::setup()
{
    LOG_INFO(
        "Distance Monitor initialized: firmware=%s protocol=%u members=%u tick=%lums",
        DM_FIRMWARE_VERSION,
        static_cast<unsigned>(DM_PROTOCOL_VERSION),
        static_cast<unsigned>(runtimeConfig_.memberCount),
        static_cast<unsigned long>(DM_TICK_INTERVAL_MS));

    /*
     * Print the configured member list once at startup.
     * This is deliberately not repeated every monitoring cycle.
     */
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        LOG_DEBUG(
            "Distance Monitor member[%u]: !%08lx base=%s",
            static_cast<unsigned>(index),
            static_cast<unsigned long>(
                runtimeConfig_.members[index].nodeNum),
            runtimeConfig_.members[index].isBase ? "yes" : "no");
    }

    /*
     * Start the periodic monitoring loop after configuration has been loaded.
     */
    setIntervalFromNow(DM_TICK_INTERVAL_MS);
}

void DistanceMonitorModule::acknowledgeLocalAlarms()
{
    // This action is deliberately local. Trackers do not need to know that a
    // human acknowledged the buzzer on the base.
    if (nodeDB == nullptr)
    {
        return;
    }

    const uint32_t localNodeNum = nodeDB->getNodeNum();
    size_t localIndex = 0U;

    if (localNodeNum == 0U || !findMemberIndex(localNodeNum, localIndex))
    {
        return;
    }

    if (!nodeStates_[localIndex].isBase)
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

        DmNodeState &remoteState = nodeStates_[index];
        if (remoteState.alarmState == DmAlarmState::Warning ||
            remoteState.alarmState == DmAlarmState::Alarm ||
            remoteState.alarmState == DmAlarmState::TrackingLost)
        {
            remoteState.alarmAcknowledged = true;
            remoteState.alarmAcknowledgedMs = nowMs;
        }
    }
}

bool DistanceMonitorModule::sendLocalSos()
{
    if (nodeDB == nullptr)
    {
        return false;
    }

    const uint32_t localNodeNum = nodeDB->getNodeNum();
    size_t localIndex = 0U;

    if (localNodeNum == 0U || !findMemberIndex(localNodeNum, localIndex))
    {
        return false;
    }

    return sendSosToMembers(localNodeNum);
}

int32_t DistanceMonitorModule::runOnce()
{
    // 1. NodeDB is mandatory for local identity and remote membership data.
    if (nodeDB == nullptr)
    {
        LOG_WARN("Distance Monitor: NodeDB unavailable");
        return DM_TICK_INTERVAL_MS;
    }

    // Runtime configuration must always contain at least the local group definition.
    if (runtimeConfig_.memberCount == 0U)
    {
        LOG_WARN("Distance Monitor: runtime configuration is empty");
        return DM_TICK_INTERVAL_MS;
    }
    // 2. Identify this device and verify that it belongs to the configured group.
    const uint32_t localNodeNum = nodeDB->getNodeNum();
    size_t localIndex = 0U;

    if (localNodeNum == 0U || !findMemberIndex(localNodeNum, localIndex))
    {
        LOG_WARN(
            "Distance Monitor: local node !%08lx is not a configured member",
            static_cast<unsigned long>(localNodeNum));
        return DM_TICK_INTERVAL_MS;
    }

    // 3. Absolute time is useful but not mandatory. Monitoring, ALIVE_REQUEST
    //    and POSITION_APP requests continue with monotonic time when RTC is absent.
    const uint32_t now = getTime();
    const bool absoluteTimeValid = now != 0U;
    const uint32_t nowMs = millis();

    // 4. Refresh one generic state object for every configured member.
    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        refreshMemberState(index, localNodeNum, now, absoluteTimeValid, nowMs);
    }

    // 5. Process every member except ourselves. All nodes maintain liveness;
    //    only a configured base aggressively maintains fresh tracker positions.
    for (size_t remoteIndex = 0U; remoteIndex < runtimeConfig_.memberCount; ++remoteIndex)
    {
        if (remoteIndex == localIndex)
        {
            continue;
        }

        processRemoteMember(localIndex, remoteIndex, nowMs);
    }

    // 6. Print one compact monitoring block instead of many unrelated INFO logs.
    logMemberSummary();

    return DM_TICK_INTERVAL_MS;
}

ProcessMessage DistanceMonitorModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Ignore senders that are not members of this Distance Monitor group.
    size_t memberIndex = 0U;
    if (mp.from == 0U || !findMemberIndex(mp.from, memberIndex))
    {
        return ProcessMessage::CONTINUE;
    }

    DmNodeState &state = nodeStates_[memberIndex];
    const uint32_t nowMs = millis();

    // Any packet from the member refreshes runtime liveness. RSSI/SNR are only
    // retained when transport/hop metadata proves that the LoRa hop was direct.
    updatePacketReceptionState(state, mp, nowMs);

    // Observe and decode full POSITION_APP payloads. Native Meshtastic still
    // receives the packet because this module always returns CONTINUE.
    if (mp.decoded.portnum == meshtastic_PortNum_POSITION_APP)
    {
        if (mp.decoded.payload.size > 0U)
        {
            decodeAndStorePosition(state, mp, nowMs);
        }
        return ProcessMessage::CONTINUE;
    }

    // Only Distance Monitor PRIVATE_APP messages are parsed below this point.
    if (mp.decoded.portnum != meshtastic_PortNum_PRIVATE_APP)
    {
        return ProcessMessage::CONTINUE;
    }

    DmMessageHeader header;
    if (!decodeMessageHeader(mp.decoded.payload.bytes, mp.decoded.payload.size, header))
    {
        return ProcessMessage::CONTINUE;
    }

    // The event identity is (sender NodeNum, sequence). Exact duplicate events
    // are ignored; a new sequence is accepted even after a sender reboot.
    if (!acceptSequence(state, header.sequence, nowMs))
    {
        return ProcessMessage::CONTINUE;
    }

    switch (header.type)
    {
    case DmMessageType::AliveRequest:
        sendAliveResponse(mp.from);
        break;

    case DmMessageType::AliveResponse:
        // Packet reception metadata already refreshed liveness above.
        break;

    case DmMessageType::Sos:
        // Physical buzzer integration will consume this flag later and emit
        // three consecutive beeps. Protocol reception itself stays hardware-free.
        state.sosPending = true;
        LOG_WARN(
            "Distance Monitor SOS received: node=!%08lx seq=%lu",
            static_cast<unsigned long>(mp.from),
            static_cast<unsigned long>(header.sequence));
        break;

    default:
        break;
    }

    return ProcessMessage::CONTINUE;
}

bool DistanceMonitorModule::wantPacket(const meshtastic_MeshPacket *packet)
{
    if (packet == nullptr)
    {
        return false;
    }

    return packet->decoded.portnum == meshtastic_PortNum_PRIVATE_APP ||
           packet->decoded.portnum == meshtastic_PortNum_POSITION_APP;
}

void DistanceMonitorModule::loadDefaultConfiguration()
{
    runtimeConfig_ = DmRuntimeConfig{};

    runtimeConfig_.maxDistanceMeters = DM_MAX_DISTANCE_M;
    runtimeConfig_.warningRatio = DM_WARNING_RATIO;
    runtimeConfig_.maxHdop = DM_MAX_HDOP;
    runtimeConfig_.positionMaxAgeSeconds = DM_POSITION_MAX_AGE_S;
    runtimeConfig_.memberCount = DM_DEFAULT_MEMBER_COUNT;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        runtimeConfig_.members[index].nodeNum = DM_DEFAULT_MEMBERS[index].nodeNum;
        runtimeConfig_.members[index].isBase = DM_DEFAULT_MEMBERS[index].isBase;

        nodeStates_[index] = DmNodeState{};
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

    // Membership attributes are refreshed from mutable runtime configuration so
    // a later app-driven configuration reload does not require state code changes.
    state.nodeNum = member.nodeNum;
    state.isBase = member.isBase;
    state.isLocal = member.nodeNum == localNodeNum;

    meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(member.nodeNum);
    state.presentInNodeDB = node != nullptr;

    // The local device owns a full meshtastic_Position object. Copy it directly
    // each tick because NodeInfoLite only stores PositionLite fields.
    if (state.isLocal)
    {
        storeFullPositionSample(state, localPosition, nowMs, false);
    }

    evaluateRadioState(state, node, now, absoluteTimeValid, nowMs);
    evaluatePositionState(state, node, now, absoluteTimeValid, nowMs);

    if (state.isLocal)
    {
        // Self-distance is a display concept only. A bad local GNSS position still
        // prevents distance calculations to remote members through isPositionUsable().
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
    state.lastHeard = node != nullptr ? node->last_heard : 0U;
    state.hasLastHeardAge = false;

    if (state.isLocal)
    {
        state.lastHeardAgeSeconds = 0U;
        state.hasLastHeardAge = true;
        state.radioState = DmRadioState::Alive;
        return;
    }

    // Preferred source: Meshtastic absolute last_heard and a coherent RTC.
    if (absoluteTimeValid && state.lastHeard != 0U && now >= state.lastHeard)
    {
        state.lastHeardAgeSeconds = now - state.lastHeard;
        state.hasLastHeardAge = true;
    }
    else if (state.hasPacketRxTime)
    {
        // Degraded source: this boot's monotonic packet reception time. This keeps
        // liveness useful even when the absolute clock has not been established.
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

void DistanceMonitorModule::evaluatePositionState(
    DmNodeState &state,
    const meshtastic_NodeInfoLite *node,
    uint32_t now,
    bool absoluteTimeValid,
    uint32_t nowMs) const
{
    state.hasPositionAge = false;
    state.positionAgeSource = DmPositionAgeSource::None;

    // A persisted NodeDB PositionLite is useful as a coordinate fallback after
    // reboot, but it cannot satisfy quality checks because it has no HDOP/fix type.
    if (!state.hasFullPositionSample)
    {
        if (node == nullptr || !nodeDB->hasValidPosition(node))
        {
            state.hasCoordinates = false;
            state.positionState = DmPositionState::Missing;
            return;
        }

        state.hasCoordinates = true;
        state.latitudeI = node->position.latitude_i;
        state.longitudeI = node->position.longitude_i;
        state.positionState = DmPositionState::Unknown;
        return;
    }

    if (!state.hasCoordinates)
    {
        state.positionState = DmPositionState::Missing;
        return;
    }

    if (state.fixType < DM_MIN_FIX_TYPE)
    {
        state.positionState = DmPositionState::NoFix;
        return;
    }

    if (state.hdop == 0U || state.hdop > runtimeConfig_.maxHdop)
    {
        state.positionState = DmPositionState::PoorHdop;
        return;
    }

    // First choice: actual GNSS solution timestamp. This is the age we really
    // want for the monitoring decision when the remote firmware supplies it.
    if (absoluteTimeValid && state.positionTimestamp != 0U)
    {
        if (now < state.positionTimestamp)
        {
            state.positionState = DmPositionState::InvalidTimestamp;
            return;
        }

        state.positionAgeSeconds = now - state.positionTimestamp;
        state.hasPositionAge = true;
        state.positionAgeSource = DmPositionAgeSource::GnssTimestamp;
    }
    // Second choice: Position.time. Meshtastic may populate this even when the
    // dedicated GNSS solution timestamp is absent.
    else if (absoluteTimeValid && state.positionTime != 0U)
    {
        if (now < state.positionTime)
        {
            state.positionState = DmPositionState::InvalidTimestamp;
            return;
        }

        state.positionAgeSeconds = now - state.positionTime;
        state.hasPositionAge = true;
        state.positionAgeSource = DmPositionAgeSource::PositionTime;
    }
    // Final remote fallback: time since the full POSITION_APP was actually seen
    // by this firmware. It proves sample reception freshness, not GNSS solution age.
    else if (!state.isLocal && state.hasPositionRxTime)
    {
        state.positionAgeSeconds = dmElapsedMs(nowMs, state.lastPositionRxMs) / 1000U;
        state.hasPositionAge = true;
        state.positionAgeSource = DmPositionAgeSource::PacketReception;
    }
    else
    {
        state.positionState = DmPositionState::NoTimestamp;
        return;
    }

    // Use >= so a 15-second threshold triggers at the 15-second tick rather than
    // waiting for the next 5-second module cycle.
    if (state.positionAgeSeconds >= runtimeConfig_.positionMaxAgeSeconds)
    {
        state.positionState = DmPositionState::Stale;
        return;
    }

    state.positionState = DmPositionState::Valid;
}

void DistanceMonitorModule::storeFullPositionSample(
    DmNodeState &state,
    const meshtastic_Position &position,
    uint32_t nowMs,
    bool markAsReceivedPacket) const
{
    // Preserve explicit nanopb coordinate presence. A real 0° latitude or
    // longitude is valid and must not be confused with an absent field.
    state.hasCoordinates = position.has_latitude_i && position.has_longitude_i;

    if (position.has_latitude_i)
    {
        state.latitudeI = position.latitude_i;
    }

    if (position.has_longitude_i)
    {
        state.longitudeI = position.longitude_i;
    }

    state.positionTimestamp = position.timestamp;
    state.positionTime = position.time;
    state.hdop = position.HDOP;
    state.fixType = position.fix_type;
    state.fixQuality = position.fix_quality;
    state.satellitesInView = position.sats_in_view;
    state.hasFullPositionSample = true;

    if (markAsReceivedPacket)
    {
        state.lastPositionRxMs = nowMs;
        state.hasPositionRxTime = true;
    }
}

bool DistanceMonitorModule::decodeAndStorePosition(
    DmNodeState &state,
    const meshtastic_MeshPacket &mp,
    uint32_t nowMs) const
{
    meshtastic_Position position = meshtastic_Position_init_zero;

    if (!pb_decode_from_bytes(
            mp.decoded.payload.bytes,
            mp.decoded.payload.size,
            &meshtastic_Position_msg,
            &position))
    {
        LOG_DEBUG(
            "Distance Monitor: failed to decode POSITION_APP from !%08lx",
            static_cast<unsigned long>(mp.from));
        return false;
    }

    storeFullPositionSample(state, position, nowMs, true);
    return true;
}

bool DistanceMonitorModule::isPositionUsable(const DmNodeState &state) const
{
    return state.positionState == DmPositionState::Valid;
}

void DistanceMonitorModule::processRemoteMember(
    size_t localIndex,
    size_t remoteIndex,
    uint32_t nowMs)
{
    DmNodeState &localState = nodeStates_[localIndex];
    DmNodeState &remoteState = nodeStates_[remoteIndex];

    // Re-arm a locally acknowledged alarm after 60 seconds if the underlying
    // state has not improved. This changes only sound eligibility, not zone state.
    if (localState.isBase)
    {
        updateAlarmAcknowledgement(remoteState, nowMs);
    }

    // All members maintain liveness of all other configured members. Cooldown
    // state is per remote member, so a stale node cannot generate a request storm.
    if (remoteState.radioState != DmRadioState::Alive &&
        retryAllowed(
            remoteState.hasAliveRequestTime,
            remoteState.lastAliveRequestMs,
            DM_ALIVE_REQUEST_COOLDOWN_MS,
            nowMs))
    {
        if (requestAlive(remoteState.nodeNum))
        {
            remoteState.lastAliveRequestMs = nowMs;
            remoteState.hasAliveRequestTime = true;
        }
    }

    // Only a base actively maintains fresh tracker positions. Trackers still
    // observe and log positions they receive through normal Meshtastic traffic.
    if (localState.isBase && !isPositionUsable(remoteState) &&
        retryAllowed(
            remoteState.hasPositionRequestTime,
            remoteState.lastPositionRequestMs,
            DM_POSITION_REQUEST_COOLDOWN_MS,
            nowMs))
    {
        if (requestPosition(remoteState.nodeNum))
        {
            remoteState.lastPositionRequestMs = nowMs;
            remoteState.hasPositionRequestTime = true;
        }
    }

    // A new distance is valid only when both local and remote GNSS samples pass
    // the same freshness and quality checks.
    if (isPositionUsable(localState) && isPositionUsable(remoteState))
    {
        const double localLatitude = static_cast<double>(localState.latitudeI) * 1e-7;
        const double localLongitude = static_cast<double>(localState.longitudeI) * 1e-7;
        const double remoteLatitude = static_cast<double>(remoteState.latitudeI) * 1e-7;
        const double remoteLongitude = static_cast<double>(remoteState.longitudeI) * 1e-7;

        remoteState.distanceMeters = dmCalculateDistanceMeters(
            localLatitude,
            localLongitude,
            remoteLatitude,
            remoteLongitude);

        remoteState.distanceState = DmDistanceState::Valid;
        remoteState.lastValidDistanceMeters = remoteState.distanceMeters;
        remoteState.lastValidDistanceMs = nowMs;
        remoteState.hasLastValidDistance = true;

        if (localState.isBase)
        {
            updateAlarmState(remoteState);
        }
        return;
    }

    // A short GNSS dropout does not immediately become a separation alarm.
    // Retain the previous measured distance for a bounded, explicit GRACE state.
    if (remoteState.hasLastValidDistance && remoteState.radioState != DmRadioState::Lost)
    {
        const float closeLimit = runtimeConfig_.maxDistanceMeters * DM_CLOSE_DISTANCE_RATIO;
        const bool wasClearlyClose = remoteState.lastValidDistanceMeters < closeLimit;
        const uint32_t graceMs = wasClearlyClose ? DM_CLOSE_DISTANCE_GRACE_MS : DM_DISTANCE_GRACE_MS;

        if (dmElapsedMs(nowMs, remoteState.lastValidDistanceMs) <= graceMs)
        {
            remoteState.distanceMeters = remoteState.lastValidDistanceMeters;
            remoteState.distanceState = DmDistanceState::Grace;
            return;
        }
    }

    remoteState.distanceState = DmDistanceState::Unknown;

    // Missing tracking data is not SAFE. Once grace expires, a base records a
    // separate TRACKING_LOST condition instead of fabricating a distance alarm.
    if (localState.isBase)
    {
        remoteState.alarmState = DmAlarmState::TrackingLost;
    }
}

void DistanceMonitorModule::updateAlarmState(DmNodeState &remoteState)
{
    if (remoteState.distanceState != DmDistanceState::Valid)
    {
        return;
    }

    const double warningDistance =
        static_cast<double>(runtimeConfig_.maxDistanceMeters) *
        static_cast<double>(runtimeConfig_.warningRatio);

    const double alarmDistance = static_cast<double>(runtimeConfig_.maxDistanceMeters);

    switch (remoteState.alarmState)
    {
    case DmAlarmState::Safe:
    case DmAlarmState::TrackingLost:
        if (remoteState.distanceMeters > alarmDistance)
        {
            remoteState.alarmState = DmAlarmState::Alarm;
            remoteState.alarmAcknowledged = false;
        }
        else if (remoteState.distanceMeters >= warningDistance)
        {
            remoteState.alarmState = DmAlarmState::Warning;
            remoteState.alarmAcknowledged = false;
        }
        else
        {
            remoteState.alarmState = DmAlarmState::Safe;
            remoteState.alarmAcknowledged = false;
        }
        break;

    case DmAlarmState::Warning:
        if (remoteState.distanceMeters > alarmDistance)
        {
            remoteState.alarmState = DmAlarmState::Alarm;
            remoteState.alarmAcknowledged = false;
        }
        else if (remoteState.distanceMeters < warningDistance)
        {
            // Project rule: warning clears only after returning below 80 percent.
            remoteState.alarmState = DmAlarmState::Safe;
            remoteState.alarmAcknowledged = false;
        }
        break;

    case DmAlarmState::Alarm:
        if (remoteState.distanceMeters < warningDistance)
        {
            // Project rule: alarm also clears only below the 80 percent boundary.
            remoteState.alarmState = DmAlarmState::Safe;
            remoteState.alarmAcknowledged = false;
        }
        break;
    }
}

void DistanceMonitorModule::updateAlarmAcknowledgement(
    DmNodeState &remoteState,
    uint32_t nowMs) const
{
    if (!remoteState.alarmAcknowledged)
    {
        return;
    }

    if (remoteState.alarmState == DmAlarmState::Safe)
    {
        remoteState.alarmAcknowledged = false;
        return;
    }

    if (dmElapsedMs(nowMs, remoteState.alarmAcknowledgedMs) >= DM_ALARM_REARM_MS)
    {
        remoteState.alarmAcknowledged = false;
    }
}

void DistanceMonitorModule::logMemberSummary() const
{
    LOG_INFO("[");

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        const DmNodeState &state = nodeStates_[index];

        char radioAge[16];
        char gnssAge[16];
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

        if (state.hasPositionAge)
        {
            dmFormatAge(state.positionAgeSeconds, gnssAge, sizeof(gnssAge));
        }
        else
        {
            std::snprintf(gnssAge, sizeof(gnssAge), "n/a");
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
        case DmDistanceState::Unknown:
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
            std::snprintf(
                alarm,
                sizeof(alarm),
                "%s/ACK",
                dmAlarmStateName(state.alarmState));
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
            gnssAge,
            dmPositionStateName(state.positionState),
            distance,
            dmDistanceStateName(state.distanceState),
            alarm,
            rssi);
    }

    LOG_INFO("]");
}

bool DistanceMonitorModule::requestAlive(uint32_t target)
{
    return sendPrivateMessage(target, DmMessageType::AliveRequest);
}

bool DistanceMonitorModule::sendAliveResponse(uint32_t target)
{
    return sendPrivateMessage(target, DmMessageType::AliveResponse);
}

bool DistanceMonitorModule::requestPosition(uint32_t target)
{
    if (service == nullptr)
    {
        return false;
    }

    if (target == 0U || target == NODENUM_BROADCAST)
    {
        return false;
    }

    meshtastic_MeshPacket *packet = allocDataPacket();
    if (packet == nullptr)
    {
        return false;
    }

    packet->to = target;
    packet->decoded.portnum = meshtastic_PortNum_POSITION_APP;
    packet->decoded.want_response = true;

    // A zero-length POSITION_APP with want_response asks the native remote
    // PositionModule for a position without pretending to carry one ourselves.
    packet->decoded.payload.size = 0U;

    packet->want_ack = true;
    packet->priority = meshtastic_MeshPacket_Priority_RELIABLE;

    LOG_DEBUG(
        "Distance Monitor position request: target=!%08lx packet_id=0x%08lx",
        static_cast<unsigned long>(target),
        static_cast<unsigned long>(packet->id));

    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    return true;
}

bool DistanceMonitorModule::sendPrivateMessage(uint32_t target, DmMessageType type)
{
    return sendPrivateMessageWithSequence(target, type, allocateSequenceNumber());
}

bool DistanceMonitorModule::sendPrivateMessageWithSequence(
    uint32_t target,
    DmMessageType type,
    uint32_t sequence)
{
    if (service == nullptr)
    {
        return false;
    }

    if (target == 0U || target == NODENUM_BROADCAST)
    {
        return false;
    }

    meshtastic_MeshPacket *packet = allocDataPacket();
    if (packet == nullptr)
    {
        return false;
    }

    size_t encodedSize = 0U;
    if (!encodeMessageHeader(
            type,
            sequence,
            packet->decoded.payload.bytes,
            sizeof(packet->decoded.payload.bytes),
            encodedSize))
    {
        return false;
    }

    packet->to = target;
    packet->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    packet->decoded.payload.size = encodedSize;
    packet->decoded.want_response = false;
    packet->want_ack = true;
    packet->priority = meshtastic_MeshPacket_Priority_RELIABLE;

    LOG_DEBUG(
        "Distance Monitor private TX: target=!%08lx type=%u seq=%lu",
        static_cast<unsigned long>(target),
        static_cast<unsigned>(type),
        static_cast<unsigned long>(sequence));

    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    return true;
}

bool DistanceMonitorModule::sendSosToMembers(uint32_t localNodeNum)
{
    // One logical SOS gets one sender-local sequence, reused for every unicast.
    // Only configured group members are addressed; no mesh-wide broadcast is used.
    const uint32_t sequence = allocateSequenceNumber();
    bool allSent = true;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        const uint32_t target = runtimeConfig_.members[index].nodeNum;
        if (target == localNodeNum)
        {
            continue;
        }

        if (!sendPrivateMessageWithSequence(target, DmMessageType::Sos, sequence))
        {
            allSent = false;
        }
    }

    return allSent;
}

uint32_t DistanceMonitorModule::allocateSequenceNumber()
{
    const uint32_t allocated = nextSequenceNumber_++;

    // Sequence zero is reserved as invalid/uninitialized.
    if (nextSequenceNumber_ == 0U)
    {
        nextSequenceNumber_ = 1U;
    }

    return allocated == 0U ? 1U : allocated;
}

bool DistanceMonitorModule::encodeMessageHeader(
    DmMessageType type,
    uint32_t sequence,
    uint8_t *buffer,
    size_t bufferSize,
    size_t &encodedSize) const
{
    encodedSize = 0U;

    if (buffer == nullptr || bufferSize < DM_PROTOCOL_HEADER_SIZE || sequence == 0U)
    {
        return false;
    }

    // Explicit byte serialization avoids C++ struct padding and endianness bugs.
    // Layout: 'D' 'M' version type sequence[0..3 little-endian].
    buffer[0] = DM_PROTOCOL_MAGIC_0;
    buffer[1] = DM_PROTOCOL_MAGIC_1;
    buffer[2] = DM_PROTOCOL_VERSION;
    buffer[3] = static_cast<uint8_t>(type);
    buffer[4] = static_cast<uint8_t>(sequence & 0xFFU);
    buffer[5] = static_cast<uint8_t>((sequence >> 8U) & 0xFFU);
    buffer[6] = static_cast<uint8_t>((sequence >> 16U) & 0xFFU);
    buffer[7] = static_cast<uint8_t>((sequence >> 24U) & 0xFFU);

    encodedSize = DM_PROTOCOL_HEADER_SIZE;
    return true;
}

bool DistanceMonitorModule::decodeMessageHeader(
    const uint8_t *buffer,
    size_t bufferSize,
    DmMessageHeader &header) const
{
    if (buffer == nullptr || bufferSize < DM_PROTOCOL_HEADER_SIZE)
    {
        return false;
    }

    if (buffer[0] != DM_PROTOCOL_MAGIC_0 || buffer[1] != DM_PROTOCOL_MAGIC_1)
    {
        return false;
    }

    if (buffer[2] != DM_PROTOCOL_VERSION)
    {
        return false;
    }

    const uint8_t rawType = buffer[3];
    if (rawType < static_cast<uint8_t>(DmMessageType::AliveRequest) ||
        rawType > static_cast<uint8_t>(DmMessageType::Sos))
    {
        return false;
    }

    header.version = buffer[2];
    header.type = static_cast<DmMessageType>(rawType);
    header.sequence =
        static_cast<uint32_t>(buffer[4]) |
        (static_cast<uint32_t>(buffer[5]) << 8U) |
        (static_cast<uint32_t>(buffer[6]) << 16U) |
        (static_cast<uint32_t>(buffer[7]) << 24U);

    return header.sequence != 0U;
}

bool DistanceMonitorModule::acceptSequence(
    DmNodeState &state,
    uint32_t sequence,
    uint32_t nowMs)
{
    // lastSeqNum is intentionally a duplicate guard, not a strict cross-reboot
    // monotonicity contract. A rebooted sender may restart its local counter.
    if (state.hasLastSeqNum && sequence == state.lastSeqNum &&
        dmElapsedMs(nowMs, state.lastSeqRxMs) <= DM_SEQUENCE_DUPLICATE_WINDOW_MS)
    {
        return false;
    }

    state.lastSeqNum = sequence;
    state.lastSeqRxMs = nowMs;
    state.hasLastSeqNum = true;
    return true;
}

void DistanceMonitorModule::updatePacketReceptionState(
    DmNodeState &state,
    const meshtastic_MeshPacket &packet,
    uint32_t nowMs)
{
    state.lastPacketRxMs = nowMs;
    state.hasPacketRxTime = true;

    if (!isDirectPacket(packet))
    {
        return;
    }

    state.rssi = static_cast<int16_t>(packet.rx_rssi);
    state.snr = packet.rx_snr;
    state.radioMetricsValid = true;

    if (!state.rssiFilterInitialized)
    {
        state.filteredRssi = static_cast<float>(state.rssi);
        state.rssiTrend = 0.0F;
        state.rssiFilterInitialized = true;
        return;
    }

    const float previousFilteredRssi = state.filteredRssi;
    state.filteredRssi =
        DM_RSSI_FILTER_ALPHA * static_cast<float>(state.rssi) +
        (1.0F - DM_RSSI_FILTER_ALPHA) * previousFilteredRssi;
    state.rssiTrend = state.filteredRssi - previousFilteredRssi;
}

bool DistanceMonitorModule::isDirectPacket(const meshtastic_MeshPacket &packet) const
{
    // RSSI is meaningful for our member only when this packet arrived by LoRa,
    // did not pass through MQTT, and consumed zero mesh hops.
    const bool loraTransport =
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1 ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT2 ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT3;

    if (!loraTransport || packet.via_mqtt)
    {
        return false;
    }

    // Meshtastic defines hop_start - hop_limit as the number of traveled hops.
    // hop_start==0 has no useful provenance for this decision, so fail closed.
    return packet.hop_start > 0U && packet.hop_limit == packet.hop_start;
}

bool DistanceMonitorModule::retryAllowed(
    bool hasPreviousRequest,
    uint32_t previousRequestMs,
    uint32_t cooldownMs,
    uint32_t nowMs) const
{
    return !hasPreviousRequest || dmElapsedMs(nowMs, previousRequestMs) >= cooldownMs;
}
