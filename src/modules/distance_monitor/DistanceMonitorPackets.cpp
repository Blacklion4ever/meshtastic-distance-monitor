#include "DistanceMonitorModule.h"
#include "DistanceMonitorUtils.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/mesh-pb-constants.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
void writeU32Le(uint8_t *buffer, size_t offset, uint32_t value)
{
    buffer[offset] = static_cast<uint8_t>(value & 0xFFU);
    buffer[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    buffer[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    buffer[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

uint32_t readU32Le(const uint8_t *buffer, size_t offset)
{
    return static_cast<uint32_t>(buffer[offset]) |
           (static_cast<uint32_t>(buffer[offset + 1U]) << 8U) |
           (static_cast<uint32_t>(buffer[offset + 2U]) << 16U) |
           (static_cast<uint32_t>(buffer[offset + 3U]) << 24U);
}

int8_t encodeSignedByte(float value)
{
    const long rounded = std::lround(value);
    return static_cast<int8_t>(std::max<long>(-127L, std::min<long>(127L, rounded)));
}

uint8_t encodeUnsignedByte(float value)
{
    const long rounded = std::lround(value);
    return static_cast<uint8_t>(std::max<long>(0L, std::min<long>(255L, rounded)));
}

}

ProcessMessage DistanceMonitorModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    initializeIfNeeded();

    size_t senderIndex = 0U;
    if (mp.from == 0U || !findMemberIndex(mp.from, senderIndex) ||
        mp.decoded.portnum != meshtastic_PortNum_PRIVATE_APP)
    {
        return ProcessMessage::CONTINUE;
    }

    DmMessageHeader header;
    if (!decodeMessageHeader(mp.decoded.payload.bytes, mp.decoded.payload.size, header))
        return ProcessMessage::CONTINUE;

    const uint32_t nowMs = millis();
    DmNodeState &sender = nodeStates_[senderIndex];
    bool duplicate = false;
    if (!acceptSequence(sender, header.sequence, nowMs, duplicate))
        return ProcessMessage::CONTINUE;

    switch (header.type)
    {
    case DmMessageType::AliveRequest:
        handleAliveRequest(sender, mp, header, nowMs);
        break;
    case DmMessageType::AliveResponse:
        handleAliveResponse(sender, mp, header, nowMs);
        break;
    case DmMessageType::PairConfirm:
        handlePairConfirm(sender, mp, header, duplicate, nowMs);
        break;
    case DmMessageType::PositionReport:
        handlePositionReport(senderIndex, sender, mp, header, nowMs);
        break;
    case DmMessageType::SetPositionInterval:
        handleSetPositionInterval(sender, mp, nowMs);
        break;
    case DmMessageType::BaseBeacon:
        handleBaseBeacon(sender, mp, nowMs);
        break;
    case DmMessageType::Sos:
        handleSos(sender, mp, header, nowMs);
        break;
    case DmMessageType::SosAck:
        handleSosAck(mp, nowMs);
        break;
    case DmMessageType::Notification:
        handleNotification(sender, mp, header, duplicate, nowMs);
        break;
    case DmMessageType::NotificationAck:
        handleNotificationAck(sender, mp, nowMs);
        break;
    case DmMessageType::ShutdownNotice:
        handleShutdownNotice(sender, mp, nowMs);
        break;
    }

    return ProcessMessage::CONTINUE;
}

bool DistanceMonitorModule::wantPacket(const meshtastic_MeshPacket *packet)
{
    return packet != nullptr &&
           packet->decoded.portnum == meshtastic_PortNum_PRIVATE_APP;
}

void DistanceMonitorModule::noteRemoteSession(
    DmNodeState &sender,
    uint32_t sessionId,
    uint32_t nowMs)
{
    const bool changed = sender.hasRemoteSession && sender.remoteSessionId != sessionId;
    if (changed)
    {
        const bool previouslyPaired = sender.paired || sender.everPaired;
        const uint32_t oldSession = sender.remoteSessionId;
        sender.paired = false;
        sender.hasRemoteUptime = false;
        sender.pairedLocalSessionId = 0U;
        sender.pairedRemoteSessionId = 0U;
        sender.shutdownNoticeReceived = false;

        if (previouslyPaired)
        {
            sender.faultCause = DmFaultCause::Unpaired;
            sender.hasFaultAudioTime = false;
            sender.faultSnoozedUntilMs = 0U;
            LOG_WARN(
                "{Alarm} Cause=UNPAIRED id=!%08lx session=%08lx->%08lx",
                static_cast<unsigned long>(sender.nodeNum),
                static_cast<unsigned long>(oldSession),
                static_cast<unsigned long>(sessionId));
        }
    }

    sender.hasRemoteSession = true;
    sender.remoteSessionId = sessionId;
}

void DistanceMonitorModule::clearUnpairedFault(DmNodeState &sender)
{
    if (sender.faultCause != DmFaultCause::Unpaired)
        return;

    sender.faultCause = DmFaultCause::None;
    sender.faultSnoozedUntilMs = 0U;
    sender.hasFaultAudioTime = false;
    LOG_INFO("{Pair} id=!%08lx restored", static_cast<unsigned long>(sender.nodeNum));
}

void DistanceMonitorModule::handleAliveRequest(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    const DmMessageHeader &,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_ALIVE_SIZE)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    noteRemoteSession(sender, readU32Le(payload, 8U), nowMs);
    sender.remoteUptimeSeconds = readU32Le(payload, 12U);
    sender.hasRemoteUptime = true;
    sendAliveResponse(mp.from);
}

void DistanceMonitorModule::handleAliveResponse(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    const DmMessageHeader &,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_ALIVE_SIZE)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    noteRemoteSession(sender, readU32Le(payload, 8U), nowMs);
    sender.remoteUptimeSeconds = readU32Le(payload, 12U);
    sender.hasRemoteUptime = true;
    sender.radioState = DmRadioState::Pairing;
    if (!sender.isBase)
        sender.hasHandshakeTxTime = false;
}

void DistanceMonitorModule::handlePairConfirm(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    const DmMessageHeader &,
    bool duplicate,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_PAIR_CONFIRM_SIZE || !sender.isBase)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t baseSessionId = readU32Le(payload, 8U);
    const uint32_t trackerSessionId = readU32Le(payload, 12U);
    const uint8_t flags = payload[16U];
    if (trackerSessionId != bootSessionId_)
        return;

    const bool samePair =
        sender.paired &&
        sender.pairedLocalSessionId == bootSessionId_ &&
        sender.pairedRemoteSessionId == baseSessionId;

    noteRemoteSession(sender, baseSessionId, nowMs);
    sender.paired = true;
    sender.everPaired = true;
    sender.pairedLocalSessionId = bootSessionId_;
    sender.pairedRemoteSessionId = baseSessionId;
    sender.radioState = DmRadioState::Alive;
    clearUnpairedFault(sender);

    if (!duplicate && !samePair &&
        (flags & DM_PAIR_FLAG_PLAY_TRACKER_TONE) != 0U)
    {
        audio_.playPairingBops();
    }

    if (!samePair)
        LOG_INFO("{Pair} id=!%08lx paired", static_cast<unsigned long>(sender.nodeNum));

    forcePositionReport_ = true;
}

void DistanceMonitorModule::handlePositionReport(
    size_t senderIndex,
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    const DmMessageHeader &header,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_POSITION_REPORT_SIZE || sender.isBase)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t trackerSessionId = readU32Le(payload, 8U);
    const uint8_t rawKind = payload[12U];
    const uint8_t flags = payload[13U];
    if (rawKind > static_cast<uint8_t>(DmPositionKind::FreshFix))
        return;

    noteRemoteSession(sender, trackerSessionId, nowMs);
    sender.positionKind = static_cast<DmPositionKind>(rawKind);
    sender.appliedIntervalSec = payload[14U];
    sender.batteryPercent = payload[15U];
    sender.moving = (flags & DM_POSITION_FLAG_MOVING) != 0U;

    if (sender.positionKind == DmPositionKind::NoFix)
    {
        sender.latitudeI = 0;
        sender.longitudeI = 0;
    }
    else
    {
        sender.latitudeI = static_cast<int32_t>(readU32Le(payload, 16U));
        sender.longitudeI = static_cast<int32_t>(readU32Le(payload, 20U));
    }

    sender.remoteBaseRssiValid =
        (flags & DM_POSITION_FLAG_BASE_RSSI_VALID) != 0U;
    if (sender.remoteBaseRssiValid)
    {
        sender.remoteBaseRssiMeanDbm = static_cast<float>(static_cast<int8_t>(payload[24U]));
        sender.remoteBaseRssiStdDb = static_cast<float>(payload[25U]);
        sender.remoteBaseRssiTrendDbPerSec =
            static_cast<float>(static_cast<int8_t>(payload[26U])) / 10.0F;
    }

    sender.lastPositionReportRxMs = nowMs;
    sender.lastPositionReportSequence = header.sequence;
    sender.hasPositionReportRxTime = true;
    sender.radioState = DmRadioState::Alive;

    if (isRadioFault(sender.faultCause) && sender.faultCause != DmFaultCause::Unpaired)
    {
        LOG_INFO(
            "{RX} link restored id=!%08lx previous=%s",
            static_cast<unsigned long>(sender.nodeNum),
            dmFaultCauseName(sender.faultCause));
        sender.faultCause = DmFaultCause::None;
        sender.comSaturationSilentUntilMs = 0U;
        sender.faultSnoozedUntilMs = 0U;
    }

    sender.shutdownNoticeReceived = false;
    if (isDirectPacket(mp))
        directInboundRssi_[senderIndex].update(static_cast<float>(mp.rx_rssi), nowMs);

    const bool pairedReport = (flags & DM_POSITION_FLAG_PAIRED) != 0U;
    if (pairedReport &&
        !sender.paired &&
        sender.pairedLocalSessionId == bootSessionId_ &&
        sender.pairedRemoteSessionId == trackerSessionId)
    {
        sender.paired = true;
        sender.everPaired = true;
        clearUnpairedFault(sender);
        audio_.playPairingBops();
        LOG_INFO("{Pair} id=!%08lx paired", static_cast<unsigned long>(sender.nodeNum));
    }

    char battery[8] = {};
    dmFormatBattery(sender.batteryPercent, battery, sizeof(battery));
    LOG_INFO(
        "{RX} report from id=!%08lx [pos=%s bat=%s state=%s interval=%us]",
        static_cast<unsigned long>(sender.nodeNum),
        dmPositionKindName(sender.positionKind),
        battery,
        sender.moving ? "MOVING" : "STATIONARY",
        static_cast<unsigned>(sender.appliedIntervalSec));
}

void DistanceMonitorModule::handleSetPositionInterval(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    uint32_t)
{
    if (mp.decoded.payload.size < DM_SET_INTERVAL_SIZE || !sender.isBase)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t trackerSessionId = readU32Le(payload, 8U);
    if (trackerSessionId != bootSessionId_)
        return;

    const uint8_t requested = payload[12U];
    const uint8_t maximum = dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);
    localAppliedIntervalSec_ = std::max<uint8_t>(
        DM_MIN_REPORT_INTERVAL_S,
        std::min<uint8_t>(requested, maximum));
    forcePositionReport_ = true;

    LOG_INFO(
        "{RX} setinterval from id=!%08lx (%u s)",
        static_cast<unsigned long>(sender.nodeNum),
        static_cast<unsigned>(localAppliedIntervalSec_));
}

void DistanceMonitorModule::handleBaseBeacon(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_BASE_BEACON_SIZE || !sender.isBase)
        return;

    noteRemoteSession(sender, readU32Le(mp.decoded.payload.bytes, 8U), nowMs);
    if (isDirectPacket(mp))
        baseBeaconRssi_.update(static_cast<float>(mp.rx_rssi), nowMs);
}

void DistanceMonitorModule::handleSos(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    const DmMessageHeader &header,
    uint32_t)
{
    if (mp.decoded.payload.size < DM_SOS_SIZE || sender.isBase)
        return;

    size_t localIndex = 0U;
    if (!findLocalIndex(localIndex) || !nodeStates_[localIndex].isBase)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t trackerSessionId = readU32Le(payload, 8U);
    const uint8_t rawCause = payload[12U];
    if (rawCause < static_cast<uint8_t>(DmSosCause::ManualButton) ||
        rawCause > static_cast<uint8_t>(DmSosCause::HighSpeedMovement))
    {
        return;
    }

    sender.hasRemoteSession = true;
    sender.remoteSessionId = trackerSessionId;
    sender.sosActive = true;
    sender.activeSosCause = static_cast<DmSosCause>(rawCause);

    if (!audio_.startSos())
    {
        LOG_ERROR("{Alarm} Cause=SOS buzzer unavailable");
        return;
    }

    sendSosAck(sender.nodeNum, header.sequence, trackerSessionId);
    LOG_WARN(
        "{RX} SOS from id=!%08lx [cause=%s seq=%lu]",
        static_cast<unsigned long>(sender.nodeNum),
        dmSosCauseName(sender.activeSosCause),
        static_cast<unsigned long>(header.sequence));
}

void DistanceMonitorModule::handleSosAck(const meshtastic_MeshPacket &mp, uint32_t)
{
    if (mp.decoded.payload.size < DM_SOS_ACK_SIZE || !pendingSos_.active)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t ackedSequence = readU32Le(payload, 8U);
    const uint32_t trackerSessionId = readU32Le(payload, 12U);
    if (trackerSessionId != bootSessionId_ || ackedSequence != pendingSos_.sequence)
        return;

    pendingSos_.active = false;
    LOG_INFO(
        "{RX} applicative ACK from id=!%08lx [SOS seq=%lu]",
        static_cast<unsigned long>(mp.from),
        static_cast<unsigned long>(ackedSequence));
}

void DistanceMonitorModule::handleNotification(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    const DmMessageHeader &header,
    bool duplicate,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_NOTIFICATION_SIZE || !sender.isBase)
        return;

    noteRemoteSession(sender, readU32Le(mp.decoded.payload.bytes, 8U), nowMs);
    if (!duplicate)
    {
        audio_.playTrackerNotification();
        LOG_INFO(
            "{RX} notification from id=!%08lx [seq=%lu]",
            static_cast<unsigned long>(sender.nodeNum),
            static_cast<unsigned long>(header.sequence));
    }
    sendNotificationAck(sender.nodeNum, header.sequence, bootSessionId_);
}

void DistanceMonitorModule::handleNotificationAck(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    uint32_t)
{
    if (mp.decoded.payload.size < DM_NOTIFICATION_ACK_SIZE || sender.isBase)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t ackedSequence = readU32Le(payload, 8U);
    const uint32_t trackerSessionId = readU32Le(payload, 12U);
    if (sender.remoteSessionId != trackerSessionId ||
        sender.pendingNotificationSequence != ackedSequence)
    {
        return;
    }

    sender.notificationAcked = true;
    LOG_INFO(
        "{RX} applicative ACK from id=!%08lx [notification seq=%lu]",
        static_cast<unsigned long>(sender.nodeNum),
        static_cast<unsigned long>(ackedSequence));
}

void DistanceMonitorModule::handleShutdownNotice(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    uint32_t nowMs)
{
    if (mp.decoded.payload.size < DM_SHUTDOWN_NOTICE_SIZE || sender.isBase)
        return;

    const uint32_t trackerSessionId = readU32Le(mp.decoded.payload.bytes, 8U);
    noteRemoteSession(sender, trackerSessionId, nowMs);
    sender.shutdownNoticeReceived = true;
    sender.paired = false;
    sender.pairedLocalSessionId = 0U;
    sender.pairedRemoteSessionId = 0U;

    LOG_WARN("{RX} shutdown from id=!%08lx", static_cast<unsigned long>(sender.nodeNum));
}

bool DistanceMonitorModule::sendAliveRequest(uint32_t target)
{
    uint8_t body[8] = {};
    writeU32Le(body, 0U, bootSessionId_);
    writeU32Le(body, 4U, uptimeSeconds(millis()));
    return sendPacket(target, DmMessageType::AliveRequest, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendAliveResponse(uint32_t target)
{
    uint8_t body[8] = {};
    writeU32Le(body, 0U, bootSessionId_);
    writeU32Le(body, 4U, uptimeSeconds(millis()));
    return sendPacket(target, DmMessageType::AliveResponse, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendPairConfirm(
    uint32_t target,
    uint32_t trackerSessionId,
    bool playTrackerTone)
{
    uint8_t body[9] = {};
    writeU32Le(body, 0U, bootSessionId_);
    writeU32Le(body, 4U, trackerSessionId);
    body[8U] = playTrackerTone ? DM_PAIR_FLAG_PLAY_TRACKER_TONE : 0U;
    return sendPacket(target, DmMessageType::PairConfirm, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendBaseBeacon()
{
    uint8_t body[4] = {};
    writeU32Le(body, 0U, bootSessionId_);
    return sendPacket(NODENUM_BROADCAST, DmMessageType::BaseBeacon, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendPositionReport(
    uint32_t target,
    const DmNodeState &localState,
    uint32_t nowMs)
{
    uint8_t body[19] = {};
    writeU32Le(body, 0U, bootSessionId_);
    body[4U] = static_cast<uint8_t>(localState.positionKind);

    uint8_t flags = 0U;
    if (localMoving_)
        flags |= DM_POSITION_FLAG_MOVING;

    size_t baseIndex = 0U;
    if (findBaseIndex(baseIndex) && nodeStates_[baseIndex].paired)
        flags |= DM_POSITION_FLAG_PAIRED;

    if (baseBeaconRssi_.isUsable(nowMs, rssiBeaconMaxAgeMs()))
    {
        flags |= DM_POSITION_FLAG_BASE_RSSI_VALID;
        body[16U] = static_cast<uint8_t>(encodeSignedByte(baseBeaconRssi_.meanDbm()));
        body[17U] = encodeUnsignedByte(baseBeaconRssi_.stdDb());
        body[18U] = static_cast<uint8_t>(encodeSignedByte(baseBeaconRssi_.trendDbPerSec() * 10.0F));
    }

    body[5U] = flags;
    body[6U] = localAppliedIntervalSec_;
    body[7U] = currentBatteryPercent();
    const bool hasPosition = localState.positionKind != DmPositionKind::NoFix;
    writeU32Le(body, 8U, hasPosition ? static_cast<uint32_t>(localState.latitudeI) : 0U);
    writeU32Le(body, 12U, hasPosition ? static_cast<uint32_t>(localState.longitudeI) : 0U);

    return sendPacket(target, DmMessageType::PositionReport, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendSetPositionInterval(
    uint32_t target,
    uint32_t trackerSessionId,
    uint8_t intervalSec)
{
    uint8_t body[5] = {};
    writeU32Le(body, 0U, trackerSessionId);
    body[4U] = intervalSec;
    return sendPacket(target, DmMessageType::SetPositionInterval, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendSos(uint32_t target, uint32_t sequence, DmSosCause cause)
{
    uint8_t body[5] = {};
    writeU32Le(body, 0U, bootSessionId_);
    body[4U] = static_cast<uint8_t>(cause);
    return sendPacket(target, DmMessageType::Sos, sequence, body, sizeof(body), true, false);
}

bool DistanceMonitorModule::sendSosAck(
    uint32_t target,
    uint32_t sosSequence,
    uint32_t trackerSessionId)
{
    uint8_t body[8] = {};
    writeU32Le(body, 0U, sosSequence);
    writeU32Le(body, 4U, trackerSessionId);
    return sendPacket(target, DmMessageType::SosAck, allocateSequenceNumber(), body, sizeof(body), true, false);
}

bool DistanceMonitorModule::sendNotification(uint32_t target, uint32_t sequence)
{
    uint8_t body[4] = {};
    writeU32Le(body, 0U, bootSessionId_);
    return sendPacket(target, DmMessageType::Notification, sequence, body, sizeof(body), true, false);
}

bool DistanceMonitorModule::sendNotificationAck(
    uint32_t target,
    uint32_t notificationSequence,
    uint32_t trackerSessionId)
{
    uint8_t body[8] = {};
    writeU32Le(body, 0U, notificationSequence);
    writeU32Le(body, 4U, trackerSessionId);
    return sendPacket(target, DmMessageType::NotificationAck, allocateSequenceNumber(), body, sizeof(body), true, false);
}

bool DistanceMonitorModule::sendShutdownNotice(uint32_t target)
{
    uint8_t body[4] = {};
    writeU32Le(body, 0U, bootSessionId_);
    return sendPacket(target, DmMessageType::ShutdownNotice, allocateSequenceNumber(), body, sizeof(body), false, false);
}

bool DistanceMonitorModule::sendPacket(
    uint32_t target,
    DmMessageType type,
    uint32_t sequence,
    const uint8_t *body,
    size_t bodySize,
    bool reliable,
    bool wantAck)
{
    if (service == nullptr || target == 0U || sequence == 0U)
        return false;

    meshtastic_MeshPacket *packet = allocDataPacket();
    if (packet == nullptr)
        return false;

    const size_t totalSize = DM_PROTOCOL_HEADER_SIZE + bodySize;
    if (totalSize > sizeof(packet->decoded.payload.bytes))
        return false;

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

    if (bodySize > 0U && body != nullptr)
        std::memcpy(packet->decoded.payload.bytes + encodedSize, body, bodySize);

    packet->to = target;
    packet->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    packet->decoded.payload.size = totalSize;
    packet->decoded.want_response = false;
    packet->want_ack = wantAck;
    packet->hop_limit = 0U;
    if (reliable)
        packet->priority = meshtastic_MeshPacket_Priority_RELIABLE;

    switch (type)
    {
    case DmMessageType::SetPositionInterval:
        LOG_INFO(
            "{TX} to id=!%08lx setinterval (%u s)",
            static_cast<unsigned long>(target),
            bodySize >= 5U ? static_cast<unsigned>(body[4U]) : 0U);
        break;
    case DmMessageType::Notification:
        LOG_INFO("{TX} to id=!%08lx notification", static_cast<unsigned long>(target));
        break;
    case DmMessageType::NotificationAck:
        LOG_INFO("{TX} to id=!%08lx applicative ACK (notification)", static_cast<unsigned long>(target));
        break;
    case DmMessageType::Sos:
        LOG_INFO("{TX} to id=!%08lx SOS", static_cast<unsigned long>(target));
        break;
    case DmMessageType::SosAck:
        LOG_INFO("{TX} to id=!%08lx applicative ACK (SOS)", static_cast<unsigned long>(target));
        break;
    case DmMessageType::ShutdownNotice:
        LOG_INFO("{TX} to id=!%08lx shutdown", static_cast<unsigned long>(target));
        break;
    case DmMessageType::PositionReport:
        LOG_DEBUG("{TX} to id=!%08lx report", static_cast<unsigned long>(target));
        break;
    case DmMessageType::PairConfirm:
        LOG_DEBUG("{TX} to id=!%08lx pair confirm", static_cast<unsigned long>(target));
        break;
    case DmMessageType::AliveRequest:
    case DmMessageType::AliveResponse:
    case DmMessageType::BaseBeacon:
    default:
        break;
    }

    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    return true;
}

uint32_t DistanceMonitorModule::allocateSequenceNumber()
{
    const uint32_t allocated = nextSequenceNumber_++;
    if (nextSequenceNumber_ == 0U)
        nextSequenceNumber_ = 1U;
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
        return false;

    buffer[0U] = DM_PROTOCOL_MAGIC_0;
    buffer[1U] = DM_PROTOCOL_MAGIC_1;
    buffer[2U] = DM_PROTOCOL_VERSION;
    buffer[3U] = static_cast<uint8_t>(type);
    writeU32Le(buffer, 4U, sequence);
    encodedSize = DM_PROTOCOL_HEADER_SIZE;
    return true;
}

bool DistanceMonitorModule::decodeMessageHeader(
    const uint8_t *buffer,
    size_t bufferSize,
    DmMessageHeader &header) const
{
    if (buffer == nullptr ||
        bufferSize < DM_PROTOCOL_HEADER_SIZE ||
        buffer[0U] != DM_PROTOCOL_MAGIC_0 ||
        buffer[1U] != DM_PROTOCOL_MAGIC_1 ||
        buffer[2U] != DM_PROTOCOL_VERSION)
    {
        return false;
    }

    const uint8_t rawType = buffer[3U];
    if (rawType < static_cast<uint8_t>(DmMessageType::AliveRequest) ||
        rawType > static_cast<uint8_t>(DmMessageType::ShutdownNotice))
    {
        return false;
    }

    header.version = buffer[2U];
    header.type = static_cast<DmMessageType>(rawType);
    header.sequence = readU32Le(buffer, 4U);
    return header.sequence != 0U;
}

bool DistanceMonitorModule::acceptSequence(
    DmNodeState &state,
    uint32_t sequence,
    uint32_t nowMs,
    bool &duplicate)
{
    duplicate = false;
    size_t freeIndex = DM_RECENT_SEQUENCE_COUNT;
    size_t oldestIndex = 0U;
    uint32_t oldestAge = 0U;

    for (size_t index = 0U; index < DM_RECENT_SEQUENCE_COUNT; ++index)
    {
        DmRecentSequence &entry = state.recentSequences[index];
        if (entry.sequence == sequence &&
            dmElapsedMs(nowMs, entry.receivedMs) <= DM_SEQUENCE_DUPLICATE_WINDOW_MS)
        {
            duplicate = true;
            entry.receivedMs = nowMs;
            return true;
        }

        if (entry.sequence == 0U && freeIndex == DM_RECENT_SEQUENCE_COUNT)
            freeIndex = index;

        const uint32_t age =
            entry.sequence == 0U ? 0U : dmElapsedMs(nowMs, entry.receivedMs);
        if (age >= oldestAge)
        {
            oldestAge = age;
            oldestIndex = index;
        }
    }

    const size_t targetIndex =
        freeIndex < DM_RECENT_SEQUENCE_COUNT ? freeIndex : oldestIndex;
    state.recentSequences[targetIndex].sequence = sequence;
    state.recentSequences[targetIndex].receivedMs = nowMs;
    return true;
}

bool DistanceMonitorModule::isDirectPacket(const meshtastic_MeshPacket &packet) const
{
    const bool lora =
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1 ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT2 ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT3;
    return lora && !packet.via_mqtt && packet.hop_limit == packet.hop_start;
}
