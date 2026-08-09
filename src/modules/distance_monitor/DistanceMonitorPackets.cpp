#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/mesh-pb-constants.h"

#include <Arduino.h>

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
} // namespace

ProcessMessage DistanceMonitorModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    size_t senderIndex = 0U;
    if (mp.from == 0U || !findMemberIndex(mp.from, senderIndex))
    {
        return ProcessMessage::CONTINUE;
    }

    DmNodeState &sender = nodeStates_[senderIndex];
    const uint32_t nowMs = millis();
    updatePacketReceptionState(sender, mp, nowMs);

    if (mp.decoded.portnum != meshtastic_PortNum_PRIVATE_APP)
    {
        return ProcessMessage::CONTINUE;
    }

    DmMessageHeader header;
    if (!decodeMessageHeader(mp.decoded.payload.bytes, mp.decoded.payload.size, header) ||
        !acceptSequence(sender, header.sequence, nowMs))
    {
        return ProcessMessage::CONTINUE;
    }

    switch (header.type)
    {
    case DmMessageType::AliveRequest:
        LOG_INFO("Distance Monitor ALIVE request RX: from=!%08lx",
                 static_cast<unsigned long>(mp.from));
        sendAliveResponse(mp.from);
        break;

    case DmMessageType::AliveResponse:
        LOG_INFO("Distance Monitor ALIVE response RX: from=!%08lx",
                 static_cast<unsigned long>(mp.from));
        break;

    case DmMessageType::PositionRequest:
    {
        size_t localIndex = 0U;
        if (nodeDB == nullptr || !findMemberIndex(nodeDB->getNodeNum(), localIndex))
        {
            break;
        }

        const DmNodeState &local = nodeStates_[localIndex];
        LOG_INFO(
            "Distance Monitor position request RX: from=!%08lx -> %s age=%lus",
            static_cast<unsigned long>(mp.from),
            dmPositionKindName(local.positionKind),
            static_cast<unsigned long>(positionAgeSeconds(local, nowMs)));

        sendPositionResponse(mp.from, local, nowMs);
        break;
    }

    case DmMessageType::PositionResponse:
        if (decodePositionResponse(sender, mp, nowMs))
        {
            LOG_INFO(
                "Distance Monitor position response RX: from=!%08lx result=%s age=%lus",
                static_cast<unsigned long>(mp.from),
                dmPositionKindName(sender.positionKind),
                static_cast<unsigned long>(sender.positionAgeAtRxSeconds));
        }
        break;

    case DmMessageType::Sos:
        sender.sosPending = true;
        LOG_WARN("Distance Monitor SOS received: node=!%08lx seq=%lu",
                 static_cast<unsigned long>(mp.from),
                 static_cast<unsigned long>(header.sequence));
        break;
    }

    return ProcessMessage::CONTINUE;
}

bool DistanceMonitorModule::wantPacket(const meshtastic_MeshPacket *packet)
{
    return packet != nullptr &&
           packet->decoded.portnum == meshtastic_PortNum_PRIVATE_APP;
}

bool DistanceMonitorModule::requestAlive(uint32_t target)
{
    LOG_INFO("Distance Monitor ALIVE request TX: target=!%08lx",
             static_cast<unsigned long>(target));
    return sendPrivateMessage(target, DmMessageType::AliveRequest);
}

bool DistanceMonitorModule::sendAliveResponse(uint32_t target)
{
    LOG_INFO("Distance Monitor ALIVE response TX: target=!%08lx",
             static_cast<unsigned long>(target));
    return sendPrivateMessage(target, DmMessageType::AliveResponse);
}

bool DistanceMonitorModule::requestPosition(uint32_t target)
{
    LOG_INFO("Distance Monitor position request TX: target=!%08lx",
             static_cast<unsigned long>(target));
    return sendPrivateMessage(target, DmMessageType::PositionRequest);
}

bool DistanceMonitorModule::sendPositionResponse(
    uint32_t target,
    const DmNodeState &local,
    uint32_t nowMs)
{
    if (service == nullptr || target == 0U || target == NODENUM_BROADCAST)
    {
        return false;
    }

    meshtastic_MeshPacket *packet = allocDataPacket();
    if (packet == nullptr || sizeof(packet->decoded.payload.bytes) < DM_POSITION_RESPONSE_SIZE)
    {
        return false;
    }

    const uint32_t sequence = allocateSequenceNumber();
    size_t encodedSize = 0U;
    if (!encodeMessageHeader(
            DmMessageType::PositionResponse,
            sequence,
            packet->decoded.payload.bytes,
            sizeof(packet->decoded.payload.bytes),
            encodedSize))
    {
        return false;
    }

    const bool hasPosition = local.positionKind != DmPositionKind::NoFix;
    const DmPositionKind kind = hasPosition ? local.positionKind : DmPositionKind::NoFix;
    const uint32_t age = hasPosition ? positionAgeSeconds(local, nowMs) : 0U;

    uint8_t *payload = packet->decoded.payload.bytes;
    payload[8U] = static_cast<uint8_t>(kind);
    writeU32Le(payload, 9U, age);
    writeU32Le(payload, 13U, hasPosition ? static_cast<uint32_t>(local.latitudeI) : 0U);
    writeU32Le(payload, 17U, hasPosition ? static_cast<uint32_t>(local.longitudeI) : 0U);

    packet->to = target;
    packet->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    packet->decoded.payload.size = DM_POSITION_RESPONSE_SIZE;
    packet->decoded.want_response = false;
    packet->want_ack = true;
    packet->priority = meshtastic_MeshPacket_Priority_RELIABLE;

    LOG_INFO(
        "Distance Monitor position response TX: target=!%08lx result=%s age=%lus",
        static_cast<unsigned long>(target),
        dmPositionKindName(kind),
        static_cast<unsigned long>(age));

    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    return true;
}

bool DistanceMonitorModule::decodePositionResponse(
    DmNodeState &state,
    const meshtastic_MeshPacket &mp,
    uint32_t nowMs) const
{
    if (mp.decoded.payload.size < DM_POSITION_RESPONSE_SIZE)
    {
        return false;
    }

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint8_t rawKind = payload[8U];
    if (rawKind > static_cast<uint8_t>(DmPositionKind::CachedStationary))
    {
        return false;
    }

    state.positionKind = static_cast<DmPositionKind>(rawKind);
    state.positionAgeAtRxSeconds = readU32Le(payload, 9U);
    state.positionRxMs = nowMs;
    if (state.positionKind == DmPositionKind::NoFix)
    {
        state.positionAgeAtRxSeconds = 0U;
        return true;
    }

    state.latitudeI = static_cast<int32_t>(readU32Le(payload, 13U));
    state.longitudeI = static_cast<int32_t>(readU32Le(payload, 17U));
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
    if (service == nullptr || target == 0U || target == NODENUM_BROADCAST)
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
    const uint32_t sequence = allocateSequenceNumber();
    bool allSent = true;

    for (size_t index = 0U; index < runtimeConfig_.memberCount; ++index)
    {
        const uint32_t target = runtimeConfig_.members[index].nodeNum;
        if (target != localNodeNum &&
            !sendPrivateMessageWithSequence(target, DmMessageType::Sos, sequence))
        {
            allSent = false;
        }
    }
    return allSent;
}

uint32_t DistanceMonitorModule::allocateSequenceNumber()
{
    const uint32_t allocated = nextSequenceNumber_++;
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

    buffer[0] = DM_PROTOCOL_MAGIC_0;
    buffer[1] = DM_PROTOCOL_MAGIC_1;
    buffer[2] = DM_PROTOCOL_VERSION;
    buffer[3] = static_cast<uint8_t>(type);
    writeU32Le(buffer, 4U, sequence);
    encodedSize = DM_PROTOCOL_HEADER_SIZE;
    return true;
}

bool DistanceMonitorModule::decodeMessageHeader(
    const uint8_t *buffer,
    size_t bufferSize,
    DmMessageHeader &header) const
{
    if (buffer == nullptr || bufferSize < DM_PROTOCOL_HEADER_SIZE ||
        buffer[0] != DM_PROTOCOL_MAGIC_0 ||
        buffer[1] != DM_PROTOCOL_MAGIC_1 ||
        buffer[2] != DM_PROTOCOL_VERSION)
    {
        return false;
    }

    const uint8_t rawType = buffer[3];
    if (rawType < static_cast<uint8_t>(DmMessageType::AliveRequest) ||
        rawType > static_cast<uint8_t>(DmMessageType::PositionResponse))
    {
        return false;
    }

    header.version = buffer[2];
    header.type = static_cast<DmMessageType>(rawType);
    header.sequence = readU32Le(buffer, 4U);
    return header.sequence != 0U;
}

bool DistanceMonitorModule::acceptSequence(
    DmNodeState &state,
    uint32_t sequence,
    uint32_t nowMs)
{
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

    const float sampleRssi = static_cast<float>(packet.rx_rssi);
    state.radioMetricsValid = true;

    if (!state.rssiFilterInitialized)
    {
        state.filteredRssi = sampleRssi;
        state.rssiTrend = 0.0F;
        state.rssiFilterInitialized = true;
        return;
    }

    const float previous = state.filteredRssi;
    state.filteredRssi =
        DM_RSSI_FILTER_ALPHA * sampleRssi +
        (1.0F - DM_RSSI_FILTER_ALPHA) * previous;
    state.rssiTrend = state.filteredRssi - previous;
}

bool DistanceMonitorModule::isDirectPacket(const meshtastic_MeshPacket &packet) const
{
    const bool lora =
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1 ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT2 ||
        packet.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT3;

    return lora && !packet.via_mqtt &&
           packet.hop_start > 0U && packet.hop_limit == packet.hop_start;
}
