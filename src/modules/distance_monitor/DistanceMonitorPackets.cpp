#include "DistanceMonitorModule.h"

#include "DistanceMonitorUtils.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/mesh-pb-constants.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace
{
void writeU16Le(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset] = static_cast<uint8_t>(value & 0xFFU);
    buffer[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

uint16_t readU16Le(const uint8_t *buffer, size_t offset)
{
    return static_cast<uint16_t>(buffer[offset]) |
           static_cast<uint16_t>(
               static_cast<uint16_t>(buffer[offset + 1U]) << 8U);
}

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
    return static_cast<int8_t>(
        std::max<long>(-127L, std::min<long>(127L, rounded)));
}

void formatRssi(bool valid, float valueDbm, char *buffer, size_t size)
{
    if (buffer == nullptr || size == 0U)
        return;

    if (!valid || !std::isfinite(valueDbm))
    {
        std::snprintf(buffer, size, "NA");
        return;
    }

    std::snprintf(buffer, size, "%lddBm", std::lround(valueDbm));
}

void formatCoordinateE7(int32_t valueE7, char *buffer, size_t size)
{
    if (buffer == nullptr || size == 0U)
        return;

    const long raw = static_cast<long>(valueE7);
    const bool negative = raw < 0L;
    const unsigned long magnitude =
        static_cast<unsigned long>(negative ? -raw : raw);
    const unsigned long degrees = magnitude / 10000000UL;
    const unsigned long fraction = magnitude % 10000000UL;

    std::snprintf(
        buffer,
        size,
        "%s%lu.%07lu",
        negative ? "-" : "",
        degrees,
        fraction);
}
} // namespace

ProcessMessage DistanceMonitorModule::handleReceived(
    const meshtastic_MeshPacket &mp)
{
    initializeIfNeeded();

    size_t senderIndex = 0U;
    if (mp.from == 0U ||
        !findMemberIndex(mp.from, senderIndex) ||
        mp.decoded.portnum != meshtastic_PortNum_PRIVATE_APP)
    {
        return ProcessMessage::CONTINUE;
    }

    DmMessageHeader header;
    if (!decodeMessageHeader(
            mp.decoded.payload.bytes,
            mp.decoded.payload.size,
            header))
    {
        return ProcessMessage::CONTINUE;
    }

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
    DmSessionId sessionId,
    uint32_t nowMs)
{
    (void)nowMs;

    const bool changed =
        sender.hasRemoteSession && sender.remoteSessionId != sessionId;
    if (changed)
    {
        const bool previouslyPaired = sender.paired || sender.everPaired;
        const DmSessionId oldSession = sender.remoteSessionId;

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
                "{DM@Alarm} Cause=UNPAIRED id=!%08lx session=%04x->%04x silent=%s",
                static_cast<unsigned long>(sender.nodeNum),
                static_cast<unsigned>(oldSession),
                static_cast<unsigned>(sessionId),
                DM_ALARM_AUDIO_SILENT ? "YES" : "NO");
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

    LOG_INFO(
        "{DM@Pair} id=!%08lx restored",
        static_cast<unsigned long>(sender.nodeNum));
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
    noteRemoteSession(sender, readU16Le(payload, 8U), nowMs);
    sender.remoteUptimeSeconds = readU32Le(payload, 10U);
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
    noteRemoteSession(sender, readU16Le(payload, 8U), nowMs);
    sender.remoteUptimeSeconds = readU32Le(payload, 10U);
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
    const DmSessionId baseSessionId = readU16Le(payload, 8U);
    const DmSessionId trackerSessionId = readU16Le(payload, 10U);
    const uint8_t flags = payload[12U];

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
        // Pairing confirmation remains audible even in alarm silent mode.
        audio_.playPairingBops();
    }

    if (!samePair)
    {
        LOG_INFO(
            "{DM@Pair} id=!%08lx paired session=%04x",
            static_cast<unsigned long>(sender.nodeNum),
            static_cast<unsigned>(baseSessionId));
    }

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
    const DmSessionId trackerSessionId = readU16Le(payload, 8U);
    const uint8_t flags = payload[10U];

    noteRemoteSession(sender, trackerSessionId, nowMs);

    sender.positionKind =
        (flags & DM_POSITION_FLAG_FIX) != 0U
            ? DmPositionKind::FreshFix
            : DmPositionKind::NoFix;
    sender.appliedIntervalSec = payload[11U];
    sender.batteryPercent = payload[12U];
    sender.moving = (flags & DM_POSITION_FLAG_MOVING) != 0U;

    if (sender.positionKind == DmPositionKind::FreshFix)
    {
        const int32_t latitudeI = static_cast<int32_t>(readU32Le(payload, 13U));
        const int32_t longitudeI = static_cast<int32_t>(readU32Le(payload, 17U));
        const uint16_t positionDop = readU16Le(payload, 22U);

        // A report advertising FIX without coordinates/DOP is malformed for V7.
        if (latitudeI == 0 || longitudeI == 0 || positionDop == 0U)
        {
            sender.positionKind = DmPositionKind::NoFix;
            sender.positionDop = 0U;
            sender.positionAccuracyMeters =
                std::numeric_limits<double>::infinity();
        }
        else
        {
            sender.latitudeI = latitudeI;
            sender.longitudeI = longitudeI;
            sender.positionDop = positionDop;
            sender.positionAccuracyMeters =
                dmAccuracyMetersFromDop(sender.positionDop);
            sender.lastFixRxMs = nowMs;
            sender.hasLastFixRxTime = true;
        }
    }
    else
    {
        // Keep latitude/longitude as last-known position for UI/recovery use.
        // positionKind remains authoritative: stale coordinates are never used
        // for the live GPS distance decision.
        sender.positionDop = 0U;
        sender.positionAccuracyMeters =
            std::numeric_limits<double>::infinity();
    }

    sender.remoteBaseRssiValid =
        (flags & DM_POSITION_FLAG_BASE_RSSI_VALID) != 0U;
    if (sender.remoteBaseRssiValid)
    {
        sender.remoteBaseRssiDbm =
            static_cast<float>(static_cast<int8_t>(payload[21U]));
    }
    else
    {
        sender.remoteBaseRssiDbm = 0.0F;
    }

    sender.lastPositionReportRxMs = nowMs;
    sender.lastPositionReportSequence = header.sequence;
    sender.hasPositionReportRxTime = true;
    sender.radioState = DmRadioState::Alive;

    if (isRadioFault(sender.faultCause) &&
        sender.faultCause != DmFaultCause::Unpaired)
    {
        LOG_INFO(
            "{DM@RX} link restored id=!%08lx previous=%s",
            static_cast<unsigned long>(sender.nodeNum),
            dmFaultCauseName(sender.faultCause));
        sender.faultCause = DmFaultCause::None;
        sender.comSaturationSilentUntilMs = 0U;
        sender.faultSnoozedUntilMs = 0U;
    }

    sender.shutdownNoticeReceived = false;

    const bool rssiTbValid = isDirectPacket(mp);
    const float rssiTbDbm =
        rssiTbValid ? static_cast<float>(mp.rx_rssi) : 0.0F;
    if (rssiTbValid)
        directInboundRssi_[senderIndex].update(rssiTbDbm, nowMs);

    // A valid report is a visible RX event on the base.
    triggerStatusLedDoubleBlink();

    const bool pairedReport =
        (flags & DM_POSITION_FLAG_PAIRED) != 0U;
    if (pairedReport &&
        !sender.paired &&
        sender.pairedLocalSessionId == bootSessionId_ &&
        sender.pairedRemoteSessionId == trackerSessionId)
    {
        sender.paired = true;
        sender.everPaired = true;
        clearUnpairedFault(sender);
        audio_.playPairingBops();
        LOG_INFO(
            "{DM@Pair} id=!%08lx paired",
            static_cast<unsigned long>(sender.nodeNum));
    }

    char battery[8] = {};
    char rssiBt[16] = {};
    char rssiTb[16] = {};
    dmFormatBattery(sender.batteryPercent, battery, sizeof(battery));
    formatRssi(
        sender.remoteBaseRssiValid,
        sender.remoteBaseRssiDbm,
        rssiBt,
        sizeof(rssiBt));
    formatRssi(rssiTbValid, rssiTbDbm, rssiTb, sizeof(rssiTb));

    if (sender.positionKind == DmPositionKind::FreshFix)
    {
        char latitude[24] = {};
        char longitude[24] = {};
        formatCoordinateE7(sender.latitudeI, latitude, sizeof(latitude));
        formatCoordinateE7(sender.longitudeI, longitude, sizeof(longitude));

        LOG_INFO(
            "{DM@RX} report from id=!%08lx [pos=FIX Lat=%s Lon=%s dop=%u acc=%.0fm bat=%s state=%s interval=%us RSSI_BT=%s RSSI_TB=%s]",
            static_cast<unsigned long>(sender.nodeNum),
            latitude,
            longitude,
            static_cast<unsigned>(sender.positionDop),
            sender.positionAccuracyMeters,
            battery,
            sender.moving ? "MOVING" : "STATIONARY",
            static_cast<unsigned>(sender.appliedIntervalSec),
            rssiBt,
            rssiTb);
    }
    else
    {
        LOG_INFO(
            "{DM@RX} report from id=!%08lx [pos=NO_FIX bat=%s state=%s interval=%us RSSI_BT=%s RSSI_TB=%s]",
            static_cast<unsigned long>(sender.nodeNum),
            battery,
            sender.moving ? "MOVING" : "STATIONARY",
            static_cast<unsigned>(sender.appliedIntervalSec),
            rssiBt,
            rssiTb);
    }
}

void DistanceMonitorModule::handleSetPositionInterval(
    DmNodeState &sender,
    const meshtastic_MeshPacket &mp,
    uint32_t)
{
    if (mp.decoded.payload.size < DM_SET_INTERVAL_SIZE || !sender.isBase)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const DmSessionId trackerSessionId = readU16Le(payload, 8U);
    if (trackerSessionId != bootSessionId_)
        return;

    const uint8_t requested = payload[10U];
    const uint8_t maximum =
        dmComputeMaxReportIntervalSec(runtimeConfig_.maxDistanceMeters);
    localAppliedIntervalSec_ = std::max<uint8_t>(
        DM_MIN_REPORT_INTERVAL_S,
        std::min<uint8_t>(requested, maximum));
    forcePositionReport_ = true;

    LOG_INFO(
        "{DM@RX} setinterval from id=!%08lx (%u s)",
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

    noteRemoteSession(sender, readU16Le(mp.decoded.payload.bytes, 8U), nowMs);
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
    const DmSessionId trackerSessionId = readU16Le(payload, 8U);
    const uint8_t rawCause = payload[10U];
    if (rawCause < static_cast<uint8_t>(DmSosCause::ManualButton) ||
        rawCause > static_cast<uint8_t>(DmSosCause::HighSpeedMovement))
    {
        return;
    }

    sender.hasRemoteSession = true;
    sender.remoteSessionId = trackerSessionId;
    sender.sosActive = true;
    sender.activeSosCause = static_cast<DmSosCause>(rawCause);
    triggerStatusLedDoubleBlink();

    if (!DM_ALARM_AUDIO_SILENT && !audio_.startSos())
        LOG_ERROR("{DM@Alarm} Cause=SOS buzzer unavailable");

    sendSosAck(sender.nodeNum, header.sequence, trackerSessionId);

    LOG_WARN(
        "{DM@Alarm} Cause=SOS id=!%08lx source=remote cause=%s seq=%lu silent=%s",
        static_cast<unsigned long>(sender.nodeNum),
        dmSosCauseName(sender.activeSosCause),
        static_cast<unsigned long>(header.sequence),
        DM_ALARM_AUDIO_SILENT ? "YES" : "NO");
}

void DistanceMonitorModule::handleSosAck(
    const meshtastic_MeshPacket &mp,
    uint32_t)
{
    if (mp.decoded.payload.size < DM_SOS_ACK_SIZE || !pendingSos_.active)
        return;

    const uint8_t *payload = mp.decoded.payload.bytes;
    const uint32_t ackedSequence = readU32Le(payload, 8U);
    const DmSessionId trackerSessionId = readU16Le(payload, 12U);
    if (trackerSessionId != bootSessionId_ ||
        ackedSequence != pendingSos_.sequence)
    {
        return;
    }

    pendingSos_.active = false;
    LOG_INFO(
        "{DM@RX} applicative ACK from id=!%08lx [SOS seq=%lu]",
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

    noteRemoteSession(sender, readU16Le(mp.decoded.payload.bytes, 8U), nowMs);

    if (!duplicate)
    {
        // User notification remains audible in silent alarm mode.
        audio_.playTrackerNotification();
        LOG_INFO(
            "{DM@RX} notification from id=!%08lx [seq=%lu]",
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
    const DmSessionId trackerSessionId = readU16Le(payload, 12U);

    if (sender.remoteSessionId != trackerSessionId ||
        sender.pendingNotificationSequence != ackedSequence)
    {
        return;
    }

    sender.notificationAcked = true;
    LOG_INFO(
        "{DM@RX} applicative ACK from id=!%08lx [notification seq=%lu]",
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

    const DmSessionId trackerSessionId =
        readU16Le(mp.decoded.payload.bytes, 8U);
    noteRemoteSession(sender, trackerSessionId, nowMs);
    sender.shutdownNoticeReceived = true;
    sender.paired = false;
    sender.pairedLocalSessionId = 0U;
    sender.pairedRemoteSessionId = 0U;

    LOG_WARN(
        "{DM@RX} shutdown from id=!%08lx",
        static_cast<unsigned long>(sender.nodeNum));
}

bool DistanceMonitorModule::sendAliveRequest(uint32_t target)
{
    uint8_t body[6] = {};
    writeU16Le(body, 0U, bootSessionId_);
    writeU32Le(body, 2U, uptimeSeconds(millis()));

    return sendPacket(
        target,
        DmMessageType::AliveRequest,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);
}

bool DistanceMonitorModule::sendAliveResponse(uint32_t target)
{
    uint8_t body[6] = {};
    writeU16Le(body, 0U, bootSessionId_);
    writeU32Le(body, 2U, uptimeSeconds(millis()));

    return sendPacket(
        target,
        DmMessageType::AliveResponse,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);
}

bool DistanceMonitorModule::sendPairConfirm(
    uint32_t target,
    DmSessionId trackerSessionId,
    bool playTrackerTone)
{
    uint8_t body[5] = {};
    writeU16Le(body, 0U, bootSessionId_);
    writeU16Le(body, 2U, trackerSessionId);
    body[4U] = playTrackerTone
                   ? static_cast<uint8_t>(DM_PAIR_FLAG_PLAY_TRACKER_TONE)
                   : 0U;

    return sendPacket(
        target,
        DmMessageType::PairConfirm,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);
}

bool DistanceMonitorModule::sendBaseBeacon()
{
    uint8_t body[2] = {};
    writeU16Le(body, 0U, bootSessionId_);

    return sendPacket(
        NODENUM_BROADCAST,
        DmMessageType::BaseBeacon,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);
}

bool DistanceMonitorModule::sendPositionReport(
    uint32_t target,
    const DmNodeState &localState,
    uint32_t nowMs)
{
    // V7 PositionReport body (16 bytes):
    // session[2], flags[1], interval[1], battery[1], lat[4], lon[4],
    // RSSI_BT[1], DOP[2]. Position kind is encoded in flags.
    uint8_t body[16] = {};
    writeU16Le(body, 0U, bootSessionId_);

    uint8_t flags = 0U;
    if (localMoving_)
        flags |= DM_POSITION_FLAG_MOVING;

    const bool hasPosition =
        localState.positionKind == DmPositionKind::FreshFix &&
        localState.latitudeI != 0 &&
        localState.longitudeI != 0 &&
        localState.positionDop != 0U;
    if (hasPosition)
        flags |= DM_POSITION_FLAG_FIX;

    size_t baseIndex = 0U;
    if (findBaseIndex(baseIndex) && nodeStates_[baseIndex].paired)
        flags |= DM_POSITION_FLAG_PAIRED;

    if (baseBeaconRssi_.isInitialized() &&
        dmElapsedMs(nowMs, baseBeaconRssi_.lastUpdateMs()) <=
            rssiBeaconMaxAgeMs())
    {
        flags |= DM_POSITION_FLAG_BASE_RSSI_VALID;
        body[13U] = static_cast<uint8_t>(
            encodeSignedByte(baseBeaconRssi_.meanDbm()));
    }

    body[2U] = flags;
    body[3U] = localAppliedIntervalSec_;
    body[4U] = currentBatteryPercent();
    writeU32Le(
        body,
        5U,
        hasPosition ? static_cast<uint32_t>(localState.latitudeI) : 0U);
    writeU32Le(
        body,
        9U,
        hasPosition ? static_cast<uint32_t>(localState.longitudeI) : 0U);
    writeU16Le(body, 14U, hasPosition ? localState.positionDop : 0U);

    const bool sent = sendPacket(
        target,
        DmMessageType::PositionReport,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);

    if (sent)
        triggerStatusLedDoubleBlink();

    return sent;
}

bool DistanceMonitorModule::sendSetPositionInterval(
    uint32_t target,
    DmSessionId trackerSessionId,
    uint8_t intervalSec)
{
    uint8_t body[3] = {};
    writeU16Le(body, 0U, trackerSessionId);
    body[2U] = intervalSec;

    return sendPacket(
        target,
        DmMessageType::SetPositionInterval,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);
}

bool DistanceMonitorModule::sendSos(
    uint32_t target,
    uint32_t sequence,
    DmSosCause cause)
{
    uint8_t body[3] = {};
    writeU16Le(body, 0U, bootSessionId_);
    body[2U] = static_cast<uint8_t>(cause);

    const bool sent = sendPacket(
        target,
        DmMessageType::Sos,
        sequence,
        body,
        sizeof(body),
        true,
        false);

    if (sent)
        triggerStatusLedDoubleBlink();

    return sent;
}

bool DistanceMonitorModule::sendSosAck(
    uint32_t target,
    uint32_t sosSequence,
    DmSessionId trackerSessionId)
{
    uint8_t body[6] = {};
    writeU32Le(body, 0U, sosSequence);
    writeU16Le(body, 4U, trackerSessionId);

    return sendPacket(
        target,
        DmMessageType::SosAck,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        true,
        false);
}

bool DistanceMonitorModule::sendNotification(
    uint32_t target,
    uint32_t sequence)
{
    uint8_t body[2] = {};
    writeU16Le(body, 0U, bootSessionId_);

    return sendPacket(
        target,
        DmMessageType::Notification,
        sequence,
        body,
        sizeof(body),
        true,
        false);
}

bool DistanceMonitorModule::sendNotificationAck(
    uint32_t target,
    uint32_t notificationSequence,
    DmSessionId trackerSessionId)
{
    uint8_t body[6] = {};
    writeU32Le(body, 0U, notificationSequence);
    writeU16Le(body, 4U, trackerSessionId);

    return sendPacket(
        target,
        DmMessageType::NotificationAck,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        true,
        false);
}

bool DistanceMonitorModule::sendShutdownNotice(uint32_t target)
{
    uint8_t body[2] = {};
    writeU16Le(body, 0U, bootSessionId_);

    return sendPacket(
        target,
        DmMessageType::ShutdownNotice,
        allocateSequenceNumber(),
        body,
        sizeof(body),
        false,
        false);
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
    {
        std::memcpy(
            packet->decoded.payload.bytes + encodedSize,
            body,
            bodySize);
    }

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
            "{DM@TX} to id=!%08lx setinterval (%u s)",
            static_cast<unsigned long>(target),
            bodySize >= 3U ? static_cast<unsigned>(body[2U]) : 0U);
        break;
    case DmMessageType::Notification:
        LOG_INFO(
            "{DM@TX} to id=!%08lx notification",
            static_cast<unsigned long>(target));
        break;
    case DmMessageType::NotificationAck:
        LOG_INFO(
            "{DM@TX} to id=!%08lx applicative ACK (notification)",
            static_cast<unsigned long>(target));
        break;
    case DmMessageType::Sos:
        LOG_INFO(
            "{DM@TX} to id=!%08lx SOS",
            static_cast<unsigned long>(target));
        break;
    case DmMessageType::SosAck:
        LOG_INFO(
            "{DM@TX} to id=!%08lx applicative ACK (SOS)",
            static_cast<unsigned long>(target));
        break;
    case DmMessageType::ShutdownNotice:
        LOG_INFO(
            "{DM@TX} to id=!%08lx shutdown",
            static_cast<unsigned long>(target));
        break;
    case DmMessageType::PairConfirm:
        LOG_DEBUG(
            "{DM@TX} to id=!%08lx pair confirm",
            static_cast<unsigned long>(target));
        break;
    case DmMessageType::AliveRequest:
    case DmMessageType::AliveResponse:
    case DmMessageType::PositionReport:
    case DmMessageType::BaseBeacon:
    default:
        break;
    }

    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    return true;
}

uint32_t DistanceMonitorModule::allocateSequenceNumber()
{
    uint32_t sequence = nextSequenceNumber_++;
    if (sequence == 0U)
        sequence = nextSequenceNumber_++;
    return sequence;
}

bool DistanceMonitorModule::encodeMessageHeader(
    DmMessageType type,
    uint32_t sequence,
    uint8_t *buffer,
    size_t bufferSize,
    size_t &encodedSize) const
{
    encodedSize = 0U;
    if (buffer == nullptr || bufferSize < DM_PROTOCOL_HEADER_SIZE)
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
    if (buffer == nullptr || bufferSize < DM_PROTOCOL_HEADER_SIZE)
        return false;

    if (buffer[0U] != DM_PROTOCOL_MAGIC_0 ||
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
    if (sequence == 0U)
        return false;

    for (size_t i = 0U; i < DM_RECENT_SEQUENCE_COUNT; ++i)
    {
        DmRecentSequence &recent = state.recentSequences[i];
        if (recent.sequence == sequence &&
            dmElapsedMs(nowMs, recent.receivedMs) <=
                DM_SEQUENCE_DUPLICATE_WINDOW_MS)
        {
            duplicate = true;
            return true;
        }
    }

    size_t oldestIndex = 0U;
    uint32_t oldestAge = 0U;
    for (size_t i = 0U; i < DM_RECENT_SEQUENCE_COUNT; ++i)
    {
        DmRecentSequence &recent = state.recentSequences[i];
        if (recent.sequence == 0U)
        {
            oldestIndex = i;
            oldestAge = UINT32_MAX;
            break;
        }

        const uint32_t age = dmElapsedMs(nowMs, recent.receivedMs);
        if (age >= oldestAge)
        {
            oldestAge = age;
            oldestIndex = i;
        }
    }

    state.recentSequences[oldestIndex].sequence = sequence;
    state.recentSequences[oldestIndex].receivedMs = nowMs;
    return true;
}

bool DistanceMonitorModule::isDirectPacket(
    const meshtastic_MeshPacket &packet) const
{
    // Keep the Meshtastic direct-LoRa test from 95170a3. This excludes MQTT
    // and relayed packets so RSSI_BT/RSSI_TB remain true direct-path samples.
    const bool lora =
        packet.transport_mechanism ==
            meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA ||
        packet.transport_mechanism ==
            meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1 ||
        packet.transport_mechanism ==
            meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT2 ||
        packet.transport_mechanism ==
            meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT3;

    return lora && !packet.via_mqtt && packet.hop_limit == packet.hop_start;
}
