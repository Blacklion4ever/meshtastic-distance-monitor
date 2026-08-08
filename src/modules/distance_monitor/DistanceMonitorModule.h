#pragma once

#include "concurrency/OSThread.h"
#include "mesh/SinglePortModule.h"

#include "DistanceMonitorConfig.h"
#include "DistanceMonitorTypes.h"

/**
 * Core Distance Monitor Meshtastic module.
 *
 * Every device runs the same firmware. Its behavior is selected at runtime by
 * locating its own NodeNum in the member configuration and reading the isBase
 * flag. Bases actively maintain remote GNSS freshness and evaluate distances;
 * trackers maintain liveness, answer private protocol requests and may emit SOS.
 */
class DistanceMonitorModule : public SinglePortModule, private concurrency::OSThread
{
public:
    /** Construct the module on Meshtastic PRIVATE_APP. */
    DistanceMonitorModule();

    /** Initialize mutable configuration and schedule the first monitoring tick. */
    void setup() override;

    /**
     * Acknowledge all active local base warnings/alarms for DM_ALARM_REARM_MS.
     *
     * This method is intentionally local-only. No AlarmAck is sent over LoRa.
     * It is ready to be called later by the physical single-press button handler.
     */
    void acknowledgeLocalAlarms();

    /**
     * Send one SOS event by unicast to every other configured member.
     *
     * One sender-local sequence number is shared by all unicasts belonging to
     * the same SOS event. This method is ready for the future double-press handler.
     */
    bool sendLocalSos();

protected:
    /**
     * Execute one monitoring cycle.
     *
     * The function remains intentionally small: identity, state refresh,
     * per-member processing and compact logging are delegated to helpers.
     *
     * @return Delay in milliseconds before the next execution.
     */
    int32_t runOnce() override;

    /**
     * Receive Distance Monitor PRIVATE_APP messages and observe POSITION_APP.
     *
     * Full POSITION_APP payloads are decoded here because NodeInfoLite stores a
     * reduced PositionLite that does not contain HDOP, fix_type or the actual
     * GNSS solution timestamp required by Distance Monitor.
     *
     * @param mp Decoded Meshtastic packet delivered to this module.
     * @return CONTINUE so native Meshtastic modules can also process the packet.
     */
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

    /**
     * Subscribe to PRIVATE_APP and POSITION_APP packets.
     *
     * @param packet Candidate Meshtastic packet.
     * @return true when Distance Monitor wants to observe this packet.
     */
    bool wantPacket(const meshtastic_MeshPacket *packet) override;

private:
    /** Mutable configuration used by the running module. */
    DmRuntimeConfig runtimeConfig_;

    /** Dynamic state associated one-to-one with runtimeConfig_.members[]. */
    DmNodeState nodeStates_[DM_MAX_MEMBERS] = {};

    /**
     * Sender-local application sequence counter.
     *
     * The logical event identifier is the pair (sender NodeNum, sequence).
     * Each remote DmNodeState stores the last accepted sender sequence.
     */
    uint32_t nextSequenceNumber_ = 1U;

    /** Load mutable runtime configuration from DistanceMonitorConfig.h defaults. */
    void loadDefaultConfiguration();

    /**
     * Find a configured member by NodeNum.
     *
     * @param nodeNum Meshtastic node number to search for.
     * @param index Receives the member index on success.
     * @return true when the node belongs to this Distance Monitor group.
     */
    bool findMemberIndex(uint32_t nodeNum, size_t &index) const;

    /**
     * Refresh identity, NodeDB radio state and GNSS state for one member.
     *
     * @param index Index into runtimeConfig_.members[] and nodeStates_[].
     * @param localNodeNum NodeNum of the device running this module.
     * @param now Unix time in seconds; may be zero when unavailable.
     * @param absoluteTimeValid True when now can be used for absolute age checks.
     * @param nowMs Monotonic millis() value for cooldowns and fallback ages.
     */
    void refreshMemberState(
        size_t index,
        uint32_t localNodeNum,
        uint32_t now,
        bool absoluteTimeValid,
        uint32_t nowMs);

    /**
     * Evaluate radio freshness from last_heard, with monotonic receive fallback.
     *
     * @param state Runtime state to update.
     * @param node Current NodeDB entry; may be nullptr.
     * @param now Current Unix time.
     * @param absoluteTimeValid True when now is valid.
     * @param nowMs Current monotonic time.
     */
    void evaluateRadioState(
        DmNodeState &state,
        const meshtastic_NodeInfoLite *node,
        uint32_t now,
        bool absoluteTimeValid,
        uint32_t nowMs) const;

    /**
     * Evaluate coordinates, fix type, HDOP and sample freshness.
     *
     * @param state Runtime state containing the latest full position sample.
     * @param node Current NodeDB entry, used only as a PositionLite fallback.
     * @param now Current Unix time.
     * @param absoluteTimeValid True when now is valid.
     * @param nowMs Current monotonic time.
     */
    void evaluatePositionState(
        DmNodeState &state,
        const meshtastic_NodeInfoLite *node,
        uint32_t now,
        bool absoluteTimeValid,
        uint32_t nowMs) const;

    /**
     * Copy one full Meshtastic Position payload into a generic member state.
     *
     * @param state Runtime state receiving the GNSS fields.
     * @param position Full Position object to copy.
     * @param nowMs Current monotonic receive time.
     * @param markAsReceivedPacket True for a remote POSITION_APP reception.
     */
    void storeFullPositionSample(
        DmNodeState &state,
        const meshtastic_Position &position,
        uint32_t nowMs,
        bool markAsReceivedPacket) const;

    /**
     * Decode one full POSITION_APP payload and store its GNSS quality fields.
     *
     * @param state Runtime state of the packet sender.
     * @param mp Received POSITION_APP MeshPacket.
     * @param nowMs Current monotonic receive time.
     * @return true when the Position protobuf decoded successfully.
     */
    bool decodeAndStorePosition(
        DmNodeState &state,
        const meshtastic_MeshPacket &mp,
        uint32_t nowMs) const;

    /** Return true only when a member position is usable for distance calculation. */
    bool isPositionUsable(const DmNodeState &state) const;

    /**
     * Process liveness, GNSS requests, distance and base alarm state for one remote.
     *
     * @param localIndex Index of this device in the configured member list.
     * @param remoteIndex Index of the remote configured member.
     * @param nowMs Current monotonic time.
     */
    void processRemoteMember(
        size_t localIndex,
        size_t remoteIndex,
        uint32_t nowMs);

    /**
     * Update SAFE/WARNING/ALARM from a newly measured valid distance.
     * Warning and alarm both clear only below the 80 percent threshold.
     */
    void updateAlarmState(DmNodeState &remoteState);

    /**
     * Re-enable audible alarm eligibility after the local 60-second silence.
     * This only clears the acknowledgement flag; it does not change alarm state.
     */
    void updateAlarmAcknowledgement(DmNodeState &remoteState, uint32_t nowMs) const;

    /** Print one compact multi-line summary block for all configured members. */
    void logMemberSummary() const;

    /** Send an application-level ALIVE_REQUEST to one configured member. */
    bool requestAlive(uint32_t target);

    /** Send an application-level ALIVE_RESPONSE to one configured member. */
    bool sendAliveResponse(uint32_t target);

    /** Ask the native Meshtastic POSITION_APP module for a fresh remote position. */
    bool requestPosition(uint32_t target);

    /** Send one PRIVATE_APP message with a newly allocated local sequence number. */
    bool sendPrivateMessage(uint32_t target, DmMessageType type);

    /**
     * Send one PRIVATE_APP message with an explicit sequence number.
     * Used when one logical SOS is unicasted to several configured members.
     */
    bool sendPrivateMessageWithSequence(
        uint32_t target,
        DmMessageType type,
        uint32_t sequence);

    /** Send one SOS event by unicast to every other configured member. */
    bool sendSosToMembers(uint32_t localNodeNum);

    /** Allocate and return the next sender-local application sequence number. */
    uint32_t allocateSequenceNumber();

    /** Encode the fixed private-protocol header into a byte buffer. */
    bool encodeMessageHeader(
        DmMessageType type,
        uint32_t sequence,
        uint8_t *buffer,
        size_t bufferSize,
        size_t &encodedSize) const;

    /** Decode and validate the fixed private-protocol header. */
    bool decodeMessageHeader(
        const uint8_t *buffer,
        size_t bufferSize,
        DmMessageHeader &header) const;

    /**
     * Accept or reject one sender sequence.
     *
     * Exact duplicate sequences received within the duplicate window are
     * rejected. A different sequence is accepted immediately, which avoids
     * blocking a freshly rebooted sender that restarted its local counter.
     */
    bool acceptSequence(
        DmNodeState &state,
        uint32_t sequence,
        uint32_t nowMs);

    /**
     * Update packet reception timing and direct-link RSSI/SNR metadata.
     * Routed packets refresh liveness but never overwrite direct-link RSSI.
     */
    void updatePacketReceptionState(
        DmNodeState &state,
        const meshtastic_MeshPacket &packet,
        uint32_t nowMs);

    /** Return true when packet transport/hop metadata indicates direct LoRa reception. */
    bool isDirectPacket(const meshtastic_MeshPacket &packet) const;

    /** Return true when a monotonic retry cooldown has elapsed or never started. */
    bool retryAllowed(
        bool hasPreviousRequest,
        uint32_t previousRequestMs,
        uint32_t cooldownMs,
        uint32_t nowMs) const;
};
