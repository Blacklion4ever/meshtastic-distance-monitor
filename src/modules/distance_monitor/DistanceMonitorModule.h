#pragma once

#include "concurrency/OSThread.h"
#include "mesh/SinglePortModule.h"

#include "DistanceMonitorConfig.h"
#include "DistanceMonitorTypes.h"

class DistanceMonitorModule : public SinglePortModule, private concurrency::OSThread
{
public:
    DistanceMonitorModule();
    void setup() override;

    void acknowledgeLocalAlarms();
    bool sendLocalSos();

protected:
    int32_t runOnce() override;
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    bool wantPacket(const meshtastic_MeshPacket *packet) override;

private:
    DmRuntimeConfig runtimeConfig_;
    DmNodeState nodeStates_[DM_MAX_MEMBERS] = {};
    uint32_t nextSequenceNumber_ = 1U;

    // Local autonomous position manager.
    bool localPositionStarted_ = false;
    bool imuAvailable_ = false;
    bool imuWarningLogged_ = false;
    bool imuReferenceValid_ = false;
    float imuReferenceX_ = 0.0F;
    float imuReferenceY_ = 0.0F;
    float imuReferenceZ_ = 0.0F;
    uint32_t lastMotionMs_ = 0U;
    uint32_t localFixMs_ = 0U;
    uint32_t lastGpsSolutionId_ = 0U;
    meshtastic_Position localFix_ = {};
    DmPositionKind localPositionKind_ = DmPositionKind::NoFix;

    // Periodic logs.
    uint32_t lastLocalSampleLogMs_ = 0U;
    bool hasLocalSampleLogTime_ = false;
    uint32_t lastSummaryLogMs_ = 0U;
    bool hasSummaryLogTime_ = false;

    void loadDefaultConfiguration();
    bool findMemberIndex(uint32_t nodeNum, size_t &index) const;

    void refreshMemberState(
        size_t index,
        uint32_t localNodeNum,
        uint32_t now,
        bool absoluteTimeValid,
        uint32_t nowMs);

    void evaluateRadioState(
        DmNodeState &state,
        const meshtastic_NodeInfoLite *node,
        uint32_t now,
        bool absoluteTimeValid,
        uint32_t nowMs) const;

    uint32_t positionAgeSeconds(const DmNodeState &state, uint32_t nowMs) const;
    bool isPositionUsable(const DmNodeState &state, uint32_t nowMs) const;

    // Autonomous GPS/IMU implementation lives in DistanceMonitorPosition.cpp.
    void startLocalPositionManager(uint32_t nowMs);
    void updateLocalPosition(DmNodeState &localState, uint32_t nowMs);
    bool readImuMotion(bool &motionDetected);
    bool readNewValidGpsFix();
    uint32_t currentGpsSolutionId() const;
    void captureGpsFix(uint32_t nowMs);
    void wakeGps();
    void sleepGps();
    uint32_t localFixAgeSeconds(uint32_t nowMs) const;
    void copyLocalPositionToState(DmNodeState &localState, uint32_t nowMs) const;
    void logLocalPositionSample(const DmNodeState &localState, uint32_t nowMs);

    void processRemoteMember(size_t localIndex, size_t remoteIndex, uint32_t nowMs);
    void updateAlarmState(DmNodeState &remoteState);
    void updateAlarmAcknowledgement(DmNodeState &remoteState, uint32_t nowMs) const;
    void logMemberSummary(uint32_t nowMs) const;

    // Packet/protocol implementation lives in DistanceMonitorPackets.cpp.
    bool requestAlive(uint32_t target);
    bool sendAliveResponse(uint32_t target);
    bool requestPosition(uint32_t target);
    bool sendPositionResponse(uint32_t target, const DmNodeState &localState, uint32_t nowMs);
    bool decodePositionResponse(DmNodeState &state, const meshtastic_MeshPacket &mp, uint32_t nowMs) const;
    bool sendPrivateMessage(uint32_t target, DmMessageType type);
    bool sendPrivateMessageWithSequence(uint32_t target, DmMessageType type, uint32_t sequence);
    bool sendSosToMembers(uint32_t localNodeNum);
    uint32_t allocateSequenceNumber();

    bool encodeMessageHeader(
        DmMessageType type,
        uint32_t sequence,
        uint8_t *buffer,
        size_t bufferSize,
        size_t &encodedSize) const;

    bool decodeMessageHeader(
        const uint8_t *buffer,
        size_t bufferSize,
        DmMessageHeader &header) const;

    bool acceptSequence(DmNodeState &state, uint32_t sequence, uint32_t nowMs);
    void updatePacketReceptionState(DmNodeState &state, const meshtastic_MeshPacket &packet, uint32_t nowMs);
    bool isDirectPacket(const meshtastic_MeshPacket &packet) const;
};
