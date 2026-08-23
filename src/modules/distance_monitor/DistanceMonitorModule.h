#pragma once

#include "DistanceMonitorAudio.h"
#include "DistanceMonitorConfig.h"
#include "DistanceMonitorRssi.h"
#include "DistanceMonitorTypes.h"
#include "concurrency/OSThread.h"
#include "mesh/SinglePortModule.h"

class DistanceMonitorModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    DistanceMonitorModule();
    void setup() override;
    bool handleSingleButtonPress();
    bool handleDoubleButtonPress();
    void handleShutdownThresholdReached();
    bool isLocalBase() const;
    uint32_t prepareLocalShutdown();

  protected:
    int32_t runOnce() override;
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    bool wantPacket(const meshtastic_MeshPacket *packet) override;

  private:
    DmRuntimeConfig runtimeConfig_;
    DmNodeState nodeStates_[DM_MAX_MEMBERS] = {};

    // TB = tracker->base direct RSSI. BT = base->tracker RSSI is measured on
    // the tracker and sent in PositionReport.
    DmRssiFilter directInboundRssi_[DM_MAX_MEMBERS] = {};
    DmRssiFilter baseBeaconRssi_;
    DistanceMonitorAudio audio_;

    int statusLedPin_ = -1;
    uint32_t lastLedHeartbeatMs_ = 0U;
    bool hasLedHeartbeatTime_ = false;
    uint32_t ledTransitionMs_ = 0U;
    uint8_t ledEventPhase_ = 0U;

    bool moduleInitialized_ = false;
    uint32_t nextSequenceNumber_ = 1U;
    uint32_t bootMs_ = 0U;
    DmSessionId bootSessionId_ = 0U;

    uint32_t lastBaseBeaconTxMs_ = 0U;
    bool hasBaseBeaconTxTime_ = false;
    uint32_t lastSummaryLogMs_ = 0U;
    bool hasSummaryLogTime_ = false;
    uint32_t lastControlTickMs_ = 0U;
    bool hasControlTickTime_ = false;

    bool forcePositionReport_ = true;
    uint32_t lastPositionReportTxMs_ = 0U;
    bool hasPositionReportTxTime_ = false;
    DmPendingSos pendingSos_;
    uint32_t lastSosTriggerMs_ = 0U;

    bool localPositionStarted_ = false;
    bool imuAvailable_ = false;
    bool imuWarningLogged_ = false;
    bool localMoving_ = true;
    bool gravityReferenceValid_ = false;
    float gravityX_ = 0.0F;
    float gravityY_ = 0.0F;
    float gravityZ_ = 1.0F;
    uint32_t lastMotionMs_ = 0U;
    bool highSpeedSosLatched_ = false;
    uint32_t highSpeedBelowSinceMs_ = 0U;

    // One IMU magnitude ring is used by tracker fall detection.
    // fallBufferHead_ always points to the newest physical sample; logical
    // phase offsets are expressed as sample ages from that dynamic head.
    float fallMagnitudeBuffer_[DM_MAX_ROLL_BUFFER] = {};
    size_t fallBufferHead_ = 0U;
    size_t fallBufferCount_ = 0U;
    uint8_t lastFallPassMask_ = 0U;
    uint32_t lastFallStatsLogMs_ = 0U;
    bool hasFallStatsLogTime_ = false;
    bool localFallDetected_ = false;
    uint32_t localFallDetectedMs_ = 0U;

    uint32_t localFixMs_ = 0U;
    uint32_t lastGpsSolutionId_ = 0U;
    uint32_t lastGpsFunctionalCheckMs_ = 0U;
    uint32_t lastGpsAcceptedMs_ = 0U;
    bool hasGpsAcceptedTime_ = false;
    uint32_t lastGpsObservedSolutionId_ = 0U;
    uint32_t lastGpsProgressMs_ = 0U;
    bool hasGpsProgressTime_ = false;
    uint32_t lastGpsRecoveryMs_ = 0U;
    bool hasGpsRecoveryTime_ = false;
    meshtastic_Position localFix_ = {};
    DmPositionKind localPositionKind_ = DmPositionKind::NoFix;
    uint8_t localAppliedIntervalSec_ = DM_MIN_REPORT_INTERVAL_S;

    void loadDefaultConfiguration();
    void initializeIfNeeded();
    void applyRuntimeProfile();
    bool findMemberIndex(uint32_t nodeNum, size_t &index) const;
    bool findLocalIndex(size_t &index) const;
    bool findBaseIndex(size_t &index) const;
    uint32_t uptimeSeconds(uint32_t nowMs) const;
    uint8_t currentBatteryPercent() const;

    void processBase(size_t localIndex, uint32_t nowMs);
    void processTracker(size_t localIndex, uint32_t nowMs);
    void processPairing(size_t localIndex, uint32_t nowMs);
    void processRemoteMember(size_t localIndex, size_t remoteIndex, uint32_t nowMs);
    void processLinkState(size_t remoteIndex, uint32_t nowMs);
    void processPendingSos(uint32_t nowMs);
    void processNotifications(uint32_t nowMs);
    void updateBaseAudio(size_t localIndex, uint32_t nowMs);

    // Evaluate GPS first. It returns false when either fix is absent or when
    // combined DOP-derived accuracy is too large; caller then uses RSSI LUT.
    void evaluateRemoteDistance(size_t localIndex, size_t remoteIndex, uint32_t nowMs);
    bool evaluateGpsDistance(
        const DmNodeState &localState,
        DmNodeState &remoteState,
        uint32_t nowMs);
    bool evaluateRssiDistance(
        size_t remoteIndex,
        DmNodeState &remoteState,
        uint32_t nowMs);
    void updateDistanceAlertState(DmNodeState &remoteState, float currentRatio);

    DmFaultCause diagnoseRadioLoss(size_t remoteIndex) const;
    bool isRadioFault(DmFaultCause cause) const;
    bool isFaultAudible(const DmNodeState &state, uint32_t nowMs) const;
    uint32_t linkTimeoutMs(uint8_t intervalSec = 0U) const;
    uint32_t rssiBeaconMaxAgeMs() const;
    void logSummary(size_t localIndex, uint32_t nowMs) const;

    void setupStatusLed();
    void serviceStatusLed(uint32_t nowMs);
    void triggerStatusLedDoubleBlink();
    void setStatusLed(bool on);

    // GNSS stays enabled at 0.5 Hz. DOP is not a hard fix rejection anymore:
    // accuracy is carried to the distance layer which decides GPS vs RSSI.
    void startLocalPositionManager(uint32_t nowMs);
    void ensureGpsAlwaysOn(uint32_t nowMs, bool forceDiagnostic = false);
    void sampleLocalImu(DmNodeState &localState, uint32_t nowMs);
    void updateLocalPosition(DmNodeState &localState, uint32_t nowMs);
    bool readImuSample(bool &motionDetected, float &rawNormG);
    bool readNewUsableGpsPosition();
    uint32_t currentGpsSolutionId() const;
    void captureGpsFix(uint32_t nowMs);
    uint32_t localFixAgeSeconds(uint32_t nowMs) const;
    uint32_t freshFixMaxAgeSeconds() const;
    void copyLocalPositionToState(DmNodeState &localState, uint32_t nowMs) const;

    // Push one magnitude into the ring and evaluate the three logical windows
    // once the complete PRE+IMPACT+POST history is available.
    void updateFallDetection(DmNodeState &localState, uint32_t nowMs, float rawNormG);
    size_t fallBufferIndexFromAge(size_t age) const;
    DmFallPhaseStats analyzeFallPhase(size_t startAge, size_t sampleCount) const;
    void logFallAnalysis(
        uint32_t nowMs,
        const DmFallPhaseStats &pre,
        const DmFallPhaseStats &impact,
        const DmFallPhaseStats &post,
        bool prePass,
        bool impactPass,
        bool postPass);
    void handleLocalFallDetected(DmNodeState &localState, uint32_t nowMs);
    void triggerLocalSos(DmSosCause cause);

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
    bool acceptSequence(
        DmNodeState &state,
        uint32_t sequence,
        uint32_t nowMs,
        bool &duplicate);
    bool isDirectPacket(const meshtastic_MeshPacket &packet) const;
    void noteRemoteSession(DmNodeState &sender, DmSessionId sessionId, uint32_t nowMs);
    void clearUnpairedFault(DmNodeState &sender);

    bool sendAliveRequest(uint32_t target);
    bool sendAliveResponse(uint32_t target);
    bool sendPairConfirm(uint32_t target, DmSessionId trackerSessionId, bool playTrackerTone);
    bool sendBaseBeacon();
    bool sendPositionReport(uint32_t target, const DmNodeState &localState, uint32_t nowMs);
    bool sendSetPositionInterval(uint32_t target, DmSessionId trackerSessionId, uint8_t intervalSec);
    bool sendSos(uint32_t target, uint32_t sequence, DmSosCause cause);
    bool sendSosAck(uint32_t target, uint32_t sosSequence, DmSessionId trackerSessionId);
    bool sendNotification(uint32_t target, uint32_t sequence);
    bool sendNotificationAck(uint32_t target, uint32_t notificationSequence, DmSessionId trackerSessionId);
    bool sendShutdownNotice(uint32_t target);
    bool sendPacket(
        uint32_t target,
        DmMessageType type,
        uint32_t sequence,
        const uint8_t *body,
        size_t bodySize,
        bool reliable,
        bool wantAck);

    void handleAliveRequest(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        const DmMessageHeader &header,
        uint32_t nowMs);
    void handleAliveResponse(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        const DmMessageHeader &header,
        uint32_t nowMs);
    void handlePairConfirm(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        const DmMessageHeader &header,
        bool duplicate,
        uint32_t nowMs);
    void handlePositionReport(
        size_t senderIndex,
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        const DmMessageHeader &header,
        uint32_t nowMs);
    void handleSetPositionInterval(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        uint32_t nowMs);
    void handleBaseBeacon(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        uint32_t nowMs);
    void handleSos(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        const DmMessageHeader &header,
        uint32_t nowMs);
    void handleSosAck(const meshtastic_MeshPacket &mp, uint32_t nowMs);
    void handleNotification(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        const DmMessageHeader &header,
        bool duplicate,
        uint32_t nowMs);
    void handleNotificationAck(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        uint32_t nowMs);
    void handleShutdownNotice(
        DmNodeState &sender,
        const meshtastic_MeshPacket &mp,
        uint32_t nowMs);
};

extern DistanceMonitorModule *distanceMonitorModule;
