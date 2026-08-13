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
    bool handleSearchToggle();
    bool isLocalBase() const;

    uint32_t prepareLocalShutdown();

protected:
    int32_t runOnce() override;
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    bool wantPacket(const meshtastic_MeshPacket *packet) override;

private:
    DmRuntimeConfig runtimeConfig_;
    DmNodeState nodeStates_[DM_MAX_MEMBERS] = {};
    DmRssiFilter directInboundRssi_[DM_MAX_MEMBERS] = {};
    DmRssiCalibrationProfile rssiProfiles_[DM_MAX_MEMBERS] = {};
    DmRssiFilter baseBeaconRssi_;
    DistanceMonitorAudio audio_;
    bool moduleInitialized_ = false;
    uint32_t nextSequenceNumber_ = 1U;
    uint32_t bootMs_ = 0U;
    uint32_t bootSessionId_ = 0U;

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
    uint32_t vehicleAccelAboveSinceMs_ = 0U;
    uint32_t highSpeedWatchUntilMs_ = 0U;
    bool highSpeedSosLatched_ = false;
    uint32_t highSpeedBelowSinceMs_ = 0U;

    bool fallFreefallArmed_ = false;
    uint32_t fallFreefallMs_ = 0U;
    uint32_t localFixMs_ = 0U;
    uint32_t lastGpsSolutionId_ = 0U;
    uint32_t lastGpsNoFixDiagMs_ = 0U;
    bool hasGpsNoFixDiagTime_ = false;
    meshtastic_Position localFix_ = {};
    DmPositionKind localPositionKind_ = DmPositionKind::NoFix;
    // localAppliedIntervalSec_ remains the Distance Monitor position-report
    // interval used by the wire protocol. It no longer drives GNSS cadence.
    uint8_t localAppliedIntervalSec_ = DM_MIN_REPORT_INTERVAL_S;

    // Kept for source compatibility with existing packet/base-demand code.
    // DistanceMonitorPosition.cpp ignores this value for GNSS power/cadence;
    // GNSS is fixed to ALWAYS_ON at 1 Hz.
    uint8_t localDesiredGpsIntervalSec_ = 1U;
    bool gpsSleeping_ = false;

    bool searchModeActive_ = false;
    size_t searchTargetIndex_ = DM_MAX_MEMBERS;
    uint32_t lastSearchPulseMs_ = 0U;
    bool hasSearchPulseTime_ = false;

    bool rssiCalibrationLoaded_ = false;
    bool rssiCalibrationDirty_[DM_MAX_MEMBERS] = {};
    uint32_t rssiCalibrationGeneration_[DM_MAX_MEMBERS] = {};
    uint32_t rssiCalibrationSavedSamples_[DM_MAX_MEMBERS] = {};
    uint32_t lastCalibrationSequence_[DM_MAX_MEMBERS] = {};

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
    void processSearchMode(size_t localIndex, uint32_t nowMs);
    void evaluateRemoteDistance(size_t localIndex, size_t remoteIndex, uint32_t nowMs);
    bool evaluateGpsDistance(
        const DmNodeState &localState,
        DmNodeState &remoteState,
        uint32_t nowMs);
    bool evaluateRssiDistance(
        const DmNodeState &localState,
        size_t remoteIndex,
        DmNodeState &remoteState,
        uint32_t nowMs);
    void updateDistanceAlertState(DmNodeState &remoteState, float currentRatio);
    void updateRssiCalibration(
        size_t remoteIndex,
        DmNodeState &remoteState,
        float distanceMeters,
        uint32_t nowMs);
    DmRssiTableEstimate currentRssiEstimate(
        size_t remoteIndex,
        const DmNodeState &remoteState,
        const DmNodeState &localState,
        uint32_t nowMs) const;
    void loadRssiCalibrationIfNeeded(size_t localIndex);
    bool loadRssiCalibrationProfile(size_t memberIndex);
    bool saveRssiCalibrationProfile(size_t memberIndex);
    void saveAllRssiCalibration(bool force);
    void logRssiCalibrationTable(size_t memberIndex) const;

    DmFaultCause diagnoseRadioLoss(size_t remoteIndex) const;
    bool isRadioFault(DmFaultCause cause) const;
    bool isFaultAudible(const DmNodeState &state, uint32_t nowMs) const;
    uint32_t linkTimeoutMs() const;
    uint32_t rssiBeaconMaxAgeMs() const;

    void updateBaseGpsDemand(size_t localIndex);
    void logSummary(size_t localIndex, uint32_t nowMs) const;
    void startLocalPositionManager(uint32_t nowMs);
    void sampleLocalImu(DmNodeState &localState, uint32_t nowMs);
    void updateLocalPosition(DmNodeState &localState, uint32_t nowMs);
    bool readImuSample(
        bool &motionDetected,
        float &dynamicAccelerationMps2,
        float &rawNormG);
    bool readNewValidGpsFix();
    uint32_t currentGpsSolutionId() const;
    void captureGpsFix(uint32_t nowMs);
    // Compatibility helpers. GNSS policy is ALWAYS_ON; interval arguments from
    // legacy callers are ignored and sleepGps() is a deliberate no-op.
    void wakeGps(uint8_t updateIntervalSec);
    void sleepGps();
    void applyGpsInterval(uint8_t updateIntervalSec);
    uint32_t localFixAgeSeconds(uint32_t nowMs) const;
    uint32_t freshFixMaxAgeSeconds() const;
    void copyLocalPositionToState(DmNodeState &localState, uint32_t nowMs) const;
    void updateHighSpeedDetection(uint32_t nowMs, float dynamicAccelerationMps2);
    void updateFallDetection(uint32_t nowMs, float rawNormG);
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
    void noteRemoteSession(DmNodeState &sender, uint32_t sessionId, uint32_t nowMs);
    void clearUnpairedFault(DmNodeState &sender);

    bool sendAliveRequest(uint32_t target);
    bool sendAliveResponse(uint32_t target);
    bool sendPairConfirm(
        uint32_t target,
        uint32_t trackerSessionId,
        bool playTrackerTone);
    bool sendBaseBeacon();
    bool sendPositionReport(uint32_t target, const DmNodeState &localState, uint32_t nowMs);
    bool sendSetPositionInterval(
        uint32_t target,
        uint32_t trackerSessionId,
        uint8_t intervalSec);
    bool sendSos(
        uint32_t target,
        uint32_t sequence,
        DmSosCause cause);
    bool sendSosAck(
        uint32_t target,
        uint32_t sosSequence,
        uint32_t trackerSessionId);
    bool sendNotification(
        uint32_t target,
        uint32_t sequence);
    bool sendNotificationAck(
        uint32_t target,
        uint32_t notificationSequence,
        uint32_t trackerSessionId);
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
    void handleSosAck(
        const meshtastic_MeshPacket &mp,
        uint32_t nowMs);
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