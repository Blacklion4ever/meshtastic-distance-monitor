#pragma once

#include "DistanceMonitorConfig.h"

#include <cstddef>
#include <cstdint>

enum class DmRadioState : uint8_t
{
    Unknown = 0,
    Pairing,
    Alive,
    Suspected,
    Lost,
};

enum class DmPositionKind : uint8_t
{
    NoFix = 0,
    FreshFix = 1,
    CachedStationary = 2,
};

enum class DmDistanceSource : uint8_t
{
    None = 0,
    Gps,
    Rssi,
};

enum class DmDistanceBand : uint8_t
{
    Unknown = 0,
    Near,
    Mid,
    Warning,
    Beyond,
};

enum class DmFaultCause : uint8_t
{
    None = 0,
    RadioLowBattery,
    RadioRangeLoss,
    RadioComSaturation,
    RadioUnexpectedLoss,
    RemoteShutdown,
    DistanceEstimateUnavailable,
};

enum class DmSosCause : uint8_t
{
    ManualButton = 1,
    FallDetected = 2,
    HighSpeedMovement = 3,
};

enum class DmMessageType : uint8_t
{
    AliveRequest = 1,
    AliveResponse = 2,
    PairConfirm = 3,
    PositionReport = 4,
    SetPositionInterval = 5,
    BaseBeacon = 6,
    Sos = 7,
    SosAck = 8,
    Notification = 9,
    NotificationAck = 10,
    ShutdownNotice = 11,
};

enum DmPositionReportFlags : uint8_t
{
    DM_POSITION_FLAG_MOVING = 1U << 0U,
    DM_POSITION_FLAG_BASE_RSSI_VALID = 1U << 1U,
    DM_POSITION_FLAG_PAIRED = 1U << 2U,
};

enum DmPairConfirmFlags : uint8_t
{
    DM_PAIR_FLAG_PLAY_TRACKER_TONE = 1U << 0U,
};

struct DmMemberConfig
{
    uint32_t nodeNum = 0U;
    bool isBase = false;
};

struct DmRuntimeConfig
{
    size_t memberCount = 0U;
    DmMemberConfig members[DM_MAX_MEMBERS] = {};
    float maxDistanceMeters = DM_MAX_DISTANCE_M;
    uint32_t maxDop = DM_MAX_DOP;
};

struct DmMessageHeader
{
    uint8_t version = 0U;
    DmMessageType type = DmMessageType::AliveRequest;
    uint32_t sequence = 0U;
};

struct DmRecentSequence
{
    uint32_t sequence = 0U;
    uint32_t receivedMs = 0U;
};

struct DmDistanceBandEstimate
{
    DmDistanceBand band = DmDistanceBand::Unknown;
    float confidence = 0.0F;
    float fusedRssiDbm = 0.0F;
    float fusedTrendDbPerSec = 0.0F;
    float probabilities[4] = {};
};

struct DmPendingSos
{
    bool active = false;
    uint32_t targetNode = 0U;
    uint32_t sequence = 0U;
    DmSosCause cause = DmSosCause::ManualButton;
    uint32_t lastTxMs = 0U;
};

struct DmNodeState
{
    uint32_t nodeNum = 0U;
    bool isLocal = false;
    bool isBase = false;

    uint32_t remoteSessionId = 0U;
    uint32_t remoteUptimeSeconds = 0U;
    bool hasRemoteUptime = false;
    bool hasRemoteSession = false;

    bool paired = false;
    uint32_t pairedLocalSessionId = 0U;
    uint32_t pairedRemoteSessionId = 0U;
    uint32_t lastHandshakeTxMs = 0U;
    bool hasHandshakeTxTime = false;

    uint32_t lastAnyPacketRxMs = 0U;
    bool hasAnyPacketRxTime = false;

    uint32_t lastPositionReportRxMs = 0U;
    uint32_t lastPositionReportSequence = 0U;
    uint32_t lastIntervalEvaluationSequence = 0U;
    bool hasPositionReportRxTime = false;
    DmRadioState radioState = DmRadioState::Unknown;

    DmFaultCause faultCause = DmFaultCause::None;
    uint32_t faultStartedMs = 0U;
    uint32_t faultSnoozedUntilMs = 0U;
    uint32_t comSaturationSilentUntilMs = 0U;
    uint32_t lastFaultAudioMs = 0U;
    bool hasFaultAudioTime = false;

    bool shutdownNoticeReceived = false;
    uint32_t shutdownNoticeMs = 0U;

    DmPositionKind positionKind = DmPositionKind::NoFix;
    int32_t latitudeI = 0;
    int32_t longitudeI = 0;
    uint32_t positionAgeAtRxSeconds = 0U;
    uint32_t positionRxMs = 0U;

    uint8_t appliedIntervalSec = 0U;
    uint8_t desiredIntervalSec = 0U;
    uint8_t batteryPercent = 0U;
    bool moving = false;

    bool remoteBaseRssiValid = false;
    float remoteBaseRssiMeanDbm = 0.0F;
    float remoteBaseRssiStdDb = 0.0F;
    float remoteBaseRssiTrendDbPerSec = 0.0F;

    DmDistanceSource distanceSource = DmDistanceSource::None;
    double distanceMeters = 0.0;
    float distanceRatio = 0.0F;
    bool hasLastValidDistance = false;
    double lastValidDistanceMeters = 0.0;
    float lastValidDistanceRatio = 0.0F;
    uint32_t lastValidDistanceMs = 0U;

    DmDistanceBandEstimate rssiEstimate = {};
    bool distanceEstimateFault = false;

    float currentDistanceAlertRatio = 0.0F;
    float activeDistanceAlertRatio = 0.0F;
    uint32_t distanceSnoozedUntilMs = 0U;
    uint32_t lastDistanceBipMs = 0U;
    bool hasDistanceBipTime = false;

    bool criticalDistanceLatch = false;
    float criticalDistanceLatchRatio = 0.0F;

    bool sosActive = false;
    uint32_t activeSosSequence = 0U;
    DmSosCause activeSosCause = DmSosCause::ManualButton;

    uint32_t pendingNotificationSequence = 0U;
    uint32_t pendingNotificationSinceMs = 0U;
    bool notificationAcked = false;

    DmRecentSequence recentSequences[DM_RECENT_SEQUENCE_COUNT] = {};
};
