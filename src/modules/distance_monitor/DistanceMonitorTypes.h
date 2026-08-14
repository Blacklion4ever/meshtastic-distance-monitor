#pragma once

#include "DistanceMonitorConfig.h"
#include <cstddef>
#include <cstdint>

using DmSessionId = uint16_t;

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
};

enum class DmDistanceSource : uint8_t
{
    None = 0,
    Gps,
    Rssi,
};

// Unknown plus four operational RSSI/distance bands.
enum class DmDistanceBand : uint8_t
{
    Unknown = 0,
    Near,
    Medium,
    Warning,
    VeryFar,
};

enum class DmFaultCause : uint8_t
{
    None = 0,
    RadioLowBattery,
    RadioRangeLoss,
    RadioComSaturation,
    RadioUnexpectedLoss,
    RemoteShutdown,
    Unpaired,
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

// Position kind is folded into the flags to save one byte on every report.
enum DmPositionReportFlags : uint8_t
{
    DM_POSITION_FLAG_FIX = 1U << 0U,
    DM_POSITION_FLAG_MOVING = 1U << 1U,
    DM_POSITION_FLAG_BASE_RSSI_VALID = 1U << 2U,
    DM_POSITION_FLAG_PAIRED = 1U << 3U,
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
    float bestRssiDbm = 0.0F;
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
    bool isBase = false;

    DmSessionId remoteSessionId = 0U;
    uint32_t remoteUptimeSeconds = 0U;
    bool hasRemoteUptime = false;
    bool hasRemoteSession = false;
    bool paired = false;
    bool everPaired = false;
    DmSessionId pairedLocalSessionId = 0U;
    DmSessionId pairedRemoteSessionId = 0U;
    uint32_t lastHandshakeTxMs = 0U;
    bool hasHandshakeTxTime = false;

    uint32_t lastPositionReportRxMs = 0U;
    uint32_t lastPositionReportSequence = 0U;
    uint32_t lastIntervalEvaluationSequence = 0U;
    bool hasPositionReportRxTime = false;

    DmRadioState radioState = DmRadioState::Unknown;
    DmFaultCause faultCause = DmFaultCause::None;
    uint32_t faultSnoozedUntilMs = 0U;
    uint32_t comSaturationSilentUntilMs = 0U;
    uint32_t lastFaultAudioMs = 0U;
    bool hasFaultAudioTime = false;
    bool shutdownNoticeReceived = false;

    DmPositionKind positionKind = DmPositionKind::NoFix;
    int32_t latitudeI = 0;
    int32_t longitudeI = 0;
    // Best non-zero DOP (min(PDOP,HDOP)), in Meshtastic hundredths.
    uint16_t positionDop = 0U;
    double positionAccuracyMeters = 0.0;

    uint8_t appliedIntervalSec = 0U;
    uint8_t batteryPercent = DM_BATTERY_UNKNOWN;
    bool moving = false;

    // Tracker-measured base beacon RSSI (BT direction).
    bool remoteBaseRssiValid = false;
    float remoteBaseRssiDbm = 0.0F;

    DmDistanceSource distanceSource = DmDistanceSource::None;
    // distanceMeters is the safety value used by alarms. For GPS this is the
    // minimum plausible distance after subtracting combined accuracy.
    double distanceMeters = 0.0;
    double rawDistanceMeters = 0.0;
    double combinedAccuracyMeters = 0.0;
    float distanceRatio = 0.0F;
    bool hasLastValidDistance = false;
    double lastValidDistanceMeters = 0.0;
    float lastValidDistanceRatio = 0.0F;
    uint32_t lastValidDistanceMs = 0U;

    DmDistanceBandEstimate rssiEstimate = {};
    float currentDistanceAlertRatio = 0.0F;
    float activeDistanceAlertRatio = 0.0F;
    uint32_t distanceSnoozedUntilMs = 0U;
    uint32_t lastDistanceBipMs = 0U;
    bool hasDistanceBipTime = false;
    bool criticalDistanceLatch = false;
    float criticalDistanceLatchRatio = 0.0F;

    bool sosActive = false;
    DmSosCause activeSosCause = DmSosCause::ManualButton;
    uint32_t pendingNotificationSequence = 0U;
    uint32_t pendingNotificationSinceMs = 0U;
    bool notificationAcked = false;

    DmRecentSequence recentSequences[DM_RECENT_SEQUENCE_COUNT] = {};
};
