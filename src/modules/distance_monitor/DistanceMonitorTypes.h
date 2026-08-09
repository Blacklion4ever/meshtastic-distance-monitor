#pragma once

#include "DistanceMonitorConfig.h"

#include <cstddef>
#include <cstdint>

enum class DmRadioState : uint8_t
{
    Unknown = 0,
    Alive,
    Stale,
    Lost,
};

/** Position state decided by the source node itself. */
enum class DmPositionKind : uint8_t
{
    NoFix = 0,
    FreshFix = 1,
    CachedStationary = 2,
};

enum class DmDistanceState : uint8_t
{
    Unknown = 0,
    Local,
    Valid,
    Grace,
};

enum class DmAlarmState : uint8_t
{
    Safe = 0,
    Warning,
    Alarm,
    TrackingLost,
};

enum class DmMessageType : uint8_t
{
    AliveRequest = 1,
    AliveResponse = 2,
    Sos = 3,
    PositionRequest = 4,
    PositionResponse = 5,
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
    float warningRatio = DM_WARNING_RATIO;
    uint32_t freshPositionMaxAgeSeconds = DM_FRESH_POSITION_MAX_AGE_S;
    uint32_t cachedPositionMaxAgeSeconds = DM_CACHED_POSITION_MAX_AGE_S;
    uint32_t maxDop = DM_MAX_DOP;
};

/** Runtime state for one configured member. */
struct DmNodeState
{
    // Identity
    uint32_t nodeNum = 0U;
    bool isLocal = false;
    bool isBase = false;

    // Radio
    uint32_t lastHeardAgeSeconds = 0U;
    bool hasLastHeardAge = false;
    DmRadioState radioState = DmRadioState::Unknown;
    uint32_t lastPacketRxMs = 0U;
    bool hasPacketRxTime = false;
    uint32_t lastAliveRequestMs = 0U;
    bool hasAliveRequestTime = false;

    // Position: source node decides the semantic kind; receiver only tracks age.
    DmPositionKind positionKind = DmPositionKind::NoFix;
    int32_t latitudeI = 0;
    int32_t longitudeI = 0;
    uint32_t positionAgeAtRxSeconds = 0U;
    uint32_t positionRxMs = 0U;

    // Base polling
    uint32_t lastPositionRequestMs = 0U;
    bool hasPositionRequestTime = false;

    // Distance / alarm
    DmDistanceState distanceState = DmDistanceState::Unknown;
    double distanceMeters = 0.0;
    bool hasLastValidDistance = false;
    double lastValidDistanceMeters = 0.0;
    uint32_t lastValidDistanceMs = 0U;
    DmAlarmState alarmState = DmAlarmState::Safe;
    bool alarmAcknowledged = false;
    uint32_t alarmAcknowledgedMs = 0U;

    // Direct-link metrics
    bool radioMetricsValid = false;
    float filteredRssi = 0.0F;
    float rssiTrend = 0.0F;
    bool rssiFilterInitialized = false;

    // Private protocol
    uint32_t lastSeqNum = 0U;
    bool hasLastSeqNum = false;
    uint32_t lastSeqRxMs = 0U;
    bool sosPending = false;
};

struct DmMessageHeader
{
    uint8_t version = 0U;
    DmMessageType type = DmMessageType::AliveRequest;
    uint32_t sequence = 0U;
};
