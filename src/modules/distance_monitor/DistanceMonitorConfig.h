#pragma once

#include <cstddef>
#include <cstdint>

static constexpr size_t DM_MAX_MEMBERS = 8;

/** Protocol v4: PositionResponse = kind + age + latitude + longitude only. */
static constexpr uint8_t DM_PROTOCOL_VERSION = 4;
static constexpr const char *DM_FIRMWARE_VERSION = "0.4.2-dev";

/** Fast local loop for IMU/GPS state; user-facing summary remains slower. */
static constexpr uint32_t DM_TICK_INTERVAL_MS = 1000U;
static constexpr uint32_t DM_SUMMARY_INTERVAL_MS = 5000U;

/** Base position sampling/polling cadence. */
static constexpr uint32_t DM_POSITION_REQUEST_INTERVAL_MS = 15U * 1000U;

/** A moving FreshFix older than this is no longer trusted. */
static constexpr uint32_t DM_FRESH_POSITION_MAX_AGE_S = 30U;

/** Base-side maximum age accepted for CachedStationary. */
static constexpr uint32_t DM_CACHED_POSITION_MAX_AGE_S = 5U * 60U;

/** Quiet IMU period required before sleeping the GNSS. */
static constexpr uint32_t DM_STATIONARY_CONFIRM_MS = 10U * 1000U;

/** Acceleration-vector change considered movement. */
static constexpr float DM_IMU_MOTION_THRESHOLD_G = 0.08F;

/** Native Meshtastic GNSS cadence while active. */
static constexpr uint32_t DM_GPS_ACTIVE_UPDATE_INTERVAL_S = 5U;

/** Keep native periodic wakeups out of the way while stationary. */
static constexpr uint32_t DM_GPS_SLEEP_UPDATE_INTERVAL_S = 24U * 60U * 60U;

static constexpr uint32_t DM_LAST_HEARD_STALE_S = 60U;
static constexpr uint32_t DM_LAST_HEARD_LOST_S = 10U * 60U;
static constexpr uint32_t DM_ALIVE_REQUEST_COOLDOWN_MS = 30U * 1000U;

static constexpr float DM_MAX_DISTANCE_M = 100.0F;
static constexpr float DM_WARNING_RATIO = 0.80F;
static constexpr uint32_t DM_DISTANCE_GRACE_MS = 20U * 1000U;
static constexpr uint32_t DM_CLOSE_DISTANCE_GRACE_MS = 60U * 1000U;
static constexpr float DM_CLOSE_DISTANCE_RATIO = 0.20F;
static constexpr float DM_RSSI_FILTER_ALPHA = 0.20F;
static constexpr uint32_t DM_ALARM_REARM_MS = 60U * 1000U;
static constexpr uint32_t DM_SEQUENCE_DUPLICATE_WINDOW_MS = 5U * 60U * 1000U;

static constexpr uint8_t DM_PROTOCOL_MAGIC_0 = 0x44U;
static constexpr uint8_t DM_PROTOCOL_MAGIC_1 = 0x4DU;
static constexpr size_t DM_PROTOCOL_HEADER_SIZE = 8U;

/** kind:u8 + age:u32 + lat:i32 + lon:i32 */
static constexpr size_t DM_POSITION_RESPONSE_SIZE = DM_PROTOCOL_HEADER_SIZE + 13U;

static constexpr uint32_t DM_MIN_VALID_UNIX_TIME_S = 1704067200U;
static constexpr uint32_t DM_MAX_DOP = 300U;

struct DmDefaultMember
{
    uint32_t nodeNum;
    bool isBase;
};

static constexpr DmDefaultMember DM_DEFAULT_MEMBERS[] = {
    {0x8B3A4A14U, true},
    {0xE87C9792U, false},
};

static constexpr size_t DM_DEFAULT_MEMBER_COUNT =
    sizeof(DM_DEFAULT_MEMBERS) / sizeof(DM_DEFAULT_MEMBERS[0]);

static_assert(DM_DEFAULT_MEMBER_COUNT <= DM_MAX_MEMBERS,
              "DM_DEFAULT_MEMBERS exceeds DM_MAX_MEMBERS");
