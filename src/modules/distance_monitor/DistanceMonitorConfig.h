#pragma once

#include <cstddef>
#include <cstdint>

/**
 * Distance Monitor compile-time defaults.
 *
 * These values are intentionally short and use the DM_ prefix so they remain
 * readable throughout the module. They are defaults only: the runtime
 * configuration is mutable and can later be loaded from persistent storage or
 * changed by the companion application without changing the module API.
 */

/** Maximum number of Distance Monitor members supported by one device. */
static constexpr size_t DM_MAX_MEMBERS = 8;

/** Distance Monitor protocol version carried by PRIVATE_APP messages. */
static constexpr uint8_t DM_PROTOCOL_VERSION = 1;

/** Human-readable firmware version for Distance Monitor diagnostics. */
static constexpr const char *DM_FIRMWARE_VERSION = "0.2.0-dev";

/** Main Distance Monitor processing period. */
static constexpr uint32_t DM_TICK_INTERVAL_MS = 5000U;

/**
 * A node is considered radio-stale after this many seconds without being heard.
 * A stale node remains usable, but an ALIVE_REQUEST may be sent.
 */
static constexpr uint32_t DM_LAST_HEARD_STALE_S = 60U;

/**
 * A node is considered radio-lost after this many seconds without being heard.
 * The node remains configured; only its current radio state becomes LOST.
 */
static constexpr uint32_t DM_LAST_HEARD_LOST_S = 10U * 60U;

/** Minimum delay between two ALIVE_REQUEST messages sent to the same member. */
static constexpr uint32_t DM_ALIVE_REQUEST_COOLDOWN_MS = 30U * 1000U;

/**
 * Maximum acceptable age of a GNSS position.
 *
 * With a 5-second module tick, a base starts requesting a fresh position as
 * soon as the last usable position becomes older than 15 seconds.
 */
static constexpr uint32_t DM_POSITION_MAX_AGE_S = 15U;

/** Minimum delay between two POSITION_APP requests sent to the same member. */
static constexpr uint32_t DM_POSITION_REQUEST_COOLDOWN_MS = 15U * 1000U;

/** Minimum accepted GNSS fix type. 2 means a 2D fix or better. */
static constexpr uint32_t DM_MIN_FIX_TYPE = 2U;

/**
 * Maximum accepted HDOP in Meshtastic 1/100 units.
 * 300 means HDOP <= 3.00.
 */
static constexpr uint32_t DM_MAX_HDOP = 300U;

/** Default maximum monitoring distance in meters. */
static constexpr float DM_MAX_DISTANCE_M = 100.0F;

/** Warning zone starts at 80 percent of the configured maximum distance. */
static constexpr float DM_WARNING_RATIO = 0.80F;

/**
 * Standard grace period after losing a usable GNSS position while a recent,
 * previously valid distance is still available.
 */
static constexpr uint32_t DM_DISTANCE_GRACE_MS = 20U * 1000U;

/**
 * Longer grace period when the last valid distance was clearly close.
 * This handles short GNSS dropouts without converting them into false alarms.
 */
static constexpr uint32_t DM_CLOSE_DISTANCE_GRACE_MS = 60U * 1000U;

/** A distance below this fraction of the maximum is considered clearly close. */
static constexpr float DM_CLOSE_DISTANCE_RATIO = 0.20F;

/** Exponential filter coefficient used for direct-link RSSI samples. */
static constexpr float DM_RSSI_FILTER_ALPHA = 0.20F;

/** Alarm silence period after a local acknowledgement on the base. */
static constexpr uint32_t DM_ALARM_REARM_MS = 60U * 1000U;

/**
 * Exact duplicate application sequences received inside this window are ignored.
 * A different sequence is accepted immediately, including after a sender reboot.
 */
static constexpr uint32_t DM_SEQUENCE_DUPLICATE_WINDOW_MS = 5U * 60U * 1000U;

/** First byte of the private Distance Monitor protocol signature: ASCII 'D'. */
static constexpr uint8_t DM_PROTOCOL_MAGIC_0 = 0x44U;

/** Second byte of the private Distance Monitor protocol signature: ASCII 'M'. */
static constexpr uint8_t DM_PROTOCOL_MAGIC_1 = 0x4DU;

/** Size of the fixed Distance Monitor private-message header. */
static constexpr size_t DM_PROTOCOL_HEADER_SIZE = 8U;

/**
 * Small compile-time representation used only to seed the mutable runtime
 * member list on a fresh/default configuration.
 */
struct DmDefaultMember
{
    /** Meshtastic node number of the member. */
    uint32_t nodeNum;

    /** True when this member acts as a monitoring base. */
    bool isBase;
};

/**
 * Default members compiled into the firmware.
 *
 * The runtime copy is intentionally mutable. A future persistence layer can
 * replace these defaults with values written by the companion application.
 */
static constexpr DmDefaultMember DM_DEFAULT_MEMBERS[] = {
    {0x8B3A4A14U, true},
    {0xE87C9792U, false},
};

/** Number of entries in DM_DEFAULT_MEMBERS. */
static constexpr size_t DM_DEFAULT_MEMBER_COUNT =
    sizeof(DM_DEFAULT_MEMBERS) / sizeof(DM_DEFAULT_MEMBERS[0]);

static_assert(
    DM_DEFAULT_MEMBER_COUNT <= DM_MAX_MEMBERS,
    "DM_DEFAULT_MEMBERS exceeds DM_MAX_MEMBERS");
