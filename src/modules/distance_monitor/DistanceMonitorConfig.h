#pragma once

#include <cstddef>
#include <cstdint>

static constexpr size_t DM_MAX_MEMBERS = 2U;

static constexpr uint8_t DM_PROTOCOL_VERSION = 5U;
static constexpr const char *DM_FIRMWARE_VERSION = "1.0.0-beta3-search-beacon";

static constexpr uint32_t DM_TICK_INTERVAL_MS = 40U;
static constexpr uint32_t DM_CONTROL_INTERVAL_MS = 1000U;
static constexpr uint32_t DM_SUMMARY_INTERVAL_MS = 5000U;

static constexpr float DM_MAX_DISTANCE_M = 100.0F;
static constexpr float DM_DISTANCE_ALERT_START_RATIO = 0.80F;

static constexpr float DM_DESIGN_RELATIVE_SPEED_MPS = 7.0F / 3.6F;
static constexpr uint8_t DM_MIN_SAMPLES_PER_DMAX = 3U;
static constexpr uint8_t DM_MIN_REPORT_INTERVAL_S = 9U;
static constexpr uint8_t DM_MAX_REPORT_INTERVAL_CAP_S = 20U;

static constexpr uint32_t DM_BASE_BEACON_INTERVAL_MS = 10U * 1000U;
static constexpr uint32_t DM_LINK_TIMEOUT_MARGIN_MS = 5U * 1000U;
static constexpr uint32_t DM_LINK_PROBE_INTERVAL_MS = 5U * 1000U;
static constexpr uint32_t DM_COM_SATURATION_SILENT_MS = 0U;

static constexpr uint32_t DM_PAIR_CONFIRM_DELAY_MS = 15U * 1000U;
static constexpr uint32_t DM_HANDSHAKE_RETRY_MS = 20U * 1000U;
static constexpr uint32_t DM_PAIR_REMOTE_BOOT_WINDOW_S = 30U;

static constexpr uint32_t DM_DISTANCE_SNOOZE_MS = 60U * 1000U;
static constexpr uint32_t DM_FAULT_SNOOZE_MS = 60U * 1000U;
static constexpr uint32_t DM_COM_SATURATION_SNOOZE_MS = 10U * 60U * 1000U;
static constexpr uint32_t DM_FAULT_AUDIO_PERIOD_MS = 30U * 1000U;

static constexpr uint32_t DM_RESEND_TIMEOUT_MS = 5U * 1000U;
static constexpr uint32_t DM_NOTIFICATION_ACK_WINDOW_MS = 10U * 1000U;
static constexpr uint32_t DM_SHUTDOWN_TX_GRACE_MS = 500U;

static constexpr uint32_t DM_STATIONARY_CONFIRM_MS = 10U * 1000U;
static constexpr float DM_IMU_MOTION_THRESHOLD_G = 0.08F;
static constexpr float DM_IMU_GRAVITY_ALPHA = 0.01F;

static constexpr float DM_VEHICLE_ACCEL_THRESHOLD_MPS2 = 0.25F;
static constexpr uint32_t DM_VEHICLE_ACCEL_CONFIRM_MS = 1200U;
static constexpr uint32_t DM_HIGH_SPEED_GPS_INTERVAL_S = 1U;
static constexpr float DM_HIGH_SPEED_THRESHOLD_KMH = 20.0F;
static constexpr uint32_t DM_HIGH_SPEED_WATCH_HOLD_MS = 60U * 1000U;
static constexpr uint32_t DM_HIGH_SPEED_CLEAR_MS = 10U * 1000U;

static constexpr float DM_FALL_FREEFALL_THRESHOLD_G = 0.65F;
static constexpr float DM_FALL_IMPACT_THRESHOLD_G = 1.55F;
static constexpr uint32_t DM_FALL_IMPACT_WINDOW_MS = 900U;
static constexpr uint32_t DM_SOS_REARM_MS = 10U * 1000U;

static constexpr uint32_t DM_GPS_SLEEP_UPDATE_INTERVAL_S = 24U * 60U * 60U;
static constexpr uint32_t DM_FRESH_FIX_EXTRA_GRACE_S = 3U;
static constexpr uint32_t DM_MAX_DOP = 300U;

static constexpr float DM_RSSI_FILTER_ALPHA = 0.25F;
static constexpr float DM_RSSI_TREND_ALPHA = 0.25F;
static constexpr uint8_t DM_RSSI_MIN_SAMPLES = 3U;
static constexpr uint32_t DM_RSSI_VALID_BEACON_MULTIPLIER = 3U;
static constexpr float DM_RSSI_MIN_STD_DB = 3.0F;
static constexpr float DM_RSSI_BOOTSTRAP_STD_DB = 10.0F;

static constexpr size_t DM_RSSI_CALIBRATION_BIN_COUNT = 8U;
static constexpr float DM_RSSI_OPEN_BIN_MAX_M = 10000.0F;
static constexpr uint32_t DM_RSSI_TABLE_MIN_BIN_SAMPLES = 4U;
static constexpr float DM_RSSI_ALERT_PROBABILITY = 0.80F;
static constexpr uint32_t DM_RSSI_STATIONARY_FRESHNESS_MS = 30U * 1000U;
static constexpr uint32_t DM_RSSI_MOVING_FRESHNESS_MS = 12U * 1000U;

static constexpr uint32_t DM_RSSI_CHECKPOINT_INTERVAL_MS = 15U * 60U * 1000U;
static constexpr uint32_t DM_RSSI_CHECKPOINT_SAMPLE_DELTA = 32U;

static constexpr uint32_t DM_SEARCH_LONG_PRESS_MS = 3000U;
static constexpr uint32_t DM_SEARCH_LONG_LONG_PRESS_MS = 7000U;
static constexpr uint32_t DM_SEARCH_BEACON_INTERVAL_MS = 3000U;
static constexpr uint32_t DM_SEARCH_BEACON_TIMEOUT_MS = 4200U;
static constexpr uint32_t DM_SEARCH_START_REFRESH_MS = 9000U;
static constexpr uint32_t DM_SEARCH_TRACKER_LEASE_MS = 13000U;
static constexpr float DM_SEARCH_FILTER_ALPHA = 1.0F;
static constexpr float DM_SEARCH_TREND_CLAMP_DB_PER_S = 6.0F;
static constexpr uint32_t DM_SEARCH_TREND_INTERPOLATION_MS = 1500U;
static constexpr float DM_SEARCH_TREND_INTERPOLATION_CLAMP_DB = 6.0F;
static constexpr uint32_t DM_SEARCH_PULSE_INTERVAL_MS = 750U;
static constexpr float DM_SEARCH_RSSI_BEST_DBM = -12.0F;
static constexpr float DM_SEARCH_RSSI_WORST_DBM = -74.0F;
static constexpr uint16_t DM_SEARCH_DETECTION_MIN_HZ = 1450U;
static constexpr uint16_t DM_SEARCH_DETECTION_NEAR_HZ = 2390U;
static constexpr uint16_t DM_SEARCH_DETECTION_DURATION_MIN_MS = 25U;
static constexpr uint16_t DM_SEARCH_DETECTION_DURATION_MAX_MS = 85U;
static constexpr uint16_t DM_SEARCH_DETECTION_DELAY_FAR_MS = 180U;
static constexpr uint16_t DM_SEARCH_DETECTION_DELAY_NEAR_MS = 160U;
static constexpr float DM_SEARCH_PATHLOSS_SPAN_DB = 60.0F;

static constexpr uint8_t DM_LOW_BATTERY_PERCENT = 10U;
static constexpr uint8_t DM_BATTERY_UNKNOWN = 255U;
static constexpr float DM_RANGE_LOSS_MIN_RATIO = 0.50F;
static constexpr float DM_COM_SATURATION_MAX_RATIO = 0.10F;
static constexpr float DM_RSSI_GOOD_DBM = -75.0F;
static constexpr float DM_RSSI_DEGRADING_TREND_DB_PER_S = -0.15F;
static constexpr float DM_RSSI_STABLE_TREND_DB_PER_S = -0.05F;

static constexpr uint16_t DM_BIP_FREQ_START_HZ = 2300U;
static constexpr uint16_t DM_BIP_FREQ_END_HZ = 3100U;
static constexpr uint16_t DM_BOP_FREQ_HZ = 1900U;
static constexpr uint16_t DM_BIP_STAGE_1_MS = 70U;
static constexpr uint16_t DM_BIP_STAGE_2_MS = 40U;
static constexpr uint16_t DM_BOP_DURATION_MS = 220U;

static constexpr uint8_t DM_PROTOCOL_MAGIC_0 = 0x44U;
static constexpr uint8_t DM_PROTOCOL_MAGIC_1 = 0x4DU;
static constexpr size_t DM_PROTOCOL_HEADER_SIZE = 8U;

static constexpr size_t DM_ALIVE_SIZE = DM_PROTOCOL_HEADER_SIZE + 8U;
static constexpr size_t DM_PAIR_CONFIRM_SIZE = DM_PROTOCOL_HEADER_SIZE + 9U;
static constexpr size_t DM_POSITION_REPORT_SIZE = DM_PROTOCOL_HEADER_SIZE + 23U;
static constexpr size_t DM_SET_INTERVAL_SIZE = DM_PROTOCOL_HEADER_SIZE + 5U;
static constexpr size_t DM_BASE_BEACON_SIZE = DM_PROTOCOL_HEADER_SIZE + 4U;
static constexpr size_t DM_SOS_SIZE = DM_PROTOCOL_HEADER_SIZE + 5U;
static constexpr size_t DM_SOS_ACK_SIZE = DM_PROTOCOL_HEADER_SIZE + 8U;
static constexpr size_t DM_NOTIFICATION_SIZE = DM_PROTOCOL_HEADER_SIZE + 4U;
static constexpr size_t DM_NOTIFICATION_ACK_SIZE = DM_PROTOCOL_HEADER_SIZE + 8U;
static constexpr size_t DM_SHUTDOWN_NOTICE_SIZE = DM_PROTOCOL_HEADER_SIZE + 4U;

static constexpr uint32_t DM_SEQUENCE_DUPLICATE_WINDOW_MS = 5U * 60U * 1000U;
static constexpr size_t DM_RECENT_SEQUENCE_COUNT = 8U;
static constexpr uint32_t DM_MIN_VALID_UNIX_TIME_S = 1704067200U;

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
