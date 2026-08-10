#pragma once

#include <cstddef>
#include <cstdint>

static constexpr size_t DM_MAX_MEMBERS = 2U;

static constexpr uint8_t DM_PROTOCOL_VERSION = 5U;
static constexpr const char *DM_FIRMWARE_VERSION = "1.0.0-beta2";

static constexpr uint32_t DM_TICK_INTERVAL_MS = 1000U;
static constexpr uint32_t DM_SUMMARY_INTERVAL_MS = 5000U;

static constexpr float DM_MAX_DISTANCE_M = 100.0F;
static constexpr float DM_DISTANCE_ALERT_START_RATIO = 0.80F;

static constexpr float DM_DESIGN_RELATIVE_SPEED_MPS = 7.0F / 3.6F;
static constexpr uint8_t DM_MIN_SAMPLES_PER_DMAX = 3U;
static constexpr uint8_t DM_MIN_REPORT_INTERVAL_S = 9U;
static constexpr uint8_t DM_MAX_REPORT_INTERVAL_CAP_S = 20U;

static constexpr uint32_t DM_BASE_BEACON_INTERVAL_MS = 10U * 1000U;
static constexpr uint32_t DM_LINK_TIMEOUT_MARGIN_MS = 5U * 1000U;
static constexpr uint32_t DM_COM_SATURATION_SILENT_MS = 2U * 60U * 1000U;

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
static constexpr float DM_IMU_GRAVITY_ALPHA = 0.05F;

static constexpr float DM_VEHICLE_ACCEL_THRESHOLD_MPS2 = 0.25F;
static constexpr uint8_t DM_VEHICLE_ACCEL_CONFIRM_SAMPLES = 2U;
static constexpr uint32_t DM_HIGH_SPEED_GPS_INTERVAL_S = 1U;
static constexpr float DM_HIGH_SPEED_THRESHOLD_KMH = 20.0F;
static constexpr uint32_t DM_HIGH_SPEED_WATCH_HOLD_MS = 60U * 1000U;
static constexpr uint32_t DM_HIGH_SPEED_CLEAR_MS = 10U * 1000U;

static constexpr float DM_FALL_FREEFALL_THRESHOLD_G = 0.45F;
static constexpr float DM_FALL_IMPACT_THRESHOLD_G = 2.20F;
static constexpr uint32_t DM_FALL_IMPACT_WINDOW_MS = 1500U;
static constexpr uint32_t DM_SOS_REARM_MS = 10U * 1000U;

static constexpr uint32_t DM_GPS_SLEEP_UPDATE_INTERVAL_S = 24U * 60U * 60U;
static constexpr uint32_t DM_FRESH_FIX_EXTRA_GRACE_S = 3U;
static constexpr uint32_t DM_MAX_DOP = 300U;

static constexpr float DM_RSSI_FILTER_ALPHA = 0.25F;
static constexpr float DM_RSSI_TREND_ALPHA = 0.25F;
static constexpr uint8_t DM_RSSI_MIN_SAMPLES = 3U;
static constexpr uint32_t DM_RSSI_VALID_BEACON_MULTIPLIER = 3U;
static constexpr float DM_RSSI_MOVING_CONFIDENCE = 0.75F;
static constexpr float DM_RSSI_MIN_CONFIDENCE = 0.50F;
static constexpr float DM_RSSI_MIN_MARGIN = 0.15F;
static constexpr float DM_RSSI_MIN_STD_DB = 3.0F;
static constexpr uint32_t DM_RSSI_CALIBRATION_PRIOR_COUNT = 4U;

static constexpr float DM_RSSI_BOOTSTRAP_NEAR_MEAN_DBM = -30.0F;
static constexpr float DM_RSSI_BOOTSTRAP_MID_MEAN_DBM = -50.0F;
static constexpr float DM_RSSI_BOOTSTRAP_WARNING_MEAN_DBM = -60.0F;
static constexpr float DM_RSSI_BOOTSTRAP_BEYOND_MEAN_DBM = -100.0F;
static constexpr float DM_RSSI_BOOTSTRAP_STD_DB = 10.0F;

static constexpr uint8_t DM_LOW_BATTERY_PERCENT = 10U;
static constexpr uint8_t DM_BATTERY_UNKNOWN = 255U;
static constexpr float DM_RANGE_LOSS_MIN_RATIO = 0.50F;
static constexpr float DM_COM_SATURATION_MAX_RATIO = 0.10F;
static constexpr float DM_RSSI_GOOD_DBM = -75.0F;
static constexpr float DM_RSSI_DEGRADING_TREND_DB_PER_S = -0.15F;
static constexpr float DM_RSSI_STABLE_TREND_DB_PER_S = -0.05F;

static constexpr uint16_t DM_BIP_FREQ_START_HZ = 2800U;
static constexpr uint16_t DM_BIP_FREQ_END_HZ = 2400U;
static constexpr uint16_t DM_BOP_FREQ_HZ = 220U;
static constexpr uint16_t DM_BIP_STAGE_1_MS = 140U;
static constexpr uint16_t DM_BIP_STAGE_2_MS = 110U;
static constexpr uint16_t DM_BOP_DURATION_MS = 250U;

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
