#pragma once

#include <cstddef>
#include <cstdint>

// Maximum number of nodes managed by one Distance Monitor group.
static constexpr size_t DM_MAX_MEMBERS = 2U;

// Protocol V7: 16-bit boot session IDs, compact position flags and DOP transfer.
static constexpr uint8_t DM_PROTOCOL_VERSION = 7U;
static constexpr const char *DM_FIRMWARE_VERSION = "1.0.0-beta7-fall-ring";

// Main scheduling cadence.
static constexpr uint32_t DM_TICK_INTERVAL_MS = 40U;
static constexpr uint32_t DM_CONTROL_INTERVAL_MS = 1000U;
static constexpr uint32_t DM_SUMMARY_INTERVAL_MS = 5000U;

// When true, distance/radio/SOS alarm audio is muted. Alarm state, logs, radio
// traffic, pairing/notification tones and SEARCH feedback remain active.
static constexpr bool DM_ALARM_AUDIO_SILENT = true;

// Status LED ergonomics.
static constexpr bool DM_STATUS_LED_ENABLED = true;
static constexpr uint32_t DM_LED_HEARTBEAT_INTERVAL_MS = 5000U;
static constexpr uint32_t DM_LED_HEARTBEAT_ON_MS = 250U;
static constexpr uint32_t DM_LED_EVENT_ON_MS = 100U;
static constexpr uint32_t DM_LED_EVENT_OFF_MS = 100U;
// On nRF52, values below 255 are driven with PWM.
static constexpr uint8_t DM_LED_BASE_BRIGHTNESS = 255U;
static constexpr uint8_t DM_LED_TRACKER_BRIGHTNESS = 48U;

// Distance safety configuration.
static constexpr float DM_MAX_DISTANCE_M = 100.0F;
static constexpr float DM_DISTANCE_ALERT_START_RATIO = 0.80F;
// GPS is trusted only while the sum of base+tracker accuracy is at most this
// fraction of Dmax. Above that, the decision falls back to the RSSI LUT.
static constexpr float DM_GPS_MAX_COMBINED_ACCURACY_RATIO = 0.50F;
// DOP is carried as hundredths. Approximate accuracy = DOP/100 * 5 metres.
static constexpr float DM_DOP_TO_ACCURACY_METERS = 5.0F;

static constexpr float DM_DESIGN_RELATIVE_SPEED_MPS = 7.0F / 3.6F;
static constexpr uint8_t DM_MIN_SAMPLES_PER_DMAX = 3U;
static constexpr uint8_t DM_MIN_REPORT_INTERVAL_S = 9U;
static constexpr uint8_t DM_MAX_REPORT_INTERVAL_CAP_S = 20U;

// Pairing / radio timings.
static constexpr uint32_t DM_BASE_BEACON_INTERVAL_MS = 10U * 1000U;
static constexpr uint32_t DM_LINK_TIMEOUT_MARGIN_MS = 5U * 1000U;
static constexpr uint32_t DM_COM_SATURATION_SILENT_MS = 0U;
static constexpr uint32_t DM_PAIR_CONFIRM_DELAY_MS = 15U * 1000U;
static constexpr uint32_t DM_HANDSHAKE_RETRY_MS = 20U * 1000U;
static constexpr uint32_t DM_PAIR_REASSERT_INTERVAL_MS = 30U * 1000U;
static constexpr uint32_t DM_PAIR_REMOTE_BOOT_WINDOW_S = 30U;
static constexpr uint32_t DM_DISTANCE_SNOOZE_MS = 60U * 1000U;
static constexpr uint32_t DM_FAULT_SNOOZE_MS = 60U * 1000U;
static constexpr uint32_t DM_COM_SATURATION_SNOOZE_MS = 10U * 60U * 1000U;
static constexpr uint32_t DM_FAULT_AUDIO_PERIOD_MS = 30U * 1000U;
static constexpr uint32_t DM_RESEND_TIMEOUT_MS = 5U * 1000U;
static constexpr uint32_t DM_NOTIFICATION_ACK_WINDOW_MS = 10U * 1000U;
static constexpr uint32_t DM_SHUTDOWN_TX_GRACE_MS = 500U;

// Motion / SOS configuration.
static constexpr uint32_t DM_STATIONARY_CONFIRM_MS = 10U * 1000U;
// MOVING/STATIONARY sensitivity only; it is independent from fall detection.
static constexpr float DM_IMU_MOTION_THRESHOLD_G = 0.08F;
static constexpr float DM_IMU_GRAVITY_ALPHA = 0.01F;
static constexpr float DM_HIGH_SPEED_THRESHOLD_KMH = 20.0F;
static constexpr uint32_t DM_HIGH_SPEED_CLEAR_MS = 10U * 1000U;
static constexpr uint32_t DM_SOS_REARM_MS = 10U * 1000U;

// Accelerometer-only fall detector. DM samples the IMU once per module tick,
// keeps a 3 s rolling history, then analyzes three contiguous logical phases:
// newest -> POST | IMPACT | PRE -> oldest. Ratios are expressed in percent.
static constexpr uint32_t DM_IMU_SAMPLE_RATE_HZ = 1000U / DM_TICK_INTERVAL_MS;
static constexpr uint32_t DM_FALL_WINDOW_TIME_MS = 3000U;
static constexpr uint8_t DM_FALL_POST_RATIO_PERCENT = 40U;
static constexpr uint8_t DM_FALL_IMPACT_RATIO_PERCENT = 20U;
static constexpr uint8_t DM_FALL_PRE_RATIO_PERCENT = 40U;

// Number of samples in the complete ring and in each logical phase. PRE gets
// the remainder so integer rounding can never make the three phases overflow.
static constexpr size_t DM_MAX_ROLL_BUFFER =
    static_cast<size_t>((DM_FALL_WINDOW_TIME_MS * DM_IMU_SAMPLE_RATE_HZ) / 1000U);
static constexpr size_t DM_FALL_POST_SAMPLES =
    (DM_MAX_ROLL_BUFFER * DM_FALL_POST_RATIO_PERCENT) / 100U;
static constexpr size_t DM_FALL_IMPACT_SAMPLES =
    (DM_MAX_ROLL_BUFFER * DM_FALL_IMPACT_RATIO_PERCENT) / 100U;
static constexpr size_t DM_FALL_PRE_SAMPLES =
    DM_MAX_ROLL_BUFFER - DM_FALL_POST_SAMPLES - DM_FALL_IMPACT_SAMPLES;

// Each phase has one decision statistic. These are tuning defaults, not
// pediatric clinical thresholds: use the emitted PRE/IMP/POST statistics to
// tune them on the actual tracker placement and activities.
static constexpr float DM_FALL_PRE_MAX_MEAN_G = 0.70F;
static constexpr float DM_FALL_IMPACT_MIN_RMS_G = 1.80F;
static constexpr float DM_FALL_POST_MAX_STD_G = 0.15F;

// Candidate statistics are logged immediately when the PASS mask changes,
// then rate-limited while a candidate persists to avoid flooding the console.
static constexpr uint32_t DM_FALL_LOG_INTERVAL_MS = 500U;

static_assert((1000U % DM_TICK_INTERVAL_MS) == 0U,
              "DM_TICK_INTERVAL_MS must divide 1000 for IMU sample rate");
static_assert(DM_MAX_ROLL_BUFFER > 0U, "fall rolling buffer cannot be empty");
static_assert(DM_FALL_POST_SAMPLES > 0U, "POST phase cannot be empty");
static_assert(DM_FALL_IMPACT_SAMPLES > 0U, "IMPACT phase cannot be empty");
static_assert(DM_FALL_PRE_SAMPLES > 0U, "PRE phase cannot be empty");
static_assert(
    DM_FALL_POST_RATIO_PERCENT +
            DM_FALL_IMPACT_RATIO_PERCENT +
            DM_FALL_PRE_RATIO_PERCENT ==
        100U,
    "fall phase ratios must sum to 100 percent");

// GNSS is kept running continuously by DM. Native Meshtastic broadcasts stay
// practically disabled because DM owns its own compact position protocol.
static constexpr uint32_t DM_NATIVE_POSITION_BROADCAST_INTERVAL_S = 24U * 60U * 60U;
static constexpr uint32_t DM_FRESH_FIX_EXTRA_GRACE_S = 3U;
static constexpr uint32_t DM_GPS_FUNCTIONAL_CHECK_MS = 60U * 1000U;

// RSSI EMA used only to smooth direct radio measurements. No GPS-based RSSI
// calibration or persistence is performed anymore.
static constexpr float DM_RSSI_FILTER_ALPHA = 0.25F;
static constexpr float DM_RSSI_TREND_ALPHA = 0.25F;
static constexpr uint32_t DM_RSSI_VALID_BEACON_MULTIPLIER = 3U;

// Manual indoor RSSI proximity LUT. RSSI is negative: the largest value is the
// best path. Operational bands are Near, Medium, Warning and VeryFar.
static constexpr float DM_RSSI_NEAR_THRESHOLD_DBM = -50.0F;
static constexpr float DM_RSSI_MEDIUM_THRESHOLD_DBM = -80.0F;
static constexpr float DM_RSSI_WARNING_THRESHOLD_DBM = -100.0F;
// Synthetic alert ratios let the existing alarm cadence work without claiming
// that RSSI is a metric distance.
static constexpr float DM_RSSI_WARNING_ALERT_RATIO = 0.80F;
static constexpr float DM_RSSI_VERY_FAR_ALERT_RATIO = 1.10F;

// SEARCH stays GNSS-only. At <=15 m it immediately reaches maximum proximity.
static constexpr uint32_t DM_SEARCH_LONG_PRESS_MS = 2000U;
static constexpr uint32_t DM_SEARCH_LONG_LONG_PRESS_MS = 5000U;
static constexpr uint32_t DM_SEARCH_PULSE_INTERVAL_MS = 750U;
static constexpr uint32_t DM_SEARCH_CONTACT_PULSE_INTERVAL_MS = 500U;
static constexpr float DM_SEARCH_DETECTION_MAX_RATIO = 0.80F;
static constexpr float DM_SEARCH_CONTACT_DISTANCE_M = 15.0F;
static constexpr uint16_t DM_SEARCH_DETECTION_MIN_HZ = 1450U;
static constexpr uint16_t DM_SEARCH_DETECTION_NEAR_HZ = 2600U;
static constexpr uint16_t DM_SEARCH_DETECTION_CONTACT_HZ = 3100U;
static constexpr uint16_t DM_SEARCH_DETECTION_DURATION_MIN_MS = 25U;
static constexpr uint16_t DM_SEARCH_DETECTION_DURATION_MAX_MS = 90U;
static constexpr uint16_t DM_SEARCH_DETECTION_CONTACT_DURATION_MS = 180U;
static constexpr uint16_t DM_SEARCH_DETECTION_DELAY_FAR_MS = 180U;
static constexpr uint16_t DM_SEARCH_DETECTION_DELAY_NEAR_MS = 150U;

// Radio loss diagnosis thresholds.
static constexpr uint8_t DM_LOW_BATTERY_PERCENT = 10U;
static constexpr uint8_t DM_BATTERY_UNKNOWN = 255U;
static constexpr float DM_RANGE_LOSS_MIN_RATIO = 0.50F;
static constexpr float DM_COM_SATURATION_MAX_RATIO = 0.10F;
static constexpr float DM_RSSI_GOOD_DBM = -75.0F;
static constexpr float DM_RSSI_DEGRADING_TREND_DB_PER_S = -0.15F;
static constexpr float DM_RSSI_STABLE_TREND_DB_PER_S = -0.05F;

// Alarm tones.
static constexpr uint16_t DM_BIP_FREQ_START_HZ = 2300U;
static constexpr uint16_t DM_BIP_FREQ_END_HZ = 3100U;
static constexpr uint16_t DM_BOP_FREQ_HZ = 1900U;
static constexpr uint16_t DM_BIP_STAGE_1_MS = 70U;
static constexpr uint16_t DM_BIP_STAGE_2_MS = 40U;
static constexpr uint16_t DM_BOP_DURATION_MS = 220U;

// Wire protocol header: magic[2], version[1], type[1], sequence[4].
static constexpr uint8_t DM_PROTOCOL_MAGIC_0 = 0x44U;
static constexpr uint8_t DM_PROTOCOL_MAGIC_1 = 0x4DU;
static constexpr size_t DM_PROTOCOL_HEADER_SIZE = 8U;

// V7 compact message sizes. Boot/session identifiers are 16-bit.
static constexpr size_t DM_ALIVE_SIZE = DM_PROTOCOL_HEADER_SIZE + 6U;
static constexpr size_t DM_PAIR_CONFIRM_SIZE = DM_PROTOCOL_HEADER_SIZE + 5U;
static constexpr size_t DM_POSITION_REPORT_SIZE = DM_PROTOCOL_HEADER_SIZE + 16U;
static constexpr size_t DM_SET_INTERVAL_SIZE = DM_PROTOCOL_HEADER_SIZE + 3U;
static constexpr size_t DM_BASE_BEACON_SIZE = DM_PROTOCOL_HEADER_SIZE + 2U;
static constexpr size_t DM_SOS_SIZE = DM_PROTOCOL_HEADER_SIZE + 3U;
static constexpr size_t DM_SOS_ACK_SIZE = DM_PROTOCOL_HEADER_SIZE + 6U;
static constexpr size_t DM_NOTIFICATION_SIZE = DM_PROTOCOL_HEADER_SIZE + 2U;
static constexpr size_t DM_NOTIFICATION_ACK_SIZE = DM_PROTOCOL_HEADER_SIZE + 6U;
static constexpr size_t DM_SHUTDOWN_NOTICE_SIZE = DM_PROTOCOL_HEADER_SIZE + 2U;
static constexpr uint32_t DM_SEQUENCE_DUPLICATE_WINDOW_MS = 5U * 60U * 1000U;
static constexpr size_t DM_RECENT_SEQUENCE_COUNT = 8U;

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
