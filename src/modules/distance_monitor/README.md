# Distance Monitor

Distance Monitor is an experimental Meshtastic firmware module for a small
private group of GPS trackers. The primary V1 use case is one adult **base**
and one child **tracker**.

Status: **V1 beta2 / protocol V5**

The module uses `PRIVATE_APP` packets and does not modify the Meshtastic
protobuf schema.

## V1 behavior

### Pairing

After boot, the base performs a V5 handshake with every configured tracker.

- Initial pairing: base and newly booted tracker play **3 BOPs within about 1 second**.
- If the base reboots while the tracker was already running, only the base plays
  the pairing sound.
- A radio reconnection inside the same boot sessions does not replay the pairing
  sound.
- `PAIR_CONFIRM` is repeated until the tracker returns a `POSITION_REPORT`
  carrying the paired flag.

The base waits approximately `DM_PAIR_CONFIRM_DELAY_MS` before confirming the
initial pairing. `ALIVE_RESPONSE` releases the discovery retry timer immediately,
so a successful discovery does not add an unnecessary `DM_HANDSHAKE_RETRY_MS`
delay before `PAIR_CONFIRM`.

### Module initialization

Meshtastic creates core modules from `setupModules()` by constructing them; the
`MeshModule::setup()` hook is not automatically invoked by that creation path.
Distance Monitor therefore performs its initialization from its constructor through
an idempotent `initializeIfNeeded()` function. The public `setup()` hook and the
packet/button paths call the same guard defensively.

A valid boot must log both:

```text
Distance Monitor audio: buzzer ready gpio=25 ...
Distance Monitor initialized: firmware=1.0.0-beta2 protocol=5 ... session=<non-zero>
```

### Tracker position reports

Trackers publish `POSITION_REPORT` packets. The base does not poll positions.

The first report uses the local maximum report interval. The base then computes
the distance and sends `SET_POSITION_INTERVAL` when a different interval is
needed.

`SET_POSITION_INTERVAL` has **no application ACK**. Every following
`POSITION_REPORT` includes `appliedIntervalSec`. If it does not match the base
request, the base sends the command again when that report is received.

A tracker sends a report even when the GNSS has no fix. Therefore:

- `POSITION_REPORT + NO_FIX` means the radio link is alive;
- no `POSITION_REPORT` within the link timeout is treated as a communication
  problem.

### Adaptive report interval

The report interval is derived from:

```text
Tmax = Dmax / (design_relative_speed * minimum_samples_per_Dmax)
```

then limited by `DM_MAX_REPORT_INTERVAL_CAP_S`.

Defaults:

```text
design relative speed = 7 km/h
minimum samples / Dmax = 3
minimum report interval = 9 s
maximum cap = 20 s
```

The current interval is linearly reduced from `Tmax` near the base to 9 seconds
at `Dmax`, then quantized down to an odd number of seconds. The 9-second floor
is chosen for the observed EU_868 airtime of the current direct PKI packets; the
old 5-second floor could exceed the regional transmit duty-cycle budget.

Examples:

| Dmax | Effective Tmax |
|---:|---:|
| 50 m | 9 s |
| 100 m | 17 s |
| 200 m | 19 s |

The 1 Hz mode used for vehicle detection is a **GNSS acquisition rate**, not a
1 Hz LoRa report rate. It does not lower the LoRa report interval below
`DM_MIN_REPORT_INTERVAL_S`.

## GPS and IMU strategy

Each unit owns its GNSS power strategy.

- Moving: GNSS remains active at the currently required interval.
- Stationary long enough: the last valid fix becomes `CACHED_STATIONARY` and
  the GNSS can enter hard sleep.
- Movement after a stationary cache invalidates the cache until a new fix is
  acquired.
- If the IMU is unavailable, the module does not trust stationary caching and
  keeps GNSS active.

A `FRESH_FIX` or `CACHED_STATIONARY` position is usable for distance
calculation. `NO_FIX` immediately selects the RSSI fallback.

## Distance alert

There is one distance-alert sound. The BIP repetition period is interpolated
from this table:

| Distance / Dmax | BIP period |
|---:|---:|
| < 80% | silent |
| 80% | 30 s |
| 90% | 22.5 s |
| 100% | 3 s |
| 150% | 2 s |
| 200%+ | 1 s |

A base single press snoozes a currently valid distance condition for 60 seconds.

If a GPS distance has crossed 100% and the following source is a less
conclusive RSSI fallback that says `Near` or `Mid`, the previous critical
distance alarm remains latched until the adult acknowledges it. A later valid
GPS distance below 80% can clear that latch automatically.

## RSSI fallback

The base broadcasts a small `BASE_BEACON` every 10 seconds without ACK or retry.
`BASE_BEACON` is suspended while a received SOS is active so background RSSI
traffic cannot compete with the emergency exchange.

The 10-second default is deliberate for the observed EU_868 preset: a beacon
occupied roughly half a second of airtime in field logs. The previous 5-second
cadence alone consumed about 10% of channel time on the base.

The tracker calculates direct base-to-tracker RSSI statistics:

- mean;
- standard deviation;
- trend.

These are included in `POSITION_REPORT`. The base independently measures the
direct tracker-to-base RSSI and maintains the same statistics.

Only direct LoRa packets are used. Relayed or MQTT packets are excluded.

The V1 feature is:

```text
RSSI_fused = 0.5 * RSSI_base_to_tracker
           + 0.5 * RSSI_tracker_to_base
```

The same 50/50 rule is used for the trend.

### Empirical calibration

When both GPS positions are usable, the fused RSSI updates an online calibration
bucket. Calibration is kept in RAM for the current boot session.

Bootstrap values are deliberately only starting values:

| Band | Distance ratio | Mean RSSI | Std |
|---|---:|---:|---:|
| Near | 0–30% | -30 dBm | 10 dB |
| Mid | 30–80% | -50 dBm | 10 dB |
| Warning | 80–100% | -60 dBm | 10 dB |
| Beyond | >=100% | -100 dBm | 10 dB |

`DmOnlineStats` uses Welford updates. The distance estimator compares Gaussian
likelihoods for the four bands and returns `DmDistanceBandEstimate`.

The winning band is accepted only when:

```text
P1 >= DM_RSSI_MIN_CONFIDENCE
P1 - P2 >= DM_RSSI_MIN_MARGIN
```

Defaults are 0.50 and 0.15.

Movement broadens the effective standard deviation. Default confidence is:

```text
stationary = 100%
moving = 75%
```

Fallback mapping:

| RSSI estimate | Distance behavior |
|---|---|
| Near | no distance alert |
| Mid | no distance alert |
| Warning | same behavior as 80% Dmax |
| Beyond | same behavior as 100% Dmax |
| Unknown | system fault |

`Unknown` means GPS is not usable and the RSSI fallback cannot provide a
sufficiently identifiable winning band. It uses the generic fault sound.

## Link loss

The link timeout is time based:

```text
link timeout = DM_MAX_REPORT_INTERVAL + DM_LINK_TIMEOUT_MARGIN
```

It is not tied to the number of missing GPS fixes.

When reports stop, the base analyzes the last known state and logs a probable
cause:

- `RADIO_LOW_BATTERY`;
- `RADIO_RANGE_LOSS`;
- `RADIO_COM_SATURATION`;
- `RADIO_UNEXPECTED_LOSS`;
- `REMOTE_SHUTDOWN`.

The cause is diagnostic only. The user hears the same generic fault pattern.

For probable close-range communication saturation, the module waits silently for
`DM_COM_SATURATION_SILENT_MS` (default 2 minutes). It does not add active probe
traffic: a live tracker is recovered passively by its next `POSITION_REPORT`. If
the link is still unavailable after the silent period, the normal fault sound
starts. An adult press can snooze this exceptional condition for 10 minutes.

## SOS

SOS causes are distinct in the protocol and logs but use the same user alarm:

```text
ManualButton
FallDetected
HighSpeedMovement
```

The base SOS sound is continuous until the adult presses the base button.

The tracker retries the same SOS sequence every
`DM_RESEND_TIMEOUT_MS` until an application `SOS_ACK` is received.

The base sends `SOS_ACK` only after its continuous SOS tone has actually started.

### Manual SOS

Double press on a tracker.

### Fall detection

V1 beta uses a simple two-stage detector:

1. raw acceleration norm below `DM_FALL_FREEFALL_THRESHOLD_G`;
2. impact above `DM_FALL_IMPACT_THRESHOLD_G` inside
   `DM_FALL_IMPACT_WINDOW_MS`.

The default thresholds are intentionally exposed in
`DistanceMonitorConfig.h` and should be tuned with real T1000-E field data.

### Vehicle / high-speed detection

The IMU first detects dynamic acceleration above:

```text
DM_VEHICLE_ACCEL_THRESHOLD_MPS2 = 0.25 m/s²
```

After a short confirmation, the tracker wakes/keeps GNSS active at 1 Hz for a
temporary watch window.

If GNSS ground speed exceeds:

```text
DM_HIGH_SPEED_THRESHOLD_KMH = 20 km/h
```

the tracker sends `SOS(HighSpeedMovement)`.

## Notification from the base

Double press on the base sends `NOTIFICATION`.

Tracker behavior:

```text
1 BOP / second for 5 seconds
```

The tracker sends `NOTIFICATION_ACK` as soon as the notification is accepted by
the application. The sound then runs independently. The ACK therefore confirms
application reception, not acoustic completion.

Base behavior:

- application ACK received from all configured trackers: **2 rapid BOPs**;
- no application ACK: **no confirmation sound**.

Notification is not automatically retried. The adult can double press again.

Duplicate notification packets are ACKed again but do not replay the 5-second
tracker pattern.

## Shutdown

Normal tracker power-off remains available.

Before shutdown the tracker sends `SHUTDOWN_NOTICE`, then waits
`DM_SHUTDOWN_TX_GRACE_MS` (default 500 ms) before normal firmware power-off.

The notice does **not** suppress a later link-loss alert. It only changes the
diagnostic cause to `REMOTE_SHUTDOWN`.

## Audio vocabulary

The V1 intentionally keeps the number of user patterns small.

| Event | Pattern |
|---|---|
| Pairing | 3 BOPs in about 1 second |
| Distance | one BIP at interpolated cadence |
| SOS | continuous high tone |
| Notification confirmation on base | 2 rapid BOPs |
| Notification on tracker | 1 BOP/s for 5 s |
| Link/fallback fault | 2 rapid BOPs every 30 s |

Default sound synthesis:

```text
BIP: 2800 Hz for 140 ms, then 2400 Hz for 110 ms
BOP: 220 Hz for 250 ms
```

These values are expected to be tuned by ear on the actual buzzer.

## Button mapping

### Base

| Action | Distance Monitor behavior |
|---|---|
| Single press | acknowledge SOS, otherwise snooze fault, otherwise snooze/ack distance |
| Double press | send notification |
| Long shutdown press | normal shutdown |

### Tracker

| Action | Distance Monitor behavior |
|---|---|
| Single press | consumed silently |
| Double press | manual SOS |
| Long shutdown press | send shutdown notice, then normal shutdown |

The integration patch routes the physical button events before the native
Meshtastic position-ping action.

## Protocol V5

All messages use `PRIVATE_APP`.

| Type | ACK policy |
|---|---|
| `ALIVE_REQUEST` | no transport ACK; retried by pairing state machine |
| `ALIVE_RESPONSE` | no transport ACK |
| `PAIR_CONFIRM` | no transport ACK; implicit confirmation in next paired report |
| `POSITION_REPORT` | no ACK, no retry |
| `SET_POSITION_INTERVAL` | no ACK; verified by `appliedIntervalSec` |
| `BASE_BEACON` | no ACK, no retry |
| `SOS` | application ACK, retry same sequence every `DM_RESEND_TIMEOUT_MS` |
| `SOS_ACK` | no application ACK |
| `NOTIFICATION` | application ACK |
| `NOTIFICATION_ACK` | no application ACK |
| `SHUTDOWN_NOTICE` | no ACK; shutdown is never blocked |

The protocol header contains:

```text
'D' 'M' | protocol version | message type | uint32 sequence
```

Multi-byte integer fields use little-endian encoding.

## Runtime Meshtastic profile

At module setup the V1 applies these values in RAM:

- role: `CLIENT_MUTE`;
- native buzzer feedback: disabled;
- triple-click GPS toggle: disabled;
- heartbeat LED: disabled;
- native smart position broadcast: disabled;
- native position broadcast period: reduced to a very long interval;
- device telemetry: disabled;
- MQTT: disabled;
- External Notification: disabled at runtime so it cannot compete for the T1000-E buzzer.

The module does not call `saveToDisk()` for these settings.

For a dedicated child-tracking installation, configure the radio/channel
persistently as well:

- use a private channel with a random PSK;
- set the correct legal radio region;
- disable native channel position precision if native Meshtastic position
  messages must never expose coordinates;
- keep both nodes on the same modem/channel settings.

## Configuration

Edit `DistanceMonitorConfig.h`.

At minimum, configure the node numbers:

```cpp
static constexpr DmDefaultMember DM_DEFAULT_MEMBERS[] = {
    {0x8B3A4A14U, true},   // Base
    {0xE87C9792U, false},  // Tracker
};
```

Only one base should be marked `true`.

The main distance parameter is:

```cpp
DM_MAX_DISTANCE_M
```

The rest of the report cadence is derived automatically from `Dmax`.

## Files

```text
DistanceMonitorAudio.cpp/.h     Non-blocking BIP/BOP/SOS scheduler
DistanceMonitorConfig.h         V1 parameters and node membership
DistanceMonitorModule.cpp/.h    State machine, alarms, buttons and orchestration
DistanceMonitorPackets.cpp      Protocol V5 serialization and packet handling
DistanceMonitorPosition.cpp     GNSS/IMU power, fall and high-speed detection
DistanceMonitorRssi.cpp/.h      RSSI filters, online calibration and estimator
DistanceMonitorTypes.h          V5 states and structures
DistanceMonitorUtils.cpp/.h     Timing, distance and interpolation helpers
README.md                       This document
```

Two small changes outside the folder are required. See:

```text
integration/distance-monitor-v5.patch
```

## V1 beta notes

This is a safety-oriented experimental module, not a certified child-safety
device. In particular, RSSI-to-distance behavior and fall detection are
environment-dependent and must be validated with the actual trackers, body
placement, buildings and radio preset used in the field.

Recommended validation before calling the beta stable:

1. pairing/reboot/reconnection matrix;
2. report interval versus Dmax;
3. indoor/outdoor GNSS transitions;
4. RSSI calibration and `Unknown` rate;
5. close-range radio saturation behavior;
6. battery-loss and hard-power-loss cases;
7. notification ACK loss;
8. SOS ACK loss/retry;
9. manual shutdown notice;
10. fall and vehicle-speed false-positive tests.
