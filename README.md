# Heating Cube Firmware

ESP32-S3 firmware for a three-zone heating controller. Each zone pairs an NTC
thermistor, a capacitive touch pad and a MOSFET-driven heating element under
independent PID control, with Wi-Fi and MQTT for configuration and telemetry.

## Hardware

| Zone | NTC (ADC1) | Touch pad | Heater gate |
|------|-----------|-----------|-------------|
| 1    | GPIO1 (CH0) | GPIO4 | GPIO7 |
| 2    | GPIO2 (CH1) | GPIO5 | GPIO8 |
| 3    | GPIO3 (CH2) | GPIO6 | GPIO9 |

Zone indices are shared across every module; the firmware uses 0-based indices
internally and 1-based numbering in logs and MQTT payloads.

**Thermistors** are 10 kΩ NTCs (B25/85 = 3950) on a low-side divider with a
10 kΩ series resistor. Readings are median-filtered over 5 samples, then
smoothed with an exponential moving average.

**Heaters** are driven by LEDC hardware PWM at 1 kHz, 12-bit resolution, all
three sharing one timer. Gates are forced low before anything else during
boot.

**Touch pads** use the ESP32-S3 V2 touch peripheral with hardware benchmark
tracking, smoothing, hysteresis and debounce. The touch channel number equals
the GPIO number on this chip.

## Building

Requires ESP-IDF v6.1 targeting `esp32s3`.

```bash
idf.py set-target esp32s3
idf.py menuconfig      # see Configuration below
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

Under `menuconfig` → **Heating controller**:

| Option | Default | Meaning |
|---|---|---|
| `HEATING_WIFI_SSID` | `myssid` | Wi-Fi network |
| `HEATING_WIFI_PASSWORD` | `mypassword` | Wi-Fi passphrase |
| `HEATING_MQTT_URI` | `mqtt://192.168.1.10:1883` | Broker URI (`mqtt://` or `mqtts://`) |
| `HEATING_MAX_TEMP_C` | `120` | Safety cut-out, °C |

PID gains are compile-time constants in `main/main.c` (`PID_KP`, `PID_KI`,
`PID_KD`). The defaults suit a small thermal mass: proportional-dominant with
a small derivative term, since a large one would amplify ADC noise into gate
chatter at the 10 Hz loop rate. Tune on the real load — raise `PID_KP` until
it oscillates, back off to about half, then add `PID_KI` until the
steady-state offset closes.

## MQTT interface

All topics are rooted at `heating_cube`, defined once as `TOPIC_ROOT` in
`main/wifi_mqtt.c`.

| Topic | Direction | Purpose |
|---|---|---|
| `heating_cube/setpoint` | in | Direct heating control |
| `heating_cube/commands` | in | Arm a touch-gated session |
| `heating_cube/setup` | in | Maintenance commands |
| `heating_cube/status` | out | Periodic telemetry (every 2 s) |
| `heating_cube/values` | out | One message per completed session |

### `heating_cube/setpoint` — direct mode

Heats immediately, with no touch interaction.

```json
{"temp_setpoint_c": 43, "channels": [1, 0, 1]}
```

The `channels` array describes **every** zone: a non-zero entry heats that
zone to the given temperature, a zero entry switches it off. One message
always leaves the system in a fully known state. An array shorter than three
entries switches the zones it omits off.

Three older forms remain accepted:

```json
{"setpoint": 43}              // all zones
{"zone": 2, "setpoint": 43}   // one zone, others untouched
43                            // bare number, all zones
```

Setpoints are clamped to `[0, HEATING_MAX_TEMP_C]`; out-of-range values are
rejected with a log line. A setpoint of `0` switches a zone off.

### `heating_cube/commands` — session mode

Same payload format, but it **arms** a session instead of heating straight
away. Heating begins only once all three pads are touched simultaneously, and
stops the instant any pad is released. See
[Touch-gated sessions](#touch-gated-sessions) below.

```bash
mosquitto_pub -h broker -t heating_cube/commands \
  -m '{"temp_setpoint_c":43,"channels":[1,0,1]}'
```

### `heating_cube/setup` — maintenance

Accepts a bare command name or `{"command": "..."}`:

| Command | Effect |
|---|---|
| `calibrate` | Re-measures the untouched touch baselines and derives thresholds, storing them in NVS. **The pads must not be touched while this runs.** |
| `diag` | Logs benchmark, smooth reading, deviation, threshold and active state for every pad |

```bash
mosquitto_pub -h broker -t heating_cube/setup -m 'diag'
```

### `heating_cube/status` — telemetry

Published every 2 seconds:

```json
{"online": true, "zones": [
  {"zone": 1, "temperature": 32.92, "setpoint": 43.00, "duty": 0.412, "touch": 0},
  {"zone": 2, "temperature": null,  "setpoint": 0.00,  "duty": 0.000, "touch": 0},
  {"zone": 3, "temperature": 41.08, "setpoint": 43.00, "duty": 0.255, "touch": 1}
]}
```

A zone with no valid reading reports `"temperature": null`. The broker holds a
last-will message so `"online"` goes false if the device drops off.

### `heating_cube/values` — session results

One message per completed session:

```json
{"touch_time_ms": 12345, "temp_setpoint_c": 43.00,
 "touched": [1, 1, 0], "channels": [1, 0, 1],
 "temps_max_c": [40.00, 39.50, 41.00],
 "temps_avg_c": [38.10, null, 39.40]}
```

| Field | Meaning |
|---|---|
| `touch_time_ms` | Elapsed time from PWM start to stop |
| `touched` | Pad states at the instant heating stopped |
| `temp_setpoint_c` | The setpoint the session ran at |
| `channels` | Which zones the session heated |
| `temps_max_c` | Peak temperature per NTC over the running period |
| `temps_avg_c` | Mean temperature per NTC over the running period |

Temperature statistics cover **all three** NTCs regardless of which channels
were heated, and span the running period only. A zone with no valid reading
for the entire session reports `null`.

## Touch-gated sessions

A session couples heating to a physical hold on all three pads — heat flows
only while a person is in contact with every pad, and stops the moment that
contact breaks.

```
IDLE ──(non-zero setpoint on /commands)──▶ ARMED
ARMED ──(all three pads touched)────────▶ RUNNING   [PWM on, timer starts]
RUNNING ──(any pad released)────────────▶ IDLE      [PWM off, publish]
RUNNING ──(zero setpoint on /commands)──▶ IDLE      [PWM off, publish]
ARMED ──(zero setpoint on /commands)────▶ IDLE      [no publish]
```

Key behaviours:

- **Armed does not heat.** The setpoint is held at zero until all pads are
  touched.
- **Any release stops everything**, including channels whose own pad is still
  held. All active channels start and stop together, so `touch_time_ms` is one
  number rather than one per channel.
- **A session is consumed by running.** After it stops, a fresh `/commands`
  message is required; it does not re-arm itself.
- **A zero setpoint cancels.** Cancelling a *running* session still publishes
  its results; cancelling an *armed* one does not, since nothing ran.
- **Sessions override direct setpoints** on the channels they own. When a
  session ends it releases those channels and zeroes both the session and the
  manual setpoint, so zones return to direct control cold rather than
  inheriting a stale value.

Touch state is sampled once per control cycle (100 ms), so session timing has
roughly decisecond resolution.

## Safety

Several independent cut-outs force heaters off:

- **Over-temperature** — any zone reading at or above `HEATING_MAX_TEMP_C` is
  cut immediately and its PID reset.
- **Sensor failure** — 10 consecutive invalid readings (about 1 second) cut
  that zone.
- **Boot** — every gate is driven low before any other initialisation.
- **Setpoint range** — values outside `[0, HEATING_MAX_TEMP_C]` are rejected
  rather than clamped silently.

If Wi-Fi or MQTT fails to come up, the controller keeps regulating on the last
known setpoints rather than rebooting into an uncontrolled state; the MQTT
client retries on its own.

## Source layout

| File | Responsibility |
|---|---|
| `main/main.c` | Control loop, PID scheduling, session state machine, safety cut-outs |
| `main/wifi_mqtt.c` | Wi-Fi station, MQTT client, payload parsing and publishing |
| `main/touch.c` | Touch peripheral setup, calibration, threshold storage, stuck-pad recovery |
| `main/ntc.c` | ADC sampling, median filter, Steinhart–Hart conversion |
| `main/heater.c` | LEDC PWM setup and duty control |
| `main/pid.c` | PID with derivative-on-measurement and integral clamping |

The control loop runs at 10 Hz (`CONTROL_PERIOD_MS = 100`). Each cycle reads
touch and temperature for all zones, advances the session state machine, runs
one PID step per zone, and every 20th cycle publishes status.

## Touch calibration

Touch thresholds are derived from each pad's untouched baseline and stored in
NVS, so they survive reboots. Send `calibrate` on `heating_cube/setup` with
nothing touching the pads.

Calibration samples each benchmark 16 times and rejects a pad whose reading
swings more than 5 % across those samples — normally a sign the pad was being
touched or is disconnected. Pads can also read as unstable in the first minute
after boot, before their benchmarks have settled; if a calibration is rejected
that way, simply repeat it.

Use `diag` to see how a real press compares to the threshold:

```
zone 1: benchmark=68435 smooth=68440 deviation=5 threshold=13687 active=0
```

A press should push `deviation` well clear of `threshold`. If presses are
missed, lower `TOUCH_THRESHOLD_RATIO` in `main/touch.c`; if pads trigger from
a nearby hand or trip their neighbours, raise it. The ratio is applied to the
measured benchmark at calibration time, so any change needs a recalibration to
take effect.

### Electrical note

The heater MOSFETs couple capacitively into the touch wiring — opto-isolation
breaks the galvanic path but not the electric field, and the drain dV/dt at
each PWM edge injects charge into a high-impedance touch node. If false
triggers correlate with PWM activity rather than temperature, the effective
fixes are physical: a gate resistor to soften dV/dt, greater separation
between touch and drain wiring, a grounded shield between them, or shorter
touch runs.
