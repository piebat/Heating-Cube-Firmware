#pragma once

#include <stdbool.h>
#include "esp_err.h"

/* One status/setpoint pair of topics covers every zone; the zone index
 * travels in the payload. */
#define WIFI_MQTT_ZONE_COUNT    3

/* Called when a setpoint is received on "heating_cube/setpoint".
 * zone is 0-based, and the callback runs once per zone the payload selects.
 *
 * The documented payload gives a temperature and the state of every zone:
 *   {"temp_setpoint_c": 43, "channels": [1,0,1]}
 * sets 43 C on zones 1 and 3 and switches zone 2 off. The flags are in zone
 * order; anything non-zero heats that zone, a zero turns it off. The array
 * describes the whole system, so one message always leaves every zone in a
 * known state.
 *
 * Also accepted: {"setpoint": 43}, {"zone": 2, "setpoint": 43} and a bare
 * number. Without "channels" or "zone" the value applies to every zone. */
typedef void (*wifi_mqtt_setpoint_cb_t)(int zone, float setpoint_c);

/* Called when a command is received on "heating_cube/setup". The command
 * name is NUL-terminated and lower-cased. Runs on the MQTT client task, so
 * the handler must not block for long. */
typedef void (*wifi_mqtt_command_cb_t)(const char *command);

/* Called when a session request arrives on "heating_cube/commands", in the
 * same payload format as the setpoint topic:
 *   {"temp_setpoint_c": 43, "channels": [1,0,1]}
 *
 * Unlike a setpoint, this only arms a session: heating starts once every
 * pad is touched and stops as soon as any is released. A setpoint of zero
 * cancels an armed or running session.
 *
 * channels is 0-based and has WIFI_MQTT_ZONE_COUNT entries. Runs on the
 * MQTT client task. */
typedef void (*wifi_mqtt_session_cb_t)(float setpoint_c, const bool *channels);

/* One completed heating session, published on "heating_cube/values" when
 * the session stops. */
typedef struct {
    /* PWM start to stop. All active channels run together, so this is one
     * number for the session rather than one per channel. */
    int   touch_time_ms;
    /* Pad states at the instant heating stopped: the configuration that
     * ended the session. More than one pad may be released at once. */
    int   touched[WIFI_MQTT_ZONE_COUNT];
    /* The setpoint the session ran at. */
    float temp_setpoint_c;
    /* Which channels the session heated. */
    int   channels[WIFI_MQTT_ZONE_COUNT];
    /* Peak and mean of each NTC over the running period only. A zone with
     * no valid reading during the session reports NAN, published as null. */
    float temps_max_c[WIFI_MQTT_ZONE_COUNT];
    float temps_avg_c[WIFI_MQTT_ZONE_COUNT];
} wifi_mqtt_values_t;

/* Publishes a finished session on "heating_cube/values".
 * Safe to call when the broker is down; the message is then dropped. */
void wifi_mqtt_publish_values(const wifi_mqtt_values_t *values);

/* Brings up Wi-Fi in station mode, waits for an IP, then starts the MQTT
 * client. Blocks until the network is up or the retry limit is reached.
 * Either callback may be NULL. */
esp_err_t wifi_mqtt_start(wifi_mqtt_setpoint_cb_t setpoint_cb,
                          wifi_mqtt_command_cb_t command_cb,
                          wifi_mqtt_session_cb_t session_cb);

bool wifi_mqtt_is_connected(void);

/* Publishes the state of all zones as one JSON message on "heating_cube/status".
 * Each array element is {zone, temperature, setpoint, duty, touch}; a zone
 * with no valid reading reports its temperature as null, and touch is 0 for
 * untouched or 1 for touched.
 * Safe to call when the broker is down; the message is then dropped. */
void wifi_mqtt_publish_status(const float *temperature_c,
                              const float *setpoint_c,
                              const float *duty,
                              const bool *touched);
