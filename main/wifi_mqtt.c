#include "wifi_mqtt.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_mqtt";

#define WIFI_SSID           CONFIG_HEATING_WIFI_SSID
#define WIFI_PASSWORD       CONFIG_HEATING_WIFI_PASSWORD
#define MQTT_BROKER_URI     CONFIG_HEATING_MQTT_URI
#define WIFI_MAX_RETRY      10

/* Root of every topic this device uses. Kept in one place so the prefix can
 * be changed without touching the individual topics. */
#define TOPIC_ROOT          "heating_cube"

#define TOPIC_SETPOINT      TOPIC_ROOT "/setpoint"
#define TOPIC_STATUS        TOPIC_ROOT "/status"
#define TOPIC_SETUP         TOPIC_ROOT "/setup"
#define TOPIC_COMMANDS      TOPIC_ROOT "/commands"
#define TOPIC_VALUES        TOPIC_ROOT "/values"

/* Longest command name accepted, including the terminator. */
#define COMMAND_MAX_LEN     32

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_events;
static esp_mqtt_client_handle_t s_mqtt;
static volatile bool s_mqtt_connected;
static wifi_mqtt_setpoint_cb_t s_setpoint_cb;
static wifi_mqtt_command_cb_t s_command_cb;
static wifi_mqtt_session_cb_t s_session_cb;
static int s_retries;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_mqtt_connected = false;
        if (s_retries < WIFI_MAX_RETRY) {
            s_retries++;
            ESP_LOGW(TAG, "wifi disconnected, retry %d/%d", s_retries, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&evt->ip_info.ip));
        s_retries = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/* Reads the number following "<key>":  - returns false when the key is
 * absent or the value does not parse. */
static bool json_number(const char *buf, const char *key, float *out)
{
    const char *k = strstr(buf, key);
    if (!k) {
        return false;
    }
    const char *colon = strchr(k, ':');
    if (!colon) {
        return false;
    }
    colon++;
    char *end = NULL;
    float v = strtof(colon, &end);
    if (end == colon) {
        return false;
    }
    *out = v;
    return true;
}

/* Reads the "channels" array: a per-zone flag list such as [1,0,1], in zone
 * order. A non-zero entry heats that zone to the payload's temperature, a
 * zero entry switches it off.
 *
 * Returns false when the key is absent or malformed. Entries beyond
 * WIFI_MQTT_ZONE_COUNT are ignored, and zones the array does not reach are
 * reported as zero - so a short array switches the zones it omits off,
 * exactly as an explicit 0 would. */
static bool json_channel_flags(const char *buf, bool *out)
{
    const char *k = strstr(buf, "channels");
    if (!k) {
        return false;
    }
    const char *open = strchr(k, '[');
    if (!open) {
        return false;
    }

    for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
        out[z] = false;
    }

    const char *p = open + 1;
    int z = 0;
    bool any = false;
    while (*p && *p != ']') {
        if (*p == ' ' || *p == '\t' || *p == ',' || *p == '\r' || *p == '\n') {
            p++;
            continue;
        }
        char *end = NULL;
        long v = strtol(p, &end, 10);
        if (end == p) {
            /* Something that is not a number before the closing bracket. */
            return false;
        }
        if (z < WIFI_MQTT_ZONE_COUNT) {
            out[z] = (v != 0);
        }
        z++;
        any = true;
        p = end;
    }

    /* An unterminated array means the payload was cut short, so the flags
     * cannot be trusted. */
    if (*p != ']') {
        return false;
    }
    return any;
}

static void handle_setpoint_message(const char *data, int len)
{
    /* The documented form is a temperature plus the state of every zone:
     *   {"temp_setpoint_c": 43, "channels": [1,0,1]}
     * which sets 43 C on zones 1 and 3 and switches zone 2 off. The array
     * describes the whole system, so any zone flagged 0 is turned off
     * rather than left as it was.
     *
     * Three older forms still work: a bare number ("62.5"), {"setpoint":
     * 62.5}, and {"zone": 2, "setpoint": 62.5}. Without any zone selection
     * the value applies to every zone. */
    char buf[96];
    if (len <= 0 || len >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "setpoint payload of %d bytes ignored", len);
        return;
    }
    memcpy(buf, data, len);
    buf[len] = '\0';

    /* "temp_setpoint_c" is checked first: json_number() matches on a
     * substring, so looking for "setpoint" first would also match inside
     * "temp_setpoint_c" and find the same number by luck rather than by
     * intent. Checking the longer, more specific key first keeps that
     * explicit. */
    float value;
    if (!json_number(buf, "temp_setpoint_c", &value) &&
        !json_number(buf, "setpoint", &value)) {
        /* No key at all: treat the whole payload as the number. */
        char *end = NULL;
        value = strtof(buf, &end);
        if (end == buf) {
            ESP_LOGW(TAG, "unparsable setpoint: %s", buf);
            return;
        }
    }

    bool channels[WIFI_MQTT_ZONE_COUNT];
    if (json_channel_flags(buf, channels)) {
        /* The array covers every zone: a 1 heats to the given temperature,
         * a 0 switches that zone off. A zone is never left at its previous
         * setpoint, so one message fully describes the desired state. */
        for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
            float target = channels[z] ? value : 0.0f;
            ESP_LOGI(TAG, "zone %d setpoint -> %.2f C%s", z + 1, target,
                     channels[z] ? "" : " (off)");
            if (s_setpoint_cb) {
                s_setpoint_cb(z, target);
            }
        }
        return;
    }

    float zone_f;
    if (json_number(buf, "zone", &zone_f)) {
        int zone = (int)zone_f;
        if (zone < 1 || zone > WIFI_MQTT_ZONE_COUNT) {
            ESP_LOGW(TAG, "zone %d out of range [1, %d], ignored",
                     zone, WIFI_MQTT_ZONE_COUNT);
            return;
        }
        ESP_LOGI(TAG, "zone %d setpoint -> %.2f C", zone, value);
        if (s_setpoint_cb) {
            s_setpoint_cb(zone - 1, value);
        }
        return;
    }

    ESP_LOGI(TAG, "all zones setpoint -> %.2f C", value);
    if (s_setpoint_cb) {
        for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
            s_setpoint_cb(z, value);
        }
    }
}

/* A session request: the same payload as the setpoint topic, but it arms a
 * touch-gated session instead of heating straight away. Missing "channels"
 * means every zone, matching the setpoint topic's behaviour. */
static void handle_session_message(const char *data, int len)
{
    char buf[96];
    if (len <= 0 || len >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "session payload of %d bytes ignored", len);
        return;
    }
    memcpy(buf, data, len);
    buf[len] = '\0';

    float value;
    if (!json_number(buf, "temp_setpoint_c", &value) &&
        !json_number(buf, "setpoint", &value)) {
        char *end = NULL;
        value = strtof(buf, &end);
        if (end == buf) {
            ESP_LOGW(TAG, "unparsable session setpoint: %s", buf);
            return;
        }
    }

    bool channels[WIFI_MQTT_ZONE_COUNT];
    if (!json_channel_flags(buf, channels)) {
        for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
            channels[z] = true;
        }
    }

    char flags[WIFI_MQTT_ZONE_COUNT * 2 + 1];
    int fn = 0;
    for (int z = 0; z < WIFI_MQTT_ZONE_COUNT && fn < (int)sizeof(flags) - 2; z++) {
        if (z) {
            flags[fn++] = ',';
        }
        flags[fn++] = channels[z] ? '1' : '0';
    }
    flags[fn] = '\0';

    ESP_LOGI(TAG, "session request: %.2f C on [%s]", value, flags);
    if (s_session_cb) {
        s_session_cb(value, channels);
    }
}

/* Copies the command name out of the payload, lower-cased. Accepts a bare
 * name ("calibrate") or {"command": "calibrate"}. Returns false if nothing
 * usable is there. */
static bool parse_command(const char *data, int len, char *out, size_t out_size)
{
    char buf[96];
    if (len <= 0 || len >= (int)sizeof(buf)) {
        return false;
    }
    memcpy(buf, data, len);
    buf[len] = '\0';

    const char *start = buf;
    const char *key = strstr(buf, "command");
    if (key) {
        const char *colon = strchr(key, ':');
        if (!colon) {
            return false;
        }
        start = colon + 1;
    }

    /* Skip leading whitespace and the opening quote, if any. */
    while (*start == ' ' || *start == '\t' || *start == '"') {
        start++;
    }

    size_t n = 0;
    while (start[n] && n + 1 < out_size &&
           start[n] != '"' && start[n] != ',' && start[n] != '}' &&
           start[n] != ' ' && start[n] != '\r' && start[n] != '\n') {
        char c = start[n];
        out[n] = (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
        n++;
    }
    out[n] = '\0';
    return n > 0;
}

static void handle_command_message(const char *data, int len)
{
    char command[COMMAND_MAX_LEN];
    if (!parse_command(data, len, command, sizeof(command))) {
        ESP_LOGW(TAG, "unparsable command payload");
        return;
    }

    ESP_LOGI(TAG, "command \"%s\"", command);
    if (s_command_cb) {
        s_command_cb(command);
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t event = data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "mqtt connected, subscribing to %s, %s and %s",
                 TOPIC_SETPOINT, TOPIC_SETUP, TOPIC_COMMANDS);
        esp_mqtt_client_subscribe(s_mqtt, TOPIC_SETPOINT, 1);
        esp_mqtt_client_subscribe(s_mqtt, TOPIC_SETUP, 1);
        esp_mqtt_client_subscribe(s_mqtt, TOPIC_COMMANDS, 1);
        s_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "mqtt disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_DATA:
        if (event->topic_len == strlen(TOPIC_SETPOINT) &&
            strncmp(event->topic, TOPIC_SETPOINT, event->topic_len) == 0) {
            handle_setpoint_message(event->data, event->data_len);
        } else if (event->topic_len == strlen(TOPIC_SETUP) &&
                   strncmp(event->topic, TOPIC_SETUP, event->topic_len) == 0) {
            handle_command_message(event->data, event->data_len);
        } else if (event->topic_len == strlen(TOPIC_COMMANDS) &&
                   strncmp(event->topic, TOPIC_COMMANDS, event->topic_len) == 0) {
            handle_session_message(event->data, event->data_len);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "mqtt error");
        break;
    default:
        break;
    }
}

esp_err_t wifi_mqtt_start(wifi_mqtt_setpoint_cb_t setpoint_cb,
                          wifi_mqtt_command_cb_t command_cb,
                          wifi_mqtt_session_cb_t session_cb)
{
    s_setpoint_cb = setpoint_cb;
    s_command_cb = command_cb;
    s_session_cb = session_cb;

    /* NVS is brought up in app_main, before the touch calibration is read. */

    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL), TAG, "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL), TAG, "ip handler");

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg), TAG, "wifi config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    ESP_LOGI(TAG, "connecting to \"%s\"", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "wifi connection failed after %d retries", WIFI_MAX_RETRY);
        return ESP_FAIL;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .session.keepalive = 30,
        /* Retained LWT so a subscriber can see the controller drop off. */
        .session.last_will = {
            .topic = TOPIC_STATUS,
            .msg = "{\"online\":false}",
            .qos = 1,
            .retain = 1,
        },
    };
    s_mqtt = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt) {
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(
        s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL), TAG, "mqtt handler");
    ESP_RETURN_ON_ERROR(esp_mqtt_client_start(s_mqtt), TAG, "mqtt start");

    ESP_LOGI(TAG, "mqtt client started, broker %s", MQTT_BROKER_URI);
    return ESP_OK;
}

bool wifi_mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

void wifi_mqtt_publish_status(const float *temperature_c,
                              const float *setpoint_c,
                              const float *duty,
                              const bool *touched)
{
    if (!s_mqtt || !s_mqtt_connected) {
        return;
    }

    /* A zone element runs to roughly 80 characters; 128 leaves room for wide
     * temperatures and setpoints, since a truncated payload is dropped
     * whole rather than published short. */
    char payload[64 + WIFI_MQTT_ZONE_COUNT * 128];
    int n = snprintf(payload, sizeof(payload), "{\"online\":true,\"zones\":[");
    if (n < 0 || n >= (int)sizeof(payload)) {
        return;
    }

    for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
        char temp[24];
        /* A zone with no valid reading reports null rather than a NaN
         * literal, which is not valid JSON. */
        if (isnan(temperature_c[z])) {
            snprintf(temp, sizeof(temp), "null");
        } else {
            snprintf(temp, sizeof(temp), "%.2f", temperature_c[z]);
        }

        int m = snprintf(payload + n, sizeof(payload) - n,
                         "%s{\"zone\":%d,\"temperature\":%s,"
                         "\"setpoint\":%.2f,\"duty\":%.3f,\"touch\":%d}",
                         z ? "," : "", z + 1, temp, setpoint_c[z], duty[z],
                         touched[z] ? 1 : 0);
        if (m < 0 || m >= (int)sizeof(payload) - n) {
            return;
        }
        n += m;
    }

    int m = snprintf(payload + n, sizeof(payload) - n, "]}");
    if (m < 0 || m >= (int)sizeof(payload) - n) {
        return;
    }
    n += m;

    esp_mqtt_client_publish(s_mqtt, TOPIC_STATUS, payload, n, 0, 1);
}

/* Appends an "name":[a,b,c] array of ints. Returns false if it did not fit,
 * in which case the caller drops the whole payload. */
static bool append_int_array(char *buf, size_t size, int *n,
                             const char *name, const int *values)
{
    int m = snprintf(buf + *n, size - *n, ",\"%s\":[", name);
    if (m < 0 || m >= (int)size - *n) {
        return false;
    }
    *n += m;

    for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
        m = snprintf(buf + *n, size - *n, "%s%d", z ? "," : "", values[z]);
        if (m < 0 || m >= (int)size - *n) {
            return false;
        }
        *n += m;
    }

    m = snprintf(buf + *n, size - *n, "]");
    if (m < 0 || m >= (int)size - *n) {
        return false;
    }
    *n += m;
    return true;
}

/* As above for floats, with NAN written as null so the payload stays valid
 * JSON - a zone with no reading during the session has no temperature to
 * report. */
static bool append_float_array(char *buf, size_t size, int *n,
                               const char *name, const float *values)
{
    int m = snprintf(buf + *n, size - *n, ",\"%s\":[", name);
    if (m < 0 || m >= (int)size - *n) {
        return false;
    }
    *n += m;

    for (int z = 0; z < WIFI_MQTT_ZONE_COUNT; z++) {
        if (isnan(values[z])) {
            m = snprintf(buf + *n, size - *n, "%snull", z ? "," : "");
        } else {
            m = snprintf(buf + *n, size - *n, "%s%.2f", z ? "," : "", values[z]);
        }
        if (m < 0 || m >= (int)size - *n) {
            return false;
        }
        *n += m;
    }

    m = snprintf(buf + *n, size - *n, "]");
    if (m < 0 || m >= (int)size - *n) {
        return false;
    }
    *n += m;
    return true;
}

void wifi_mqtt_publish_values(const wifi_mqtt_values_t *values)
{
    if (!s_mqtt || !s_mqtt_connected || !values) {
        return;
    }

    char payload[128 + WIFI_MQTT_ZONE_COUNT * 64];
    int n = snprintf(payload, sizeof(payload),
                     "{\"touch_time_ms\":%d,\"temp_setpoint_c\":%.2f",
                     values->touch_time_ms, values->temp_setpoint_c);
    if (n < 0 || n >= (int)sizeof(payload)) {
        return;
    }

    if (!append_int_array(payload, sizeof(payload), &n, "touched", values->touched) ||
        !append_int_array(payload, sizeof(payload), &n, "channels", values->channels) ||
        !append_float_array(payload, sizeof(payload), &n, "temps_max_c", values->temps_max_c) ||
        !append_float_array(payload, sizeof(payload), &n, "temps_avg_c", values->temps_avg_c)) {
        ESP_LOGW(TAG, "values payload too long, dropped");
        return;
    }

    int m = snprintf(payload + n, sizeof(payload) - n, "}");
    if (m < 0 || m >= (int)sizeof(payload) - n) {
        return;
    }
    n += m;

    esp_mqtt_client_publish(s_mqtt, TOPIC_VALUES, payload, n, 0, 1);
}
