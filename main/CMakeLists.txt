idf_component_register(SRCS "main.c"
                           "wifi_mqtt.c"
                           "ntc.c"
                           "touch.c"
                           "heater.c"
                           "pid.c"
                      INCLUDE_DIRS "."
                      REQUIRES esp_wifi esp_adc esp_event esp_netif esp_timer
                               nvs_flash esp_driver_gpio esp_driver_ledc
                               esp_driver_touch_sens)
