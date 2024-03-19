idf_component_register(SRCS "WiFi.c"
                    REQUIRES driver freertos esp_wifi esp_event nvs_flash esp_netif
                    INCLUDE_DIRS "include")
