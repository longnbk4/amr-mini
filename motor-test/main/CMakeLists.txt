idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ""
                    REQUIRES esp_wifi esp_http_server nvs_flash esp_netif esp_driver_gpio esp_driver_ledc esp_driver_uart)