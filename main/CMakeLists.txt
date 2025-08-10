idf_component_register(
	SRCS "http_server.c" "wifi.c" "esp32-cam-motion.c" "camera_pins.h" "tasks.c"
	INCLUDE_DIRS "." 
	REQUIRES mbedtls esp_partition app_update esp_netif esp_wifi nvs_flash esp_http_server ota-service esp32-akri esp32-camera esp_timer mqtt json
) 
