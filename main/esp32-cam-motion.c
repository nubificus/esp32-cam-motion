#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "http_server.h"
#include "wifi.h"
#include "esp_wifi.h"

/* Components */
#include "ota-service.h"
#include "esp32-akri.h"

#define WIFI_SUCCESS 1 << 0


#include "esp_camera.h"
#include "esp_timer.h"
#include "camera_pins.h"
#include "mqtt_client.h"


static int motion_count = 0;
static TickType_t last_motion_tick = 0;
#define MOTION_FRAMES_REQUIRED 3
#define MOTION_COOLDOWN_MS 5000

static volatile bool motion_detected = false;

void set_motion_detected(bool v) { motion_detected = v; }
bool is_motion_detected(void) { return motion_detected; }


esp_err_t tasks_http_handler(httpd_req_t *req);
static const char *TAG = "esp32-cam motion";

#define CONFIG_XCLK_FREQ 20000000

static esp_err_t init_camera(void)
{
    camera_config_t camera_config = {
        .pin_pwdn  = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = FRAMESIZE_QVGA,

        .jpeg_quality = 12,
        .fb_count = 2
    };
        //.grab_mode = CAMERA_GRAB_WHEN_EMPTY};//CAMERA_GRAB_LATEST. Sets when buffers should be filled

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

#include "esp_http_server.h"
#include "cJSON.h"

static char mqtt_broker[128] = {0};
static char mqtt_topic[64] = {0};
static char mqtt_user[64] = {0};
static char mqtt_pass[64] = {0};

static esp_mqtt_client_handle_t mqtt_client = NULL;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    mqtt_event_handler_cb(event_data);
}

esp_err_t mqtt_start_client() {
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }

    if (strlen(mqtt_broker) == 0) {
        ESP_LOGW(TAG, "MQTT broker URL is empty, cannot start client");
        return ESP_FAIL;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri =  mqtt_broker,
    // optionally: .verification = {...} if you use certs
    };


    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return ESP_FAIL;
    }
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started with broker %s", mqtt_broker);
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    char content[512];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(content)) {
	httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal Server Error");
        return ESP_FAIL;
    }

    int read_len = 0;
    while (remaining > 0) {
        ret = httpd_req_recv(req, content + read_len, remaining);
        if (ret <= 0) {
	    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal Server Error");
            return ESP_FAIL;
        }
        read_len += ret;
        remaining -= ret;
    }
    content[read_len] = '\0';

    ESP_LOGI(TAG, "Received config: %s", content);

    cJSON *root = cJSON_Parse(content);
    if (!root) {
        ESP_LOGE(TAG, "Invalid JSON");
	httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Json");
        return ESP_FAIL;
    }

    cJSON *broker = cJSON_GetObjectItem(root, "mqtt_broker");
    cJSON *topic = cJSON_GetObjectItem(root, "mqtt_topic");
    cJSON *user = cJSON_GetObjectItem(root, "mqtt_user");
    cJSON *pass = cJSON_GetObjectItem(root, "mqtt_pass");

    if (!broker || !cJSON_IsString(broker)) {
        ESP_LOGE(TAG, "mqtt_broker missing or invalid");
        cJSON_Delete(root);
	httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mqtt_broker missing or invalid");
        return ESP_FAIL;
    }

    if (!topic || !cJSON_IsString(topic)) {
        ESP_LOGE(TAG, "mqtt_topic missing or invalid");
        cJSON_Delete(root);
	httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mqtt_topic missing or invalid");
        return ESP_FAIL;
    }

    strncpy(mqtt_broker, broker->valuestring, sizeof(mqtt_broker) - 1);
    mqtt_broker[sizeof(mqtt_broker) - 1] = 0;

    strncpy(mqtt_topic, topic->valuestring, sizeof(mqtt_topic) - 1);
    mqtt_topic[sizeof(mqtt_topic) - 1] = 0;

    if (user && cJSON_IsString(user)) {
        strncpy(mqtt_user, user->valuestring, sizeof(mqtt_user) - 1);
        mqtt_user[sizeof(mqtt_user) - 1] = 0;
    } else {
        mqtt_user[0] = 0;
    }

    if (pass && cJSON_IsString(pass)) {
        strncpy(mqtt_pass, pass->valuestring, sizeof(mqtt_pass) - 1);
        mqtt_pass[sizeof(mqtt_pass) - 1] = 0;
    } else {
        mqtt_pass[0] = 0;
    }

    cJSON_Delete(root);

    if (mqtt_start_client() != ESP_OK) {
	httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal Server Error");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "MQTT config updated\n");
    return ESP_OK;
}

void mqtt_announce_shutdown(void) {
    char topic_str[32];
    snprintf(topic_str, sizeof(topic_str), "%s/%s", DEVICE_TYPE, "alive");
    if (mqtt_client) {
	vTaskDelay(pdMS_TO_TICKS(4000));
        esp_mqtt_client_publish(mqtt_client, topic_str, "0", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void mqtt_announce_online(void *param) {
	char topic_str[32];
	snprintf(topic_str, sizeof(topic_str), "%s/%s", DEVICE_TYPE, "alive");

	while (1) {
		if (mqtt_client) {
		      esp_mqtt_client_publish(mqtt_client, topic_str, "1", 0, 1, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}


void app_main(void)
{
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ret = connect_wifi();
	if (WIFI_SUCCESS != ret) {
		ESP_LOGI(TAG, "Failed to associate to AP, dying...");
		return;
	}

	ret = akri_server_start();
	if (ret) {
		ESP_LOGE(TAG, "Cannot start akri server");
		abort();
	}

	ret = akri_set_update_handler(ota_request_handler);
	if (ret) {
		ESP_LOGE(TAG, "Cannot set OTA request handler");
		abort();
	}

	ret = akri_set_info_handler(info_get_handler);
	if (ret) {
		ESP_LOGE(TAG, "Cannot set info handler");
		abort();
	}

	ret = akri_set_temp_handler(temp_get_handler);
	if (ret) {
		ESP_LOGE(TAG, "Cannot set temp handler");
		abort();
	}

	ret = akri_set_onboard_handler(onboard_request_handler);
	if (ret) {
		ESP_LOGE(TAG, "Cannot set onboard handler");
		abort();
	}

	        ret = akri_set_handler_generic("/tasks", HTTP_GET, tasks_http_handler);
        if (ret) {
                ESP_LOGE(TAG, "Cannot set stream handler");
                abort();
        }
	ret = akri_set_handler_generic("/config", HTTP_POST, config_post_handler);
	if (ret) {
		ESP_LOGE(TAG, "Cannot set mqtt config handler");
		abort();
	}

        ret = init_camera();
        if (ret != ESP_OK)
        {
            printf("err: %s\n", esp_err_to_name(ret));
            return;
        }        
	
	esp_register_shutdown_handler(mqtt_announce_shutdown);
	int task_res = xTaskCreate(mqtt_announce_online, "mqtt_announce", 4096, NULL, 15, NULL);
	if (task_res != pdPASS) {
		ESP_LOGE(TAG, "Failed to create mqtt announce task");
	}

	ESP_LOGI(TAG, "ESP32 CAMera initialized\n");

#define MOTION_THRESHOLD 30      // Set pixel-difference threshold (0–255)
#define MOTION_PIXELS     500    // Number of different pixels to trigger motion
#define FRAME_WAIT_MS     40     

    camera_fb_t *fb_prev = NULL;
    while (1) {
	if (is_ota_in_progress()) {
            ESP_LOGW(TAG, "OTA in progress, suspending operations");
            esp_camera_deinit();
            vTaskDelay(pdMS_TO_TICKS(1500));
	    continue;
	}

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
	    vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int changed_pixels = 0;
        if (fb_prev) {
            changed_pixels = 0;
            for (int i = 0; i < fb->len; i++) {
                int diff = abs(fb->buf[i] - fb_prev->buf[i]);
                if (diff > MOTION_THRESHOLD)
                    changed_pixels++;
            }
            //ESP_LOGI(TAG, "Pixels changed=%d", changed_pixels);
            esp_camera_fb_return(fb_prev);
        }

	TickType_t now = xTaskGetTickCount();

	if (changed_pixels > MOTION_PIXELS) {
		if ((now - last_motion_tick) > pdMS_TO_TICKS (MOTION_COOLDOWN_MS)) {
			motion_count++;
			if (motion_count >= MOTION_FRAMES_REQUIRED) {
			    last_motion_tick = now;
			    motion_count = 0;
			    ESP_LOGW (TAG, "Motion detected!");

			    set_motion_detected(true);
			    // Publish MQTT message
			    if (mqtt_client) {
				esp_mqtt_client_publish (mqtt_client, mqtt_topic, "1", 0, 1,
							 0);
			    }
			}
		    }
	} else {
		if (is_motion_detected() && (now - last_motion_tick) > pdMS_TO_TICKS (MOTION_COOLDOWN_MS)) {
			set_motion_detected(false);
			if (mqtt_client) {
				esp_mqtt_client_publish (mqtt_client, mqtt_topic, "0", 0, 1, 0);
			}
		}
	    	motion_count = 0;
	}
        fb_prev = fb;

        vTaskDelay(pdMS_TO_TICKS(FRAME_WAIT_MS));  // Set FPS
    }
    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
