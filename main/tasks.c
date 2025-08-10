#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "tasks_endpoint";

esp_err_t tasks_http_handler(httpd_req_t *req)
{
    // Make buffer large enough: ~40 bytes per task * number of tasks
    char *buf = malloc(1024);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Header so browser shows as plain text
    httpd_resp_set_type(req, "text/plain");

    // Write title
    int len = snprintf(buf, 1024, "Task Name\tState\tPrio\tStack\t#\n");
    httpd_resp_send_chunk(req, buf, len);

    // Append vTaskList output
    vTaskList(buf);
    httpd_resp_send_chunk(req, buf, strlen(buf));

    // End of chunks
    httpd_resp_send_chunk(req, NULL, 0);
    free(buf);
    return ESP_OK;
}

