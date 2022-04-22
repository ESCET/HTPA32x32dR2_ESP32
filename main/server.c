#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "connect.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "cJSON.h"
#define TAG "SERVER"
extern xSemaphoreHandle HtpaSemaphore;
extern xSemaphoreHandle HTPAReadySemaphore;
extern uint32_t temp_pix_uint32[32][32];
extern cJSON *rootJSON;
extern cJSON *rootJSON2;
 
void resetWifi(void *params)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_wifi_stop();
    xSemaphoreGive(initSemaphore);
    vTaskDelete(NULL);
}

static esp_err_t on_url_hit(httpd_req_t *req)
{
    ESP_LOGI(TAG, "url %s was hit", req->uri);
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false};
    size_t total = 0, used = 0;

    esp_err_t ret = esp_vfs_spiffs_register(&config);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }

    size_t t1otal = 0, us1ed = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", t1otal, us1ed);
    }
    char path[600];
    if (strcmp(req->uri, "/") == 0)
    {
        strcpy(path, "/spiffs/index.html");
    }
    else
    {
        sprintf(path, "/spiffs%s", req->uri);
    }
    FILE *file = fopen(path, "r");
    if (file == NULL)
    {
        httpd_resp_send_404(req);
    }
    else
    {

        char lineRead[256];
        while (fgets(lineRead, sizeof(lineRead), file))
        {
            httpd_resp_sendstr_chunk(req, lineRead);
        }
        httpd_resp_sendstr_chunk(req, NULL);
    }
    esp_vfs_spiffs_unregister(NULL);

    return ESP_OK;
}

static esp_err_t on_get_htpa_req(httpd_req_t *req)
{
    ESP_LOGI(TAG, "url %s was hit", req->uri);
    xSemaphoreGive(HtpaSemaphore);
    xSemaphoreTake(HTPAReadySemaphore, portMAX_DELAY);
    //   cJSON_Print(rootJSON);
    //   cJSON_Print(rootJSON2);
    // printf("strlen(message2): %d\n", strlen(message));
    // printf("sizeof(message2): %d\n", sizeof(message));
    
     char * http_message =cJSON_PrintUnformatted(rootJSON);
                //  printf("array2string: %s \n",http_message);
 
    // char *message = malloc((sizeof(char)*strlen(message1)+strlen(message2))+10);
    //  printf(    "dd \n");
    //  printf(    "strlen(http_message): %u \n",strlen(*http_message));
    httpd_resp_send(req, http_message, strlen(http_message));
    // httpd_resp_send_chunk(req, http_message, strlen(http_message));
    cJSON_Delete(rootJSON);
    // cJSON_Delete(rootJSON2);
    free(http_message);
    
    return ESP_OK;
}

static esp_err_t on_setwifi_req(httpd_req_t *req)
{
    ESP_LOGI(TAG, "url %s was hit", req->uri);
    char buf[150];
    memset(&buf, 0, sizeof(buf));
    httpd_req_recv(req, buf, req->content_len);
    char *ssid = strtok(buf, "\r\n");
    // pass NULL use the same buf
    char *pass = strtok(NULL, "\r\n");
    ssid = strchr(ssid, '=') + 1;
    pass = strchr(pass, '=') + 1;

    ESP_LOGI(TAG, "ssid: %s pass: %s\n", ssid, pass);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t nvs;
    nvs_open("wifiCreds", NVS_READWRITE, &nvs);
    nvs_set_str(nvs, "ssid", ssid);
    nvs_set_str(nvs, "pass", pass);
    nvs_close(nvs);

    httpd_resp_set_status(req, "303");
    httpd_resp_set_hdr(req, "Location", "/wifi-set.html");
    httpd_resp_send(req, NULL, 0);

    xTaskCreate(resetWifi, "reset wifi", 1024 * 2, NULL, 15, NULL);

    return ESP_OK;
}

void RegisterEndPoints(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "starting server");
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "COULD NOT START SERVER");
    }

    httpd_uri_t htpa_end_point_config = {
        .uri = "/api/get",
        .method = HTTP_GET,
        .handler = on_get_htpa_req};
    httpd_register_uri_handler(server, &htpa_end_point_config);

    httpd_uri_t led_end_point_config = {
        .uri = "/api/setwifi",
        .method = HTTP_POST,
        .handler = on_setwifi_req};
    httpd_register_uri_handler(server, &led_end_point_config);

    httpd_uri_t first_end_point_config = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = on_url_hit};
    httpd_register_uri_handler(server, &first_end_point_config);
}