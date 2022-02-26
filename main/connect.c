#include <stdio.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include <string.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "connect.h"


#define SSID CONFIG_WIFI_SSID
#define PASSWORD CONFIG_WIFI_PASSWORD
#define SOS_INIT_WIFI_SSID CONFIG_SOS_INIT_WIFI_SSID
#define SOS_INIT_WIFI_PASS CONFIG_SOS_INIT_WIFI_PASS
#define SOS_WIFI_CHANNEL CONFIG_SOS_WIFI_CHANNEL
#define SOS_MAX_STA_CONN CONFIG_SOS_MAX_STA_CONN
#define ESP_MAXIMUM_RETRY 5


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
esp_netif_t *sos_ap;
esp_netif_t *sos_sta;
#define TAG "CONNECTION"

static int s_retry_num = 0;
extern xSemaphoreHandle connectionSemaphore;




static void event_handler(void *arg, esp_event_base_t *event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "connected\n");
        // s_retry_num = 0;
        // xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }

    return ESP_OK;
}

void wifi_init_softap(void)
{
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    // esp_netif_destroy(sos_sta);
    // sos_ap = esp_netif_create_default_wifi_ap();

    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
    //                                                     ESP_EVENT_ANY_ID,
    //                                                     &event_handler,
    //                                                     NULL,
    //                                                     NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SOS_INIT_WIFI_SSID,
            .ssid_len = strlen(SOS_INIT_WIFI_SSID),
            .channel = SOS_WIFI_CHANNEL,
            .password = SOS_INIT_WIFI_PASS,
            .max_connection = SOS_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(SOS_INIT_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    // wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
}

void wifi_init_sta(char *ssid, char *pass)
{
    s_wifi_event_group = xEventGroupCreate();

    
 

    wifi_config_t wifi_config = {
        .sta = {

            //  .ssid = (intptr_t)ssid,

            // .ssid_len = strlen(ssid),

            // .password = (intptr_t)pass,

            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, pass);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
 }

void hardResetWifi(void *params)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_wifi_stop();
    xSemaphoreGive(initSemaphore);
    vTaskDelete(NULL);
}

void resetwifi()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    nvs_handle_t nvs;

    nvs_open("wifiCreds", NVS_READWRITE, &nvs);
    nvs_erase_key(nvs, "ssid" );
    nvs_erase_key(nvs, "pass" );
    nvs_close(nvs);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

 }
void wifiInit(void *params)
{
  
     esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // connect to wifi
    // wifi_init_sta();
    // create wifi AP
    while (true)
    {
        ESP_LOGE(TAG, "while loop");

        if (xSemaphoreTake(initSemaphore, portMAX_DELAY))
        {

            nvs_handle_t nvs;
            nvs_open("wifiCreds", NVS_READWRITE, &nvs);

            size_t ssidLen, passLen;
            char *ssid = NULL, *pass = NULL;

            if (nvs_get_str(nvs, "ssid", NULL, &ssidLen) == ESP_OK)
            {
                if (ssidLen > 0)
                {
                    ssid = malloc(ssidLen);
                    nvs_get_str(nvs, "ssid", ssid, &ssidLen);
                }
            }

            if (nvs_get_str(nvs, "pass", NULL, &passLen) == ESP_OK)
            {
                if (passLen > 0)
                {
                    pass = malloc(passLen);
                    nvs_get_str(nvs, "pass", pass, &passLen);
                }
            }
            wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
            ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

           
            if (ssid != NULL && pass != NULL)
            {
                wifi_init_sta(ssid, pass);
                  }
            else
            {
                wifi_init_softap();
               
                ESP_LOGI(TAG, "wifi_init_sta finished.");
              
            }
            esp_event_handler_instance_t instance_any_id;
            esp_event_handler_instance_t instance_got_ip;
            ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                                ESP_EVENT_ANY_ID,
                                                                &event_handler,
                                                                NULL,
                                                                &instance_any_id));
            ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                                IP_EVENT_STA_GOT_IP,
                                                                &event_handler,
                                                                NULL,
                                                                &instance_got_ip));

           

            ESP_ERROR_CHECK(esp_wifi_start());
            xSemaphoreGive(connectionSemaphore);

            if (ssid != NULL)
                free(ssid);
            if (pass != NULL)
                free(pass);
        }
    }
}