#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "RBControl_wifi.hpp"

#define TAG "RbWifi"

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
            tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_STOP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_STOP");
            tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGI(TAG, "Got IP: %s\n",
                     ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_AP_START: {
            ESP_LOGI(TAG, "SYSTEM_EVENT_AP_START");
            tcpip_adapter_ip_info_t info;
            IP4_ADDR(&info.ip, 192, 168, 0, 1);
            IP4_ADDR(&info.gw, 192, 168, 0, 1);
            IP4_ADDR(&info.netmask, 255, 255, 255, 0);
            tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
            ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));

            dhcps_lease_t lease;
            lease.enable = true;
            IP4_ADDR(&lease.start_ip, 192, 168, 0, 10);
            IP4_ADDR(&lease.end_ip, 192, 168, 0, 250);

            tcpip_adapter_dhcps_option(
                (tcpip_adapter_option_mode_t)TCPIP_ADAPTER_OP_SET,
                (tcpip_adapter_option_id_t)TCPIP_ADAPTER_REQUESTED_IP_ADDRESS,
                (void*)&lease, sizeof(dhcps_lease_t)
            );

            ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
            break;
        }
        case SYSTEM_EVENT_AP_STOP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_AP_STOP");
            tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_AP_STACONNECTED");
            break;
        default:
            break;
    }
    return ESP_OK;
}

class WiFiInitializer {
public:
    WiFiInitializer() {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );

        tcpip_adapter_init();

        ESP_ERROR_CHECK(esp_event_loop_init(&event_handler, NULL));

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    }

    ~WiFiInitializer() {

    }
};

namespace rb {

void WiFi::connect(const char *ssid, const char *pass) {
    init();

    esp_wifi_stop();

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t cfg = { 0 };
    snprintf((char*)cfg.sta.ssid, 32, "%s", ssid);
    snprintf((char*)cfg.sta.password, 64, "%s", pass);
    esp_wifi_set_config(WIFI_IF_STA, &cfg);

    ESP_ERROR_CHECK(esp_wifi_start());
}

void WiFi::startAp(const char *ssid, const char *pass, uint8_t channel) {
    init();

    esp_wifi_stop();

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t cfg = { 0 };
    snprintf((char*)cfg.ap.ssid, 32, "%s", ssid);
    snprintf((char*)cfg.ap.password, 64, "%s", pass);
    cfg.ap.channel = channel;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.ap.beacon_interval = 400;
    cfg.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));

    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
}

void WiFi::init() {
    static WiFiInitializer init;
}

}; // namespace rb