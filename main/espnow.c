#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"

#include "tusb.h"

#include "galena_protocol.h"
#include "galena_hid.h"
#include "shared.h"
#include "cdc_log.h"
#include "hid.h"

#define TAG "GAR"

static uint8_t s_light_bar_mac[] = LIGHT_BAR_MAC;

static int64_t s_suppress_relay_until = 0;

static esp_err_t send_packet(uint8_t type, int32_t value)
{
    bool mounted = tud_mounted();
    galena_packet_t pkt = {
        .type      = type,
        .value     = value,
        .osd_show  = mounted && (g_osd_mode == 1),
        .connected = mounted,
    };
    esp_err_t ret = esp_now_send(s_light_bar_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX type=%d val=%ld show=%d conn=%d ret=%d", type, (long)value, pkt.osd_show, pkt.connected, ret);
    cdc_log("TX type=%d val=%ld show=%d conn=%d ret=%d\r\n", type, (long)value, pkt.osd_show, pkt.connected, ret);
    return ret;
}

void espnow_send_encoder(int delta)
{
    esp_err_t ret = send_packet(PKT_ENCODER, (int32_t)delta);
    if (ret != ESP_OK) ESP_LOGW(TAG, "TX encoder FAILED: %d", ret);
    hid_send_event(HID_EVT_ENCODER, (int8_t)delta);
}

void espnow_send_button(int state)
{
    esp_err_t ret = send_packet(PKT_BUTTON, (int32_t)state);
    if (ret != ESP_OK) ESP_LOGW(TAG, "TX button FAILED: %d", ret);
    hid_send_event(HID_EVT_BUTTON, (int8_t)state);
}

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len)
{
    if (len != sizeof(galena_packet_t)) return;
    espnow_event_t evt;
    memcpy(&evt.pkt, data, sizeof(galena_packet_t));
    g_rx_count++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(s_espnow_queue, &evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();

    cdc_log("ESPNOW RX type=%d val=%d show=%d conn=%d\r\n",
            evt.pkt.type, evt.pkt.value, evt.pkt.osd_show, evt.pkt.connected);
}

static void action_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
        g_send_ok++;
    else
        g_send_fail++;
}

void espnow_task(void *arg)
{
    espnow_event_t evt;
    for (;;) {
        if (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        if (evt.pkt.type == PKT_BOOT) {
            s_suppress_relay_until = esp_timer_get_time() + 2000000;
            ESP_LOGI(TAG, "ESPNOW RX boot -> suppress relay for 2s");
            cdc_log("ESPNOW RX boot -> suppress relay for 2s\r\n");
        }
        else if (evt.pkt.type == PKT_BRIGHTNESS) {
            uint8_t pct = (uint8_t)evt.pkt.value;
            xSemaphoreTake(s_bri_mutex, portMAX_DELAY);
            g_shadow_brightness = pct / 100.0f;
            xSemaphoreGive(s_bri_mutex);
            if (esp_timer_get_time() < s_suppress_relay_until) {
                ESP_LOGI(TAG, "ESPNOW RX brightness=%d%% (suppressed, boot sync)", pct);
                cdc_log("ESPNOW RX brightness=%d%% (suppressed, boot sync)\r\n", pct);
            } else {
                ESP_LOGI(TAG, "ESPNOW RX brightness=%d%% -> HID type=1 val=%d", pct, (int8_t)pct);
                cdc_log("ESPNOW RX brightness=%d%% -> HID type=1 val=%d\r\n", pct, (int8_t)pct);
                hid_send_event(HID_EVT_BRIGHTNESS, (int8_t)pct);
            }
        }
        else if (evt.pkt.type == PKT_LIGHT_STATE) {
            uint8_t on = (uint8_t)evt.pkt.value;
            if (esp_timer_get_time() < s_suppress_relay_until) {
                ESP_LOGI(TAG, "ESPNOW RX light_state=%s (suppressed, boot sync)", on ? "ON" : "OFF");
                cdc_log("ESPNOW RX light_state=%s (suppressed, boot sync)\r\n", on ? "ON" : "OFF");
            } else {
                ESP_LOGI(TAG, "ESPNOW RX light_state=%s -> HID type=6 val=%d", on ? "ON" : "OFF", (int8_t)on);
                cdc_log("ESPNOW RX light_state=%s -> HID type=6 val=%d\r\n", on ? "ON" : "OFF", (int8_t)on);
                hid_send_event(HID_EVT_LIGHT_STATE, (int8_t)on);
            }
        }
        else {
            ESP_LOGW(TAG, "ESPNOW RX unknown type=%d val=%d", evt.pkt.type, evt.pkt.value);
            cdc_log("ESPNOW RX unknown type=%d val=%d\r\n", evt.pkt.type, evt.pkt.value);
        }
    }
}

void heartbeat_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        send_packet(PKT_HEARTBEAT, 0);
    }
}

void osd_poll_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (tud_mounted()) {
            hid_send_event(HID_EVT_OSD_REQUEST, 0);
        } else {
            g_osd_mode = 0;
        }
    }
}

void espnow_stats_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    for (;;) {
        uint8_t primary = 0;
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&primary, &second);
        ESP_LOGI(TAG, "ESPNOW stats: rx=%lu send_ok=%ld fail=%ld ch=%d",
                 (unsigned long)g_rx_count, (long)g_send_ok, (long)g_send_fail, primary);
        cdc_log("ESPNOW stats: rx=%lu send_ok=%ld fail=%ld ch=%d\r\n",
                (unsigned long)g_rx_count, (long)g_send_ok, (long)g_send_fail, primary);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);
    ESP_LOGI(TAG, "Wi-Fi channel=%d (requested 1)", primary);
}

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(action_send_cb));

    uint8_t pmk[] = "galena-pmk-1234";
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));

    esp_now_peer_info_t peer = { };
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_light_bar_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}
