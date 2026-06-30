#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_private/usb_phy.h"

#include "tusb.h"

#include "galena_protocol.h"
#include "galena_hid.h"
#include "shared.h"
#include "usb_desc.h"
#include "cdc_log.h"
#include "hid.h"
#include "espnow.h"
#include "encoder.h"
#include "button.h"

#define TAG "GAR"

#define PIN_OSD_DEBUG GPIO_NUM_21

// ─── Shared Globals ──────────────────────────────────────────────────────────

float            g_shadow_brightness = 1.0f;
SemaphoreHandle_t s_bri_mutex;
uint8_t          g_osd_mode = 0;

QueueHandle_t    s_espnow_queue;
QueueHandle_t    s_hid_queue;

volatile int32_t g_send_ok   = 0;
volatile int32_t g_send_fail = 0;
volatile uint32_t g_rx_count = 0;
volatile uint32_t g_osd_report_count = 0;

// ─── GPIO Init ───────────────────────────────────────────────────────────────

static void gpio_init_pins(void)
{
    gpio_config_t enc_cfg = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&enc_cfg);

    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);
}

// ─── USB Device Task ─────────────────────────────────────────────────────────

void usb_device_task(void *arg)
{
    for (;;) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ─── Entry Point ─────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "Galena Action Ring booting...");

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_init_pins();

    // USB PHY (OTG, device mode, internal PHY, full speed)
    usb_phy_handle_t phy_hdl = NULL;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_FULL,
    };
    ESP_ERROR_CHECK(usb_new_phy(&phy_conf, &phy_hdl));

    const tusb_rhport_init_t rh_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    if (!tusb_rhport_init(0, &rh_init)) {
        ESP_LOGE(TAG, "TinyUSB init failed");
        return;
    }

    // Create queues and mutex
    s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));
    s_hid_queue    = xQueueCreate(20, sizeof(hid_queue_item_t));
    s_bri_mutex    = xSemaphoreCreateMutex();
    if (s_espnow_queue == NULL || s_hid_queue == NULL || s_bri_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create queues or mutex — aborting");
        abort();
    }

    gpio_set_direction(PIN_OSD_DEBUG, GPIO_MODE_OUTPUT);

    // Wi-Fi + ESP-NOW
    wifi_init();
    espnow_init();

    uint8_t my_mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             LIGHT_BAR_MAC[0], LIGHT_BAR_MAC[1], LIGHT_BAR_MAC[2],
             LIGHT_BAR_MAC[3], LIGHT_BAR_MAC[4], LIGHT_BAR_MAC[5]);

    // Create tasks
    xTaskCreate(usb_device_task,   "usb_task",    2048, NULL, 6, NULL);
    xTaskCreate(espnow_task,       "espnow_task", 4096, NULL, 6, NULL);
    xTaskCreate(encoder_task,      "enc_task",    2048, NULL, 4, NULL);
    xTaskCreate(button_task,       "btn_task",    2048, NULL, 4, NULL);
    xTaskCreate(hid_tx_task,       "hid_tx_task", 2048, NULL, 3, NULL);
    xTaskCreate(heartbeat_task,    "hb_task",     2048, NULL, 2, NULL);
    xTaskCreate(osd_poll_task,     "osd_poll",    2048, NULL, 3, NULL);
    xTaskCreate(espnow_stats_task, "espnow_stats", 2048, NULL, 2, NULL);

    ESP_LOGI(TAG, "Galena Action Ring ready.");
}
