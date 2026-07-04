#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "tusb.h"
#include "class/hid/hid_device.h"

#include "driver/gpio.h"

#include "galena_hid.h"
#include "galena_protocol.h"
#include "hid.h"
#include "cdc_log.h"
#include "usb_desc.h"
#include "shared.h"

#define PIN_OSD_DEBUG       GPIO_NUM_21

#define TAG "GAR"

extern SemaphoreHandle_t s_bri_mutex;
extern float g_shadow_brightness;
extern uint8_t g_osd_mode;
extern volatile uint32_t g_osd_report_count;
extern QueueHandle_t s_hid_queue;

void hid_send_event(galena_hid_event_t type, int8_t value)
{
    hid_queue_item_t item = { .type = type, .value = value };
    if (xQueueSend(s_hid_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "HID queue full, dropped event=%d val=%d", type, value);
    }
}

void hid_tx_task(void *arg)
{
    hid_queue_item_t item;
    for (;;) {
        if (xQueueReceive(s_hid_queue, &item, portMAX_DELAY) != pdTRUE)
            continue;

        galena_hid_report_t report = {
            .event_type  = (uint8_t)item.type,
            .event_value = item.value,
        };
        memset(report._reserved, 0, sizeof(report._reserved));

        for (int retry = 0; retry < 10; retry++) {
            if (tud_hid_ready()) {
                if (tud_hid_report(0, &report, sizeof(report))) {
                    ESP_LOGI(TAG, "HID TX event=%d val=%d", item.type, item.value);
                    cdc_log("HID TX event=%d val=%d\r\n", item.type, item.value);
                } else {
                    ESP_LOGW(TAG, "HID TX FAILED event=%d val=%d", item.type, item.value);
                    cdc_log("HID TX FAILED event=%d val=%d\r\n", item.type, item.value);
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ─── TinyUSB Callbacks ──────────────────────────────────────────────────────

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&galena_device_desc;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return galena_config_desc;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t _desc_str[32];

    if (index == 0) {
        memcpy(&_desc_str[1], galena_string_descs[0], 2);
        _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * 1 + 2);
        return _desc_str;
    }

    if (index >= GALENA_STRING_DESC_COUNT)
        return NULL;

    const char *str = galena_string_descs[index];
    uint8_t chr_count = strlen(str);
    if (chr_count > 31) chr_count = 31;

    for (uint8_t i = 0; i < chr_count; i++)
        _desc_str[1 + i] = str[i];

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return galena_hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;
    if (reqlen < sizeof(galena_hid_report_t)) return 0;
    galena_hid_report_t *r = (galena_hid_report_t *)buffer;
    xSemaphoreTake(s_bri_mutex, portMAX_DELAY);
    r->event_type  = HID_EVT_BRIGHTNESS;
    r->event_value = (int8_t)(g_shadow_brightness * 100.0f);
    xSemaphoreGive(s_bri_mutex);
    memset(r->_reserved, 0, sizeof(r->_reserved));
    return sizeof(galena_hid_report_t);
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            const uint8_t *buffer, uint16_t bufsize)
{
    (void)instance;
    if ((report_type == HID_REPORT_TYPE_OUTPUT || report_type == HID_REPORT_TYPE_FEATURE) && bufsize >= 1) {
        g_osd_mode = buffer[0];
        g_osd_report_count++;
        gpio_set_level(PIN_OSD_DEBUG, g_osd_mode ? 1 : 0);
        ESP_LOGI(TAG, "OSD mode=%d (count=%lu)", g_osd_mode, (unsigned long)g_osd_report_count);
        cdc_log("OSD mode=%d (count=%lu)\r\n", g_osd_mode, (unsigned long)g_osd_report_count);
        hid_send_event(HID_EVT_OSD_ACK, g_osd_mode ? (int8_t)1 : (int8_t)0);
    }
}

void tud_mount_cb(void)    { ESP_LOGI(TAG, "USB mounted"); cdc_log("USB mounted\r\n"); }
void tud_umount_cb(void)   { ESP_LOGI(TAG, "USB unmounted"); cdc_log("USB unmounted\r\n"); }
void tud_suspend_cb(bool r){ ESP_LOGI(TAG, "USB suspended"); cdc_log("USB suspended\r\n"); }
void tud_resume_cb(void)   { ESP_LOGI(TAG, "USB resumed"); cdc_log("USB resumed\r\n"); }

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf; (void)rts;
    if (dtr) {
        cdc_log("CDC%d: DTR ON\r\n", itf);
    }
}

void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    char buf[64];
    uint32_t count = tud_cdc_read(buf, sizeof(buf));
    if (count > 0) {
        buf[count] = '\0';
        ESP_LOGI(TAG, "CDC%d RX: %s", itf, buf);
    }
}
