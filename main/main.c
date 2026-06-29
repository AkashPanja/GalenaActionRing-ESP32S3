#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_private/usb_phy.h"

#include "tusb.h"
#include "class/hid/hid_device.h"

#include "galena_protocol.h"
#include "galena_hid.h"

#define TAG                 "GAR"

#define PIN_ENC_A           GPIO_NUM_11
#define PIN_ENC_B           GPIO_NUM_12
#define PIN_BUTTON          GPIO_NUM_13

#define ENCODER_RESOLUTION  2
#define MAX_ENCODER_DELTA   10
#define DEBOUNCE_MS         50
#define HOLD_MS             700

static float   g_shadow_brightness = 1.0f;
static uint8_t g_osd_mode = 0;

static QueueHandle_t s_espnow_queue;
static QueueHandle_t s_hid_queue;
static uint8_t s_light_bar_mac[] = LIGHT_BAR_MAC;

typedef struct {
    galena_hid_event_t type;
    int8_t             value;
} hid_queue_item_t;

#define PIN_OSD_DEBUG       GPIO_NUM_21
static volatile uint32_t g_osd_report_count = 0;

static volatile int32_t g_send_ok   = 0;
static volatile int32_t g_send_fail = 0;
static volatile uint32_t g_rx_count = 0;

// ─── CDC Logging ───────────────────────────────────────────────────────────────

static void cdc_log(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    if (len > 0 && tud_cdc_write_available() >= (uint32_t)len) {
        for (int i = 0; i < len; i++) {
            tud_cdc_write_char(buf[i]);
        }
        tud_cdc_write_flush();
    }
}

static void hid_send_event(galena_hid_event_t type, int8_t value);

// ─── USB Device Descriptor ────────────────────────────────────────────────────

#define USBD_VID           0x303A
#define USBD_PID           0x4006

static const tusb_desc_device_t galena_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

static const char *galena_string_descs[] = {
    (const char[]){ 0x09, 0x04 },
    "Anode Labs",
    "Galena Action Ring",
    "GAR-001",
};

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x83
#define EP_HID_IN         0x84

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t galena_config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    
    // CDC Interface
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, ITF_NUM_CDC_DATA, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    
    // HID Interface
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, false,
                       sizeof(galena_hid_report_descriptor),
                       EP_HID_IN, 64, 1),
};

// ─── TinyUSB Descriptor Callbacks ────────────────────────────────────────────

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

    if (index >= sizeof(galena_string_descs) / sizeof(galena_string_descs[0]))
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

// ─── HID Report Callbacks ─────────────────────────────────────────────────────

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;
    if (reqlen < sizeof(galena_hid_report_t)) return 0;
    galena_hid_report_t *r = (galena_hid_report_t *)buffer;
    r->event_type  = HID_EVT_BRIGHTNESS;
    r->event_value = (int8_t)(g_shadow_brightness * 100.0f);
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

// ─── CDC Callbacks ────────────────────────────────────────────────────────────

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

// ─── HID TX ───────────────────────────────────────────────────────────────────

static void hid_send_event(galena_hid_event_t type, int8_t value)
{
    hid_queue_item_t item = { .type = type, .value = value };
    if (xQueueSend(s_hid_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "HID queue full, dropped event=%d val=%d", type, value);
    }
}

static void hid_tx_task(void *arg)
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

// ─── ESP-NOW Send (Action Ring → Light Bar) ──────────────────────────────────

static void espnow_send_encoder(int delta)
{
    bool mounted = tud_mounted();
    galena_packet_t pkt = {
        .type      = PKT_ENCODER,
        .value     = (int32_t)delta,
        .osd_show  = mounted && (g_osd_mode == 1),
        .connected = mounted,
    };
    esp_now_send(s_light_bar_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX encoder delta=%+d show=%d conn=%d", delta, pkt.osd_show, pkt.connected);
    cdc_log("TX encoder delta=%+d show=%d conn=%d\r\n", delta, pkt.osd_show, pkt.connected);
    hid_send_event(HID_EVT_ENCODER, (int8_t)delta);
}

static void espnow_send_button(int state)
{
    bool mounted = tud_mounted();
    galena_packet_t pkt = {
        .type      = PKT_BUTTON,
        .value     = (int32_t)state,
        .osd_show  = mounted && (g_osd_mode == 1),
        .connected = mounted,
    };
    esp_now_send(s_light_bar_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX button state=%d show=%d conn=%d", state, pkt.osd_show, pkt.connected);
    cdc_log("TX button state=%d show=%d conn=%d\r\n", state, pkt.osd_show, pkt.connected);
    hid_send_event(HID_EVT_BUTTON, (int8_t)state);
}

// ─── ESP-NOW Receive (Light Bar → Action Ring) ──────────────────────────────

typedef struct {
    galena_packet_t pkt;
} espnow_event_t;

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
    
    // Log reception to CDC
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

static int64_t s_suppress_relay_until = 0;

static void espnow_task(void *arg)
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
            g_shadow_brightness = pct / 100.0f;
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

// ─── Encoder Task ─────────────────────────────────────────────────────────────

static void encoder_task(void *arg)
{
    int8_t enc_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    uint8_t last_ab = (gpio_get_level(PIN_ENC_A) << 1) |
                       gpio_get_level(PIN_ENC_B);
    int     position    = 0;
    int     last_sent   = 0;
    int     initialized = 0;

    for (;;) {
        uint8_t a = gpio_get_level(PIN_ENC_A);
        uint8_t b = gpio_get_level(PIN_ENC_B);
        uint8_t ab = (a << 1) | b;

        int8_t step = enc_table[(last_ab << 2) | ab];
        position += step;
        last_ab = ab;

        int detent_pos = position / ENCODER_RESOLUTION;
        int delta = detent_pos - last_sent;

        if (delta != 0) {
            if (!initialized) {
                initialized = 1;
                last_sent = detent_pos;
            } else if (abs(delta) <= MAX_ENCODER_DELTA) {
                espnow_send_encoder(delta);
                last_sent = detent_pos;
            } else {
                last_sent = detent_pos;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ─── Button Task ──────────────────────────────────────────────────────────────

static void button_task(void *arg)
{
    bool       last_pressed = false;
    TickType_t press_start  = 0;
    bool       hold_fired   = false;

    for (;;) {
        bool pressed = (gpio_get_level(PIN_BUTTON) == 0);

        if (pressed && !last_pressed) {
            press_start = xTaskGetTickCount();
            hold_fired  = false;
            espnow_send_button(1);
        }

        if (pressed && !hold_fired) {
            TickType_t held = xTaskGetTickCount() - press_start;
            if (held >= pdMS_TO_TICKS(HOLD_MS)) {
                espnow_send_button(2);
                hold_fired = true;
            }
        }

        if (!pressed && last_pressed) {
            TickType_t held = xTaskGetTickCount() - press_start;
            if (!hold_fired && held >= pdMS_TO_TICKS(50) &&
                              held <  pdMS_TO_TICKS(HOLD_MS)) {
                espnow_send_button(0);
            } else if (!hold_fired) {
                espnow_send_button(0);
            }
        }

        last_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    }
}

// ─── Heartbeat (Action Ring → Light Bar) ─────────────────────────────────────

static void heartbeat_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        bool mounted = tud_mounted();
        galena_packet_t pkt = {
            .type      = PKT_HEARTBEAT,
            .value     = 0,
            .osd_show  = mounted && (g_osd_mode == 1),
            .connected = mounted,
        };
        esp_err_t ret = esp_now_send(s_light_bar_mac, (uint8_t *)&pkt, sizeof(pkt));
        ESP_LOGI(TAG, "TX heartbeat show=%d conn=%d ret=%d", pkt.osd_show, pkt.connected, ret);
        cdc_log("TX heartbeat show=%d conn=%d ret=%d\r\n", pkt.osd_show, pkt.connected, ret);
    }
}

// ─── OSD Poll ─────────────────────────────────────────────────────────────────

static void osd_poll_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (tud_mounted()) {
            hid_send_event(HID_EVT_OSD_REQUEST, 0);
        } else {
            g_osd_mode = 0;
            gpio_set_level(PIN_OSD_DEBUG, 0);
        }
    }
}

// ─── USB Device Task ─────────────────────────────────────────────────────────

static void usb_device_task(void *arg)
{
    for (;;) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ─── Debug Stats Task ─────────────────────────────────────────────────────────

static void espnow_stats_task(void *arg)
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

// ─── GPIO Init ────────────────────────────────────────────────────────────────

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

// ─── Wi-Fi + ESP-NOW Init ─────────────────────────────────────────────────────

static void wifi_init(void)
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

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(action_send_cb));

    // Set PMK for unencrypted ESP-NOW (avoids key derivation delay)
    uint8_t pmk[] = "galena-pmk-1234";
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));

    esp_now_peer_info_t peer = { };
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_light_bar_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// ─── Main ─────────────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "Galena Action Ring booting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_init_pins();

    usb_phy_handle_t phy_hdl = NULL;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_FULL,
    };
    ESP_ERROR_CHECK(usb_new_phy(&phy_conf, &phy_hdl));

    // Initialize TinyUSB with CDC + HID
    const tusb_rhport_init_t rh_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    if (!tusb_rhport_init(0, &rh_init)) {
        ESP_LOGE(TAG, "TinyUSB init failed");
        return;
    }

    s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));
    s_hid_queue    = xQueueCreate(20, sizeof(hid_queue_item_t));

    gpio_set_direction(PIN_OSD_DEBUG, GPIO_MODE_OUTPUT);

    wifi_init();
    espnow_init();

    uint8_t my_mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_light_bar_mac[0], s_light_bar_mac[1], s_light_bar_mac[2],
             s_light_bar_mac[3], s_light_bar_mac[4], s_light_bar_mac[5]);

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
