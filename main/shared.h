#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "galena_hid.h"
#include "galena_protocol.h"

// ─── Pin Definitions ─────────────────────────────────────────────────────────

#define PIN_ENC_A           GPIO_NUM_11
#define PIN_ENC_B           GPIO_NUM_12
#define PIN_BUTTON          GPIO_NUM_13

// ─── Shared Types ────────────────────────────────────────────────────────────

typedef struct {
    galena_hid_event_t type;
    int8_t             value;
} hid_queue_item_t;

typedef struct {
    galena_packet_t pkt;
} espnow_event_t;

// ─── Shared Globals ──────────────────────────────────────────────────────────

extern float            g_shadow_brightness;
extern SemaphoreHandle_t s_bri_mutex;
extern uint8_t          g_osd_mode;

extern QueueHandle_t    s_espnow_queue;
extern QueueHandle_t    s_hid_queue;

extern volatile int32_t g_send_ok;
extern volatile int32_t g_send_fail;
extern volatile uint32_t g_rx_count;
extern volatile uint32_t g_osd_report_count;
