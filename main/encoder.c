#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared.h"
#include "encoder.h"

#define ENCODER_RESOLUTION  2
#define MAX_ENCODER_DELTA   10
#define ENCODER_DEBOUNCE_MS 50

extern void espnow_send_encoder(int delta);

void encoder_task(void *arg)
{
    int8_t enc_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    uint8_t last_ab = (gpio_get_level(PIN_ENC_A) << 1) |
                       gpio_get_level(PIN_ENC_B);
    int     position       = 0;
    int     last_sent      = 0;
    int     initialized    = 0;
    TickType_t last_send_tick = 0;

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
                TickType_t now = xTaskGetTickCount();
                if (now - last_send_tick >= pdMS_TO_TICKS(ENCODER_DEBOUNCE_MS)) {
                    espnow_send_encoder(delta);
                    last_send_tick = now;
                }
                last_sent = detent_pos;
            } else {
                last_sent = detent_pos;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
