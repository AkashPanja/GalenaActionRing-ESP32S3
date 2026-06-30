#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared.h"
#include "button.h"

#define DEBOUNCE_MS         50
#define HOLD_MS             700

extern void espnow_send_button(int state);

void button_task(void *arg)
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
