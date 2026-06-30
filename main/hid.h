#pragma once
#include <stdint.h>
#include "galena_hid.h"

void hid_send_event(galena_hid_event_t type, int8_t value);
void hid_tx_task(void *arg);
