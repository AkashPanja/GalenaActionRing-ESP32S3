#pragma once
#include "tusb.h"

#define USBD_VID           0x303A
#define USBD_PID           0x4006

extern const tusb_desc_device_t galena_device_desc;
extern const char *galena_string_descs[];
extern const uint8_t galena_config_desc[];

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
