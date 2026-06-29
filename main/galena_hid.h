#pragma once
#include <stdint.h>
#include "tusb.h"

#define GALENA_USB_VID      0x303A
#define GALENA_USB_PID      0x4005

#define GALENA_REPORT_SIZE  8

typedef enum {
    HID_EVT_BRIGHTNESS  = 1,
    HID_EVT_ENCODER    = 2,
    HID_EVT_BUTTON     = 3,
    HID_EVT_OSD_REQUEST = 4,
    HID_EVT_OSD_ACK     = 5,
    HID_EVT_LIGHT_STATE = 6,
} galena_hid_event_t;

typedef struct __attribute__((packed)) {
    uint8_t event_type;
    int8_t  event_value;
    uint8_t _reserved[6];
} galena_hid_report_t;

static const uint8_t galena_hid_report_descriptor[] = {
    0x06, 0x00, 0xFF,   // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,         // Usage (Vendor Usage 0x01)
    0xA1, 0x01,         // Collection (Application)
    0x15, 0x00,         //   Logical Minimum (0)
    0x26, 0xFF, 0x00,   //   Logical Maximum (255)
    0x75, 0x08,         //   Report Size (8 bits)
    0x95, 0x08,         //   Report Count (8)
    0x09, 0x01,         //   Usage (Input Data)
    0x81, 0x02,         //   Input (Data, Variable, Absolute)
    0x95, 0x01,         //   Report Count (1)
    0x09, 0x02,         //   Usage (Output Data)
    0xB1, 0x02,         //   Feature (Data, Variable, Absolute)
    0xC0                // End Collection
};

#define GALENA_HID_DESC_LEN  sizeof(galena_hid_report_descriptor)
