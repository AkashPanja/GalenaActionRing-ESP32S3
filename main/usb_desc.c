#include "tusb.h"
#include "class/hid/hid_device.h"

#include "galena_hid.h"
#include "usb_desc.h"

const tusb_desc_device_t galena_device_desc = {
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

const char *galena_string_descs[] = {
    (const char[]){ 0x09, 0x04 },
    "Anode Labs",
    "Galena Action Ring",
    "GAR-001",
};

const uint8_t galena_config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, ITF_NUM_CDC_DATA, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, false,
                       sizeof(galena_hid_report_descriptor),
                       EP_HID_IN, 64, 1),
};
