#pragma once
#include <stdint.h>

// ─── Packet Types ────────────────────────────────────────────────────────────
#define PKT_BRIGHTNESS   0x01   // Action Ring ← Light Bar : brightness 0–100
#define PKT_ENCODER      0x02   // Light Bar   ← Action Ring : encoder delta
#define PKT_BUTTON       0x03   // Light Bar   ← Action Ring : button state
#define PKT_HEARTBEAT    0x04   // Light Bar   ← Action Ring : periodic ping (val=0)
#define PKT_LIGHT_STATE  0x05   // Action Ring ← Light Bar : 0=off, 1=on
#define PKT_BOOT         0x06   // Action Ring ← Light Bar : boot/reset notification

// ─── MAC Addresses ───────────────────────────────────────────────────────────
#define LIGHT_BAR_MAC_B0 0x88
#define LIGHT_BAR_MAC_B1 0x56
#define LIGHT_BAR_MAC_B2 0xA6
#define LIGHT_BAR_MAC_B3 0x2A
#define LIGHT_BAR_MAC_B4 0x43
#define LIGHT_BAR_MAC_B5 0xB0
#define LIGHT_BAR_MAC    {LIGHT_BAR_MAC_B0, LIGHT_BAR_MAC_B1, LIGHT_BAR_MAC_B2, LIGHT_BAR_MAC_B3, LIGHT_BAR_MAC_B4, LIGHT_BAR_MAC_B5}

#define ACTION_RING_MAC_B0 0xE0
#define ACTION_RING_MAC_B1 0x72
#define ACTION_RING_MAC_B2 0xA1
#define ACTION_RING_MAC_B3 0xE9
#define ACTION_RING_MAC_B4 0x39
#define ACTION_RING_MAC_B5 0x44
#define ACTION_RING_MAC    {ACTION_RING_MAC_B0, ACTION_RING_MAC_B1, ACTION_RING_MAC_B2, ACTION_RING_MAC_B3, ACTION_RING_MAC_B4, ACTION_RING_MAC_B5}

// ─── Shared Packet ───────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  type;      // PKT_* above
    int32_t  value;     // brightness*100 | encoder delta | button state
    uint8_t  osd_show;  // 1=brightness overlay showing on PC
    uint8_t  connected; // 1=USB connected to PC, 0=standalone
} galena_packet_t;
