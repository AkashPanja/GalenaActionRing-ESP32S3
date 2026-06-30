#pragma once

void wifi_init(void);
void espnow_init(void);
void espnow_send_encoder(int delta);
void espnow_send_button(int state);
void espnow_task(void *arg);
void heartbeat_task(void *arg);
void osd_poll_task(void *arg);
void espnow_stats_task(void *arg);
