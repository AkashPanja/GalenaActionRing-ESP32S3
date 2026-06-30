#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "tusb.h"

void cdc_log(const char *fmt, ...)
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
