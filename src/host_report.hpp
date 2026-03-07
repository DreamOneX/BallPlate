#pragma once

#include <Arduino.h>

// ── Host reporting ──────────────────────────────────────────
// Uncomment the next line to enable CSV reporting to host via Serial
#define ENABLE_HOST_REPORT

#ifdef ENABLE_HOST_REPORT
#define REPORT_TO_HOST(label, curr, pid, angle)  do { \
    Serial.print(label);       Serial.print(':');  \
    Serial.print(curr,   2);   Serial.print(',');  \
    Serial.print((pid).target(),    2); Serial.print(','); \
    Serial.print((pid).lastError(),  2); Serial.print(','); \
    Serial.print((pid).lastOutput(), 2); Serial.print(','); \
    Serial.println(angle, 2);                       \
} while(0)
#else
#define REPORT_TO_HOST(label, curr, pid, angle) ((void)0)
#endif
