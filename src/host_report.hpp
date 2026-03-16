#pragma once

#include <Arduino.h>
#include <config.hpp>
#include <pid_controller.hpp>

namespace ball_plate {

inline void reportToHost(const char* label, float curr,
                         const PIDController& pid, float angle) {
    if constexpr (ENABLE_HOST_REPORT) {
        Serial.print(label);               Serial.print(':');
        Serial.print(curr, 2);             Serial.print(',');
        Serial.print(pid.target(), 2);     Serial.print(',');
        Serial.print(pid.lastError(), 2);  Serial.print(',');
        Serial.print(pid.lastOutput(), 2); Serial.print(',');
        Serial.println(angle, 2);
}
    }

} // namespace ball_plate
