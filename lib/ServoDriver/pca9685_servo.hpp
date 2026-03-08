#pragma once

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "servo_interface.hpp"

namespace ball_plate {

/// Concrete IServo backed by a PCA9685 PWM driver board (via I2C).
///
/// Each instance controls one channel on the PCA9685. Multiple instances
/// can share the same Adafruit_PWMServoDriver object (and thus the same
/// physical board) by passing a reference to the constructor.
class PCA9685Servo final : public IServo {
public:
    /// Default pulse-width limits that map 0-180° to 500-2500 µs.
    static constexpr uint16_t kDefaultMinPulseUs = 500;
    static constexpr uint16_t kDefaultMaxPulseUs = 2500;

    /// @param driver   Reference to an already-initialised Adafruit_PWMServoDriver.
    /// @param channel  PCA9685 channel number (0-15).
    /// @param minUs    Pulse width (µs) corresponding to 0°.
    /// @param maxUs    Pulse width (µs) corresponding to 180°.
    PCA9685Servo(Adafruit_PWMServoDriver& driver, uint8_t channel,
                 uint16_t minUs = kDefaultMinPulseUs,
                 uint16_t maxUs = kDefaultMaxPulseUs)
        : driver_(driver), channel_(channel), minUs_(minUs), maxUs_(maxUs) {}

    /// Write the target angle (degrees) with the configured offset applied.
    void write(float angle) override {
        float effective = angle + offset_;
        // Clamp to the valid servo range.
        if (effective < 0.0f)   effective = 0.0f;
        if (effective > 180.0f) effective = 180.0f;

        // Map angle (0-180°) to pulse width (µs), then to a 12-bit PWM tick.
        float pulseUs = minUs_ + (effective / 180.0f) * (maxUs_ - minUs_);
        driver_.writeMicroseconds(channel_, static_cast<uint16_t>(pulseUs));
    }

    /// Set an additive offset that is applied on every write().
    void setOffset(float offset) override { offset_ = offset; }

private:
    Adafruit_PWMServoDriver& driver_;
    uint8_t  channel_;
    uint16_t minUs_;
    uint16_t maxUs_;
    float    offset_ = 0.0f;
};

} // namespace ball_plate
