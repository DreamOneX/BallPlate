#pragma once

#include <Servo.h>
#include "servo_interface.hpp"

namespace ball_plate {

/// Concrete IServo backed by the Arduino Servo library.
class ArduinoServo final : public IServo {
public:
    explicit ArduinoServo(int pin) : _pin(pin) {}

    void begin() override {
        _servo.attach(_pin);
    }

    /// Write the target angle (degrees) with the configured offset applied.
    void write(float angle) override {
        _servo.write(static_cast<int>(angle + offset_));
    }

    /// Set an additive offset that is applied on every write().
    void setOffset(float offset) override { offset_ = offset; }

    void end() override {
        _servo.detach();
    }

private:
    int _pin;
    float  offset_ = 0.0f;

public:
    Servo _servo;
};

} // namespace ball_plate
