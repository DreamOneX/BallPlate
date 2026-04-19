#pragma once

#include <Arduino.h>
#include <HardwareTimer.h>
#include <cmath>
#include "servo_interface.hpp"

namespace ball_plate {

/// One hardware timer shared by one or more servo channels.
/// Typical servo frame: 20,000 us = 50 Hz.
class Stm32PwmTimer {
public:
    explicit Stm32PwmTimer(TIM_TypeDef* instance, uint32_t periodUs = 20000)
        : _timer(instance), _periodUs(periodUs) {}

    void begin() {
        if (_begun) return;

        _timer.pause();
        _timer.setOverflow(_periodUs, MICROSEC_FORMAT);
        _begun = true;
    }

    void start() {
        if (!_begun) begin();
        _timer.resume();
    }

    HardwareTimer& raw() { return _timer; }
    uint32_t periodUs() const { return _periodUs; }

private:
    HardwareTimer _timer;
    uint32_t _periodUs;
    bool _begun = false;
};

/// Hardware-PWM servo implementation for STM32duino / PlatformIO Arduino.
class Stm32PwmServo final : public IServo {
public:
    /// channel: 1..4
    /// pin: Arduino pin name, e.g. PE14
    /// pulse range: usually 1000..2000 us, some servos may tolerate 500..2500 us
    Stm32PwmServo(Stm32PwmTimer& timer,
                  uint32_t channel,
                  uint32_t pin,
                  uint32_t minPulseUs = 1000,
                  uint32_t maxPulseUs = 2000,
                  float minAngleDeg = 0.0f,
                  float maxAngleDeg = 180.0f)
        : _timer(timer),
          _channel(channel),
          _pin(pin),
          _minPulseUs(minPulseUs),
          _maxPulseUs(maxPulseUs),
          _minAngleDeg(minAngleDeg),
          _maxAngleDeg(maxAngleDeg) {}

    void begin() override {
        if (_attached) return;

        _timer.begin();

        // HardwareTimer will configure the pin AF for PWM output.
        _timer.raw().setMode(_channel, TIMER_OUTPUT_COMPARE_PWM1, _pin);

        _attached = true;
        write(90.0f);     // safe default center
        _timer.start();
    }

    void write(float angle) override {
        if (!_attached) return;

        float target = angle + _offsetDeg;
        target = clampf(target, _minAngleDeg, _maxAngleDeg);

        const uint32_t pulseUs = angleToPulseUs(target);
        _timer.raw().setCaptureCompare(_channel, pulseUs, MICROSEC_COMPARE_FORMAT);

        // Push CCR update immediately while timer is running.
        _timer.raw().refresh();

        _lastAngleDeg = target;
    }

    void setOffset(float offset) override {
        _offsetDeg = offset;
    }

    void end() override {
        if (!_attached) return;
        _timer.raw().pauseChannel(_channel);
        _attached = false;
    }

    void reattach() {
        if (_attached) return;
        _timer.raw().resumeChannel(_channel);
        _attached = true;
        write(_lastAngleDeg);
    }

    float lastAngle() const { return _lastAngleDeg; }
    bool attached() const { return _attached; }

private:
    static float clampf(float x, float lo, float hi) {
        return (x < lo) ? lo : (x > hi) ? hi : x;
    }

    uint32_t angleToPulseUs(float angleDeg) const {
        const float spanAngle = _maxAngleDeg - _minAngleDeg;
        const float spanPulse = static_cast<float>(_maxPulseUs - _minPulseUs);

        float t = 0.0f;
        if (spanAngle > 0.0f) {
            t = (angleDeg - _minAngleDeg) / spanAngle;
        }

        const float pulse = static_cast<float>(_minPulseUs) + t * spanPulse;
        return static_cast<uint32_t>(std::lround(pulse));
    }

    Stm32PwmTimer& _timer;
    uint32_t _channel;
    uint32_t _pin;

    uint32_t _minPulseUs;
    uint32_t _maxPulseUs;
    float _minAngleDeg;
    float _maxAngleDeg;

    float _offsetDeg = 0.0f;
    float _lastAngleDeg = 90.0f;
    bool _attached = false;
};

} // namespace ball_plate