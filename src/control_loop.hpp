#pragma once

#include <HardwareTimer.h>
#include <config.hpp>
#include <pid_controller.hpp>
#include "position/uart_pos_provider.hpp"
#include <servo_interface.hpp>

namespace ball_plate {

/// Data computed by the Timer ISR, consumed by loop()
struct ControlOutput {
    float posX, posY;
    float angleX, angleY;
    volatile bool ready;  // set by ISR, cleared by loop()
};

/// Runs PID at a fixed frequency via Timer ISR.
///
/// Because the servos use PCA9685 (I2C), actual servo writes CANNOT happen
/// inside the ISR.  The ISR computes PID outputs and caches them; call
/// applyOutputs() from loop() to write the servos and report to the host.
class ControlLoop {
public:
    ControlLoop(IPosProvider& pos,
                PIDController& pidX, PIDController& pidY,
                IServo& servoX, IServo& servoY)
        : _pos(pos), _pidX(pidX), _pidY(pidY),
          _servoX(servoX), _servoY(servoY) {}

    /// Start the timer ISR.  Default timer is TIM7 (basic timer, no PWM conflict).
    void begin(uint32_t freqHz = CONTROL_FREQ_HZ, TIM_TypeDef* tim = TIM7) {
        _freq = freqHz;
        _timer = new HardwareTimer(tim);
        _timer->setOverflow(freqHz, HERTZ_FORMAT);
        _timer->attachInterrupt(std::bind(&ControlLoop::isr, this));
        _timer->resume();
    }

    /// Change control frequency at runtime
    void setFrequency(uint32_t freqHz) {
        _freq = freqHz;
        if (_timer) {
            _timer->setOverflow(freqHz, HERTZ_FORMAT);
        }
    }

    /// Call from loop(): if ISR produced new output, write servos and return true.
    bool applyOutputs() {
        if (!_output.ready) return false;

        _servoX.write(constrain(_output.angleX, 0.0f, 180.0f));
        _servoY.write(constrain(_output.angleY, 0.0f, 180.0f));
        _output.ready = false;
        return true;
    }

    /// Read-only access to the latest computed output (for reporting)
    const ControlOutput& output() const { return _output; }

private:
    void isr() {
        const float dt = 1.0f / _freq;

        Position pos;
        if (!_pos.getLatest(pos)) return;  // no new camera data

        float angleX = SERVO_CENTER + _pidX.compute(pos.x, dt);
        float angleY = SERVO_CENTER + _pidY.compute(pos.y, dt);

        _output.posX   = pos.x;
        _output.posY   = pos.y;
        _output.angleX = angleX;
        _output.angleY = angleY;
        _output.ready  = true;
    }

    IPosProvider& _pos;
    PIDController&   _pidX;
    PIDController&   _pidY;
    IServo&  _servoX;
    IServo&  _servoY;
    HardwareTimer*   _timer = nullptr;
    uint32_t         _freq  = CONTROL_FREQ_HZ;

    ControlOutput _output = {};
};

} // namespace ball_plate
