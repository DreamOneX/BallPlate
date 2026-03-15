#pragma once

#include <HardwareTimer.h>
#include <config.hpp>
#include <pid_controller.hpp>
#include <remote_command.hpp>
#include <servo_interface.hpp>
#include "position/pos_provider.hpp"

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
        if (_stopped) {
            if (_stopMode == StopMode::PARK && !_parkWritten) {
                _servoX.write(SERVO_CENTER);
                _servoY.write(SERVO_CENTER);
                _parkWritten = true;
            }
            _output.ready = false;
            return false;
        }

        if (!_output.ready) return false;

        _servoX.write(constrain(_output.angleX, 0.0f, 180.0f));
        _servoY.write(constrain(_output.angleY, 0.0f, 180.0f));
        _output.ready = false;
        return true;
    }

    /// Read-only access to the latest computed output (for reporting)
    const ControlOutput& output() const { return _output; }

    // ── Remote stop / resume ───────────────────────────────

    /// Enter stopped state (called from main loop handler).
    /// This is a control-layer soft stop, NOT a hardware E-stop.
    /// Worst-case latency: loop period + 1/CONTROL_FREQ_HZ + I2C write time.
    void remoteStop(StopMode mode) {
        _stopMode    = mode;
        _parkWritten = false;
        _stopped     = true;
    }

    /// Request resume (actual PID reset happens in ISR context to avoid races).
    void requestResume() {
        _pendingResume = true;
    }

    /// Request target update (ISR consumes on next cycle).
    void requestTargetUpdate(float x, float y) {
        noInterrupts();
        _pendingTargetX = x;
        _pendingTargetY = y;
        _pendingTargetUpdate = true;
        interrupts();
    }

    bool stopped() const { return _stopped; }

private:
    void isr() {
        // 1. Consume pending resume (checked before stop gate)
        if (_pendingResume) {
            _pidX.reset();
            _pidY.reset();
            _stopped = false;
            _pendingResume = false;
        }

        // 2. Stop gate
        if (_stopped) return;

        // 3. Consume pending target update
        if (_pendingTargetUpdate) {
            _pidX.setTarget(_pendingTargetX);
            _pidY.setTarget(_pendingTargetY);
            _pendingTargetUpdate = false;
        }

        // 4. PID computation
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

    IPosProvider&   _pos;
    PIDController&  _pidX;
    PIDController&  _pidY;
    IServo&         _servoX;
    IServo&         _servoY;
    HardwareTimer*  _timer = nullptr;
    uint32_t        _freq  = CONTROL_FREQ_HZ;

    ControlOutput _output = {};

    // ── Stop state ──
    volatile bool     _stopped     = false;
    volatile StopMode _stopMode    = StopMode::FREEZE;
    bool              _parkWritten = false;

    // ── Pending requests (main loop writes, ISR reads) ──
    volatile bool  _pendingTargetUpdate = false;
    volatile float _pendingTargetX      = 0.0f;
    volatile float _pendingTargetY      = 0.0f;
    volatile bool  _pendingResume       = false;
};

} // namespace ball_plate
