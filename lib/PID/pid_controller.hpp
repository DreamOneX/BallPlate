#pragma once

#include <Arduino.h>

namespace ball_plate {

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float target = 160.0f,
                  float out_min = -45.0f, float out_max = 45.0f)
        : _kp(kp), _ki(ki), _kd(kd),
          _target(target),
          _out_min(out_min), _out_max(out_max) {}

    /// Core PID computation; dt is in seconds
    float compute(float measured, float dt) {
        float error = _target - measured;

        // Integral — only accumulate when output is not saturated (conditional anti-windup)
        float tentative = _integral + error * dt;
        float p_term = _kp * error;
        float d_term = 0.0f;

        // Derivative — apply first-order low-pass filter to suppress high-frequency noise
        if (dt > 0.0f) {
            float raw_d = (error - _prev_error) / dt;
            _filtered_d += _alpha * (raw_d - _filtered_d);
            d_term = _kd * _filtered_d;
            _prev_error = error;
        }

        float output_unsat = p_term + _ki * tentative + d_term;
        float output = constrain(output_unsat, _out_min, _out_max);

        // Conditional anti-windup: update integral only if output is unsaturated or integral is unwinding
        if (output == output_unsat || (error * _ki * tentative < 0.0f))
            _integral = tentative;

        _last_error  = error;
        _last_output = output;
        return output;
    }

    void reset() {
        _integral = 0.0f;
        _prev_error = 0.0f;
        _filtered_d = 0.0f;
    }

    void setTarget(float t) { _target = t; }
    void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
    void setOutputLimits(float min, float max) { _out_min = min; _out_max = max; }

    /// Set derivative filter coefficient, range (0, 1]; smaller value means stronger filtering
    void setDerivativeAlpha(float a) { _alpha = constrain(a, 0.01f, 1.0f); }

    float target()     const { return _target; }
    float lastError()  const { return _last_error; }
    float lastOutput() const { return _last_output; }

private:
    float _kp, _ki, _kd;
    float _target;
    float _out_min, _out_max;
    float _integral    = 0.0f;
    float _prev_error  = 0.0f;
    float _filtered_d  = 0.0f;
    float _alpha       = 0.15f;  // Derivative low-pass filter coefficient
    float _last_error  = 0.0f;
    float _last_output = 0.0f;
};

} // namespace ball_plate