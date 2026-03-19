#![no_std]

/// PID controller with conditional anti-windup and a first-order low-pass filter on the
/// derivative term.
///
/// Direct port of `lib/PID/pid_controller.hpp`.
#[derive(Debug, Clone, Copy)]
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,

    target: f32,
    out_min: f32,
    out_max: f32,

    integral: f32,
    prev_error: f32,
    filtered_d: f32,
    alpha: f32,

    last_error: f32,
    last_output: f32,
}

impl PidController {
    /// Create a PID controller with default output limits (-45..45).
    pub const fn new(kp: f32, ki: f32, kd: f32, target: f32) -> Self {
        Self::with_limits(kp, ki, kd, target, -45.0, 45.0)
    }

    pub const fn with_limits(kp: f32, ki: f32, kd: f32, target: f32, out_min: f32, out_max: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            target,
            out_min,
            out_max,
            integral: 0.0,
            prev_error: 0.0,
            filtered_d: 0.0,
            alpha: 0.15,
            last_error: 0.0,
            last_output: 0.0,
        }
    }

    /// Core PID computation; `dt` is in seconds.
    pub fn compute(&mut self, measured: f32, dt: f32) -> f32 {
        let error = self.target - measured;

        // Integral — only accumulate when output is not saturated (conditional anti-windup)
        let tentative = self.integral + error * dt;
        let p_term = self.kp * error;
        let mut d_term = 0.0;

        // Derivative — apply first-order low-pass filter to suppress high-frequency noise
        if dt > 0.0 {
            let raw_d = (error - self.prev_error) / dt;
            self.filtered_d += self.alpha * (raw_d - self.filtered_d);
            d_term = self.kd * self.filtered_d;
            self.prev_error = error;
        }

        let output_unsat = p_term + self.ki * tentative + d_term;
        let output = output_unsat.clamp(self.out_min, self.out_max);

        // Conditional anti-windup: update integral only if output is unsaturated or integral is unwinding
        if output == output_unsat || (error * self.ki * tentative < 0.0) {
            self.integral = tentative;
        }

        self.last_error = error;
        self.last_output = output;
        output
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.filtered_d = 0.0;
    }

    pub fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.out_min = min;
        self.out_max = max;
    }

    /// Set derivative filter coefficient, range (0.01, 1.0].
    /// Smaller value means stronger filtering.
    pub fn set_derivative_alpha(&mut self, a: f32) {
        self.alpha = a.clamp(0.01, 1.0);
    }

    pub fn target(&self) -> f32 {
        self.target
    }

    pub fn last_error(&self) -> f32 {
        self.last_error
    }

    pub fn last_output(&self) -> f32 {
        self.last_output
    }
}

#[cfg(test)]
mod tests {
    use super::PidController;

    const DT: f32 = 0.01;

    fn assert_f32_within(tol: f32, expected: f32, actual: f32) {
        let diff = (expected - actual).abs();
        assert!(diff <= tol, "expected {expected}, got {actual}, diff {diff} > tol {tol}");
    }

    #[test]
    fn test_initial_accessors() {
        let pid = PidController::new(1.0, 0.0, 0.0, 100.0);
        assert_eq!(100.0, pid.target());
        assert_eq!(0.0, pid.last_error());
        assert_eq!(0.0, pid.last_output());
    }

    #[test]
    fn test_p_only_positive_error() {
        let mut pid = PidController::new(2.0, 0.0, 0.0, 100.0);
        let out = pid.compute(90.0, DT);
        assert_eq!(10.0, pid.last_error());
        assert_eq!(20.0, out);
    }

    #[test]
    fn test_p_only_negative_error() {
        let mut pid = PidController::new(2.0, 0.0, 0.0, 100.0);
        let out = pid.compute(110.0, DT);
        assert_eq!(-10.0, pid.last_error());
        assert_eq!(-20.0, out);
    }

    #[test]
    fn test_p_only_zero_error() {
        let mut pid = PidController::new(2.0, 0.0, 0.0, 100.0);
        let out = pid.compute(100.0, DT);
        assert_eq!(0.0, out);
    }

    #[test]
    fn test_integral_accumulates() {
        let mut pid = PidController::new(0.0, 1.0, 0.0, 100.0);

        let out1 = pid.compute(90.0, DT); // integral = 0.1
        assert_f32_within(0.01, 0.1, out1);

        let out2 = pid.compute(90.0, DT); // integral = 0.2
        assert_f32_within(0.01, 0.2, out2);
    }

    #[test]
    fn test_derivative_responds_to_error_change() {
        let mut pid = PidController::new(0.0, 0.0, 1.0, 100.0);
        pid.set_derivative_alpha(1.0);

        pid.compute(90.0, DT); // deriv = (10-0)/0.01 = 1000
        let out1 = pid.last_output();
        assert!(out1 > 0.0);

        let out2 = pid.compute(90.0, DT); // deriv should be 0
        assert_f32_within(0.01, 0.0, out2);
    }

    #[test]
    fn test_output_clamped_to_limits() {
        let mut pid = PidController::with_limits(10.0, 0.0, 0.0, 100.0, -5.0, 5.0);
        let out = pid.compute(50.0, DT);
        assert_eq!(5.0, out);

        let out = pid.compute(150.0, DT);
        assert_eq!(-5.0, out);
    }

    #[test]
    fn test_custom_output_limits() {
        let mut pid = PidController::new(1.0, 0.0, 0.0, 100.0);
        pid.set_output_limits(-10.0, 10.0);

        let out = pid.compute(0.0, DT);
        assert_eq!(10.0, out);
    }

    #[test]
    fn test_anti_windup_prevents_integral_blowup() {
        let mut pid = PidController::with_limits(0.0, 1.0, 0.0, 100.0, -1.0, 1.0);

        for _ in 0..200 {
            pid.compute(0.0, DT);
        }

        let mut out = 0.0;
        for _ in 0..10 {
            out = pid.compute(200.0, DT);
        }

        assert_eq!(-1.0, out);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 100.0);
        pid.compute(80.0, DT);
        pid.compute(80.0, DT);
        pid.reset();

        let mut fresh = PidController::new(1.0, 1.0, 1.0, 100.0);
        let out_reset = pid.compute(80.0, DT);
        let out_fresh = fresh.compute(80.0, DT);
        assert_eq!(out_fresh, out_reset);
    }

    #[test]
    fn test_set_target() {
        let mut pid = PidController::new(1.0, 0.0, 0.0, 100.0);
        pid.set_target(50.0);
        assert_eq!(50.0, pid.target());

        let out = pid.compute(50.0, DT);
        assert_eq!(0.0, out);
    }

    #[test]
    fn test_set_gains() {
        let mut pid = PidController::new(1.0, 0.0, 0.0, 100.0);
        pid.set_gains(3.0, 0.0, 0.0);

        let out = pid.compute(90.0, DT);
        assert_eq!(30.0, out);
    }

    #[test]
    fn test_zero_dt_skips_derivative() {
        let mut pid = PidController::new(1.0, 0.0, 1.0, 100.0);
        let out = pid.compute(90.0, 0.0);
        assert_eq!(10.0, out);
    }
}
