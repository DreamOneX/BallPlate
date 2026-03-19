//! Firmware constants mirroring `include/config.hpp`.
//!
//! Hardware pin assignments are configured in `main.rs`:
//! - I2C1:  PB6 (SCL), PB7 (SDA), DMA1_CH6 (TX), DMA1_CH0 (RX)
//! - Camera UART (USART1): PA10 (RX), DMA2_CH2 (RX DMA ring buffer)
//! - Host UART (USART2):   PA2 (TX), PA3 (RX), buffered (interrupt-driven)
//! - LEDs: PF9, PF10

// ── Serial ──────────────────────────────────────────────────
pub const HOST_BAUD: u32 = 115_200;
pub const CAMERA_BAUD: u32 = 115_200;

// ── PCA9685 / Servo ─────────────────────────────────────────
pub const PCA9685_ADDR: u8 = 0x40;
pub const PCA9685_PWM_FREQ_HZ: u16 = 50;

pub const SERVO_X_CHANNEL: u8 = 0;
pub const SERVO_Y_CHANNEL: u8 = 1;

pub const SERVO_CENTER_DEG: f32 = 90.0;

// Map 0..180° to 500..2500µs (matches `PCA9685Servo` defaults in C++).
pub const SERVO_MIN_PULSE_US: u16 = 500;
pub const SERVO_MAX_PULSE_US: u16 = 2500;

// TODO(calib): per-axis mechanical offset (deg)
pub const SERVO_X_OFFSET_DEG: f32 = 0.0;
pub const SERVO_Y_OFFSET_DEG: f32 = 0.0;

// ── PID gains ───────────────────────────────────────────────
pub const KP: f32 = 0.8;
pub const KI: f32 = 0.02;
pub const KD: f32 = 0.3;

// ── Control targets ─────────────────────────────────────────
pub const WIDTH: f32 = 320.0;
pub const HEIGHT: f32 = 240.0;

pub const CENTER_X: f32 = WIDTH / 2.0;
pub const CENTER_Y: f32 = HEIGHT / 2.0;

// ── Control loop ────────────────────────────────────────────
pub const CONTROL_FREQ_HZ: u32 = 60;
