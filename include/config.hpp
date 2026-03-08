#pragma once

#include <Arduino.h>

// ── LED pins ────────────────────────────────────────────────
constexpr int LED1_PIN = PF9;
constexpr int LED2_PIN = PF10;

// ── Serial ──────────────────────────────────────────────────
constexpr uint32_t HOST_BAUD    = 115200;  // Serial  (report to host)
constexpr uint32_t CAMERA_BAUD  = 115200;  // Serial1 (camera UART)

// ── PCA9685 / Servo ─────────────────────────────────────────
constexpr uint8_t  PCA9685_ADDR      = 0x40;  // I2C address
constexpr uint16_t PCA9685_PWM_FREQ  = 50;    // Hz (standard servo)
constexpr uint8_t  SERVO_X_CHANNEL   = 0;     // PCA9685 channel for X axis
constexpr uint8_t  SERVO_Y_CHANNEL   = 1;     // PCA9685 channel for Y axis
constexpr float    SERVO_CENTER      = 90.0f; // degrees

// ── PID gains ───────────────────────────────────────────────
constexpr float KP = 0.8f;
constexpr float KI = 0.02f;
constexpr float KD = 0.3f;

// ── Control targets ─────────────────────────────────────────
constexpr float TARGET_X = 160.0f;
constexpr float TARGET_Y = 120.0f;

// ── Control loop ────────────────────────────────────────────
constexpr uint32_t CONTROL_FREQ_HZ = 60;  // Timer ISR frequency (adjustable)
