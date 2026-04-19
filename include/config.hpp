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

// ── Arduino Servo ─────────────────────────────────────────
constexpr int SERVO_X_PIN = PE14;
constexpr int SERVO_Y_PIN = PD14;

// ── PID gains ───────────────────────────────────────────────
constexpr float KP = 0.5f;
constexpr float KI = 0.02f;
constexpr float KD = 0.3f;

// ── Control targets ─────────────────────────────────────────
constexpr float WIDTH = 320.0f;
constexpr float HEIGHT = 240.0f;
// constexpr float CENTER_X = WIDTH / 2.0f;
// constexpr float CENTER_Y = HEIGHT / 2.0f;
constexpr float CENTER_X = 201.0f;
constexpr float CENTER_Y = 84.0f;

// ── Control loop ────────────────────────────────────────────
constexpr uint32_t CONTROL_FREQ_HZ = 60;  // Timer ISR frequency (adjustable)

// ── Host reporting ─────────────────────────────────────────
constexpr bool ENABLE_HOST_REPORT = true;  // CSV reporting to host via Serial

// ── Remote control ─────────────────────────────────────────
constexpr bool ENABLE_REMOTE       = false;  // set true when ICommandReceiver impl is available
constexpr bool ALLOW_REMOTE_RESUME = true;   // false = disable wireless resume, require hardware reset
