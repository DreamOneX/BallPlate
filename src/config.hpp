#pragma once

#include <Arduino.h>

// ── LED pins ────────────────────────────────────────────────
constexpr int LED1_PIN = PF9;
constexpr int LED2_PIN = PF10;

// ── Servo pins ──────────────────────────────────────────────
constexpr int SERVO_X_PIN = PA0;
constexpr int SERVO_Y_PIN = PA1;

// ── Servo parameters ────────────────────────────────────────
constexpr float SERVO_CENTER = 90.0f;

// ── PID gains ───────────────────────────────────────────────
constexpr float KP = 0.8f;
constexpr float KI = 0.02f;
constexpr float KD = 0.3f;

// ── Control targets ─────────────────────────────────────────
constexpr float TARGET_X = 160.0f;
constexpr float TARGET_Y = 120.0f;
