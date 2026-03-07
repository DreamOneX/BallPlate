#include <Arduino.h>
#include "config.hpp"
#include "host_report.hpp"
#include "servo/arduino_servo.hpp"
#include "pid/pid_controller.hpp"
#include "position/uart_pos_provider.hpp"

ArduinoServo servoX(SERVO_X_PIN), servoY(SERVO_Y_PIN);
PIDController pidX(KP, KI, KD, TARGET_X);
PIDController pidY(KP, KI, KD, TARGET_Y);
UartPosProvider posProvider(Serial1, 115200);

unsigned long lastTime = 0;
static bool ledState = false;

void setup() {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);

    Serial.begin(115200);
    posProvider.begin();
    servoX.write(SERVO_CENTER);
    servoY.write(SERVO_CENTER);
    lastTime = millis();

    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
}

void loop() {
    Position pos;
    if (!posProvider.read(pos)) return;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    if (dt <= 0.0f || dt > 0.5f) {
        pidX.reset();
        pidY.reset();
        return;
    }

    float angleX = SERVO_CENTER + pidX.compute(pos.x, dt);
    float angleY = SERVO_CENTER + pidY.compute(pos.y, dt);

    servoX.write(constrain(angleX, 0.0f, 180.0f));
    servoY.write(constrain(angleY, 0.0f, 180.0f));

    REPORT_TO_HOST("X", pos.x, pidX, angleX);
    REPORT_TO_HOST("Y", pos.y, pidY, angleY);

    // Heartbeat LED
    ledState = !ledState;
    digitalWrite(LED1_PIN, ledState);
    digitalWrite(LED2_PIN, !ledState);
}