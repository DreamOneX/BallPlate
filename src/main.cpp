#include <Arduino.h>
#include <Servo.h>
#include "pid_controller.hpp"
#include "uart_pos_provider.hpp"

// ── Host reporting ──────────────────────────────────────────
// Uncomment the next line to enable CSV reporting to host via Serial
#define ENABLE_HOST_REPORT

#ifdef ENABLE_HOST_REPORT
#define REPORT_TO_HOST(label, curr, pid, angle)  do { \
    Serial.print(label);       Serial.print(':');  \
    Serial.print(curr,   2);   Serial.print(',');  \
    Serial.print((pid).target(),    2); Serial.print(','); \
    Serial.print((pid).lastError(),  2); Serial.print(','); \
    Serial.print((pid).lastOutput(), 2); Serial.print(','); \
    Serial.println(angle, 2);                       \
} while(0)
#else
#define REPORT_TO_HOST(label, curr, pid, angle) ((void)0)
#endif

constexpr int LED1_PIN = PF9;
constexpr int LED2_PIN = PF10;

constexpr int SERVO_X_PIN = PA0;
constexpr int SERVO_Y_PIN = PA1;

constexpr float SERVO_CENTER = 90.0f;

constexpr float KP = 0.8f;
constexpr float KI = 0.02f;
constexpr float KD = 0.3f;

constexpr float TARGET_X = 160.0f;
constexpr float TARGET_Y = 120.0f;

Servo servoX, servoY;
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
    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
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