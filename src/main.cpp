#include <Arduino.h>
#include <config.hpp>
#include "host_report.hpp"
#include <pca9685_servo.hpp>
#include <pid_controller.hpp>
#include "position/uart_pos_provider.hpp"
#include "control_loop.hpp"

using namespace ball_plate;

// ── Hardware ────────────────────────────────────────────────
static Adafruit_PWMServoDriver pwmDriver(PCA9685_ADDR);
static PCA9685Servo realServoX(pwmDriver, SERVO_X_CHANNEL);
static PCA9685Servo realServoY(pwmDriver, SERVO_Y_CHANNEL);

static IServo* servoX = &realServoX;
static IServo* servoY = &realServoY;

static PIDController pidX(KP, KI, KD, TARGET_X);
static PIDController pidY(KP, KI, KD, TARGET_Y);


static UartPosProvider realPosProvider(Serial1, CAMERA_BAUD);
static IPosProvider* posProvider = &realPosProvider;

static ControlLoop control(*posProvider, pidX, pidY, *servoX, *servoY);
static bool ledState = false;

void setup() {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);

    Serial.begin(HOST_BAUD);
    posProvider->begin();

    Wire.begin();
    pwmDriver.begin();
    pwmDriver.setPWMFreq(PCA9685_PWM_FREQ);

    servoX->write(SERVO_CENTER);
    servoY->write(SERVO_CENTER);

    control.begin();  // start Timer ISR at CONTROL_FREQ_HZ

    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
}

void loop() {
    // Drain UART buffer → update latest position (main context, not ISR)
    posProvider->update();

    // When ISR has computed new PID output, write servos (I2C) and report
    if (control.applyOutputs()) {
        const auto& out = control.output();
        REPORT_TO_HOST("X", out.posX, pidX, out.angleX);
        REPORT_TO_HOST("Y", out.posY, pidY, out.angleY);

        ledState = !ledState;
        digitalWrite(LED1_PIN, ledState);
        digitalWrite(LED2_PIN, !ledState);
    }
}