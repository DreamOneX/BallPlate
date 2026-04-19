#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <config.hpp>
#include "HardwareSerial.h"
#include <Servo.h>
#include <servo_interface.hpp>
#include <stm32_pwm_servo.hpp>
#include <pid_controller.hpp>
#include "position/uart_pos_provider.hpp"
#include "control_loop.hpp"
#include <command_dispatcher.hpp>
#include <serial_receiver.hpp>
#include "remote/remote_handler.hpp"
#include "wiring_constants.h"

using namespace ball_plate;

// ── Hardware ────────────────────────────────────────────────
// static Adafruit_PWMServoDriver pwmDriver(PCA9685_ADDR);
// static PCA9685Servo realServoX(pwmDriver, SERVO_X_CHANNEL);
// static PCA9685Servo realServoY(pwmDriver, SERVO_Y_CHANNEL);

static HardwareSerial Serial_1(PA9, PA10);
static HardwareSerial Serial_2(PA3, PA2);
static HardwareSerial OpenMVSerial(PC11, PC10);

static HardwareSerial& dbgSerial = Serial_1;
static HardwareSerial& posSerial = OpenMVSerial;
static HardwareSerial& remoteSerial = Serial_2;

// static ArduinoServo realServoX(SERVO_X_PIN);
// static ArduinoServo realServoY(SERVO_Y_PIN);

// X: TIM1_CH4
static Stm32PwmTimer servoTimerX(TIM1, 20000);
// Y: TIM4_CH3
static Stm32PwmTimer servoTimerY(TIM4, 20000);

static Stm32PwmServo realServoX(servoTimerX, 4, SERVO_X_PIN, 1000, 2000);
static Stm32PwmServo realServoY(servoTimerY, 3, SERVO_Y_PIN, 1000, 2000);

static IServo& servoX = realServoX;
static IServo& servoY = realServoY;

static PIDController pidX(KP, KI, KD, CENTER_X);
static PIDController pidY(KP, KI, KD, CENTER_Y);

static UartPosProvider realPosProvider(posSerial, CAMERA_BAUD);
static IPosProvider& posProvider = realPosProvider;

static ControlLoop control(posProvider, pidX, pidY, servoX, servoY);
static bool ledState = false;

// ── Remote control ─────────────────────────────────────────
static CommandDispatcher dispatcher;
static SerialReceiver   remoteReceiver(remoteSerial, HOST_BAUD);

inline void reportToHost(const char* label, float curr,
                         const PIDController& pid, float angle) {
    if constexpr (ENABLE_HOST_REPORT) {
        dbgSerial.print(label);               dbgSerial.print(':');
        dbgSerial.print(curr, 2);             dbgSerial.print(',');
        dbgSerial.print(pid.target(), 2);     dbgSerial.print(',');
        dbgSerial.print(pid.lastError(), 2);  dbgSerial.print(',');
        dbgSerial.print(pid.lastOutput(), 2); dbgSerial.print(',');
        dbgSerial.println(angle, 2);
    }
}

void setup() {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);

    pinMode(PF2, OUTPUT);
    digitalWrite(PF2, HIGH);

    for (int i = 0; i < 5; i++) {
        digitalWrite(LED1_PIN, HIGH);
        delay(100);
        digitalWrite(LED1_PIN, LOW);
        delay(100);
        digitalWrite(LED2_PIN, HIGH);
        delay(100);
        digitalWrite(LED2_PIN, LOW);
        delay(100);
    }


    servoX.begin();
    servoY.begin();
    servoX.write(SERVO_CENTER);
    servoY.write(SERVO_CENTER);

    Serial_1.begin(HOST_BAUD);
    Serial_1.println("Hello World");
    // Serial_2.begin(HOST_BAUD);
    OpenMVSerial.begin(HOST_BAUD);
    posProvider.begin();

    // Wire.begin();
    // pwmDriver.begin();
    // pwmDriver.setPWMFreq(PCA9685_PWM_FREQ);


    control.begin();  // start Timer ISR at CONTROL_FREQ_HZ

    if constexpr (ENABLE_REMOTE) {
        setupRemoteHandlers(dispatcher, control);
    }

    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
}

void loop() {
    // Serial_1.println("Hello World");
    // Drain UART buffer → update latest position (main context, not ISR)
    posProvider.update();

    if constexpr (ENABLE_REMOTE) {
        dispatcher.poll(remoteReceiver);
    }

    // When ISR has computed new PID output, write servos (I2C) and report
    if (control.applyOutputs()) {
        const auto& out = control.output();
        // reportToHost("X", out.posX, pidX, out.angleX);
        // reportToHost("Y", out.posY, pidY, out.angleY);
    }

    ledState = !ledState;
    digitalWrite(LED1_PIN, ledState);
    digitalWrite(LED2_PIN, !ledState);
}