#include <unity.h>
#include <pid_controller.hpp>

// ── Helper ──────────────────────────────────────────────────
static constexpr float DT = 0.01f;  // 10 ms step

void setUp()    {}   // Unity required
void tearDown() {}   // Unity required

// ── Initial state ───────────────────────────────────────────

void test_initial_accessors() {
    PIDController pid(1.0f, 0.0f, 0.0f, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pid.target());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastError());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastOutput());
}

// ── Proportional ────────────────────────────────────────────

void test_p_only_positive_error() {
    PIDController pid(2.0f, 0.0f, 0.0f, 100.0f);
    float out = pid.compute(90.0f, DT);  // error = +10
    TEST_ASSERT_EQUAL_FLOAT(10.0f, pid.lastError());
    TEST_ASSERT_EQUAL_FLOAT(20.0f, out);  // Kp * error = 2 * 10
}

void test_p_only_negative_error() {
    PIDController pid(2.0f, 0.0f, 0.0f, 100.0f);
    float out = pid.compute(110.0f, DT);  // error = -10
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, pid.lastError());
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, out);
}

void test_p_only_zero_error() {
    PIDController pid(2.0f, 0.0f, 0.0f, 100.0f);
    float out = pid.compute(100.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out);
}

// ── Integral ────────────────────────────────────────────────

void test_integral_accumulates() {
    PIDController pid(0.0f, 1.0f, 0.0f, 100.0f);

    // Step 1: error = 10, integral = 10 * 0.01 = 0.1
    float out1 = pid.compute(90.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, out1);

    // Step 2: same error, integral = 0.2
    float out2 = pid.compute(90.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, out2);
}

// ── Derivative ──────────────────────────────────────────────

void test_derivative_responds_to_error_change() {
    // Use alpha=1 to disable the low-pass filter for predictable results
    PIDController pid(0.0f, 0.0f, 1.0f, 100.0f);
    pid.setDerivativeAlpha(1.0f);

    // Step 1: error changes from 0 → 10
    pid.compute(90.0f, DT);  // deriv = (10 - 0) / 0.01 = 1000
    float out1 = pid.lastOutput();
    TEST_ASSERT_TRUE(out1 > 0.0f);

    // Step 2: error stays at 10 → derivative should be 0
    float out2 = pid.compute(90.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out2);
}

// ── Output clamping ─────────────────────────────────────────

void test_output_clamped_to_limits() {
    PIDController pid(10.0f, 0.0f, 0.0f, 100.0f, -5.0f, 5.0f);
    float out = pid.compute(50.0f, DT);  // Kp * 50 = 500, clamped to 5
    TEST_ASSERT_EQUAL_FLOAT(5.0f, out);

    out = pid.compute(150.0f, DT);  // Kp * (-50) = -500, clamped to -5
    TEST_ASSERT_EQUAL_FLOAT(-5.0f, out);
}

void test_custom_output_limits() {
    PIDController pid(1.0f, 0.0f, 0.0f, 100.0f);
    pid.setOutputLimits(-10.0f, 10.0f);

    float out = pid.compute(0.0f, DT);  // error = 100, clamped to 10
    TEST_ASSERT_EQUAL_FLOAT(10.0f, out);
}

// ── Anti-windup ─────────────────────────────────────────────

void test_anti_windup_prevents_integral_blowup() {
    PIDController pid(0.0f, 1.0f, 0.0f, 100.0f, -1.0f, 1.0f);

    // Saturate for many steps with positive error
    for (int i = 0; i < 200; i++)
        pid.compute(0.0f, DT);

    // Flip error sign — with anti-windup the integral should NOT have
    // accumulated to a huge value, so output should reach -1 within a
    // few steps (without anti-windup it would take hundreds of steps).
    float out = 0.0f;
    for (int i = 0; i < 10; i++)
        out = pid.compute(200.0f, DT);

    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out);
}

// ── Reset ───────────────────────────────────────────────────

void test_reset_clears_state() {
    PIDController pid(1.0f, 1.0f, 1.0f, 100.0f);
    pid.compute(80.0f, DT);  // build up some state
    pid.compute(80.0f, DT);
    pid.reset();

    // After reset, a new compute should behave like a fresh controller
    PIDController fresh(1.0f, 1.0f, 1.0f, 100.0f);
    float out_reset = pid.compute(80.0f, DT);
    float out_fresh = fresh.compute(80.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(out_fresh, out_reset);
}

// ── Setters ─────────────────────────────────────────────────

void test_set_target() {
    PIDController pid(1.0f, 0.0f, 0.0f, 100.0f);
    pid.setTarget(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, pid.target());

    float out = pid.compute(50.0f, DT);  // error = 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out);
}

void test_set_gains() {
    PIDController pid(1.0f, 0.0f, 0.0f, 100.0f);
    pid.setGains(3.0f, 0.0f, 0.0f);

    float out = pid.compute(90.0f, DT);  // error = 10, Kp = 3
    TEST_ASSERT_EQUAL_FLOAT(30.0f, out);
}

// ── dt edge cases ───────────────────────────────────────────

void test_zero_dt_skips_derivative() {
    PIDController pid(1.0f, 0.0f, 1.0f, 100.0f);
    float out = pid.compute(90.0f, 0.0f);
    // With dt=0 only P contributes (D is skipped)
    TEST_ASSERT_EQUAL_FLOAT(10.0f, out);
}

// ── Runner ──────────────────────────────────────────────────

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_initial_accessors);
    RUN_TEST(test_p_only_positive_error);
    RUN_TEST(test_p_only_negative_error);
    RUN_TEST(test_p_only_zero_error);
    RUN_TEST(test_integral_accumulates);
    RUN_TEST(test_derivative_responds_to_error_change);
    RUN_TEST(test_output_clamped_to_limits);
    RUN_TEST(test_custom_output_limits);
    RUN_TEST(test_anti_windup_prevents_integral_blowup);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_set_target);
    RUN_TEST(test_set_gains);
    RUN_TEST(test_zero_dt_skips_derivative);

    return UNITY_END();
}
