#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "PID.h"
#include "telemetry.h"

param_t param;
int Servo_Ctl;

void telem_tap_angle_target(float target) { (void)target; }
void telem_tap_pwm_accel(float accel) { (void)accel; }
void telem_tap_inner(float p, float i, float d, float target, float output)
{
    (void)p; (void)i; (void)d; (void)target; (void)output;
}

static void expect_close(float actual, float expected)
{
    if (fabsf(actual - expected) > 0.0000003f) {
        fprintf(stderr, "servo=%d sgain=%.7f: %.9f != %.9f\n",
                Servo_Ctl, (double)Servo_Gain_K, (double)actual, (double)expected);
        assert(0);
    }
}

static float compensation(int steer)
{
    Servo_Ctl = steer;
    return Servo_Gain();
}

/* Frozen pre-change formula: regression oracle for preserved endpoints and
   every negative command, independent of the new interpolation. */
static float original_compensation(int steer)
{
    static const float coefficients[] = {
        -0.0010f, -0.0052f, -0.0058f, -0.0061f, -0.0068f,
        -0.0069f, -0.0066f, -0.0061f, -0.0062f, -0.0063f,
        -0.0010f, -0.0024f, -0.0037f, -0.0049f, -0.0051f,
        -0.0053f, -0.0055f, -0.0057f, -0.00585f, -0.0060f
    };
    int magnitude = steer < 0 ? -steer : steer;
    int index = magnitude ? (magnitude - 1) / 10 : 0;
    if (index > 9) index = 9;
    if (steer < 0) index += 10;
    return steer * Servo_Gain_K + coefficients[index] * steer;
}

static void test_preserved_values_across_menu_gains(void)
{
    static const int anchors[] = {0, 12, 24, 36, 48, 60, 70, 80, 90, 100, 120};
    for (int setting = 0; setting <= 100; ++setting) {
        Servo_Gain_K = setting * 0.0001f;
        for (unsigned i = 0; i < sizeof(anchors) / sizeof(anchors[0]); ++i)
            expect_close(compensation(anchors[i]), original_compensation(anchors[i]));
        for (int steer = -Servo_Delta; steer <= 0; ++steer)
            assert(compensation(steer) == original_compensation(steer));
        for (int steer = 101; steer <= Servo_Delta; ++steer)
            assert(compensation(steer) == original_compensation(steer));
    }
}

static void test_candidate_values_and_observed_transitions(void)
{
    static const struct { int steer; float expected; } points[] = {
        {0, 0.0f}, {4, -0.0008f}, {8, -0.0016f}, {12, -0.0024f},
        {24, -0.0192f}, {36, -0.0396f}, {40, -0.0552f},
        {44, -0.0708f}, {48, -0.0864f}, {60, -0.1140f}
    };
    Servo_Gain_K = 0.0050f;
    for (unsigned i = 0; i < sizeof(points) / sizeof(points[0]); ++i)
        expect_close(compensation(points[i].steer), points[i].expected);

    float at8 = compensation(8), at12 = compensation(12);
    float at40 = compensation(40), at44 = compensation(44);
    expect_close(at12 - at8, -0.0008f);
    expect_close(at44 - at40, -0.0156f);
    expect_close(compensation(8) - compensation(12), 0.0008f);
    expect_close(compensation(40) - compensation(44), 0.0156f);
}

static void test_positive_sweep_has_no_bin_jumps_or_hysteresis(void)
{
    float forward[Servo_Delta + 1];
    Servo_Gain_K = 0.0050f;
    for (int steer = 0; steer <= Servo_Delta; ++steer) {
        forward[steer] = compensation(steer);
        assert(isfinite(forward[steer]));
        if (steer > 0) {
            /* Largest candidate segment slope is 0.0039 deg/unit.
               Check every integer, including both sides of old bin edges. */
            assert(fabsf(forward[steer] - forward[steer - 1]) <= 0.0039003f);
            if (steer <= 60) assert(forward[steer] < forward[steer - 1]);
        }
    }
    for (int steer = Servo_Delta; steer >= 0; --steer)
        assert(compensation(steer) == forward[steer]);
}

static void test_sgain_remains_an_additive_linear_term(void)
{
    for (int steer = -Servo_Delta; steer <= Servo_Delta; ++steer) {
        Servo_Gain_K = 0.0015f;
        float baseline = compensation(steer);
        Servo_Gain_K = 0.0050f;
        expect_close(compensation(steer) - baseline, steer * (0.0050f - 0.0015f));
    }
}

int main(void)
{
    assert(Servo_Gain_K == 0.0015f); /* No change to the boot default. */
    test_preserved_values_across_menu_gains();
    test_candidate_values_and_observed_transitions();
    test_positive_sweep_has_no_bin_jumps_or_hysteresis();
    test_sgain_remains_an_additive_linear_term();
    puts("PID steering compensation tests passed");
    return 0;
}
