#include "visual_trajectory_speed.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float lhs, float rhs)
{
    return fabsf(lhs - rhs) < 0.0001f;
}

int main(void)
{
    float previous = 0.01f;
    float output = 0.0f;
    bool ok;

    /* Negative mapping changes direction in position, never in the speed cap. */
    ok = visual_trajectory_limit_motor_speed(
        0.20f, -1.0f, 0.0f, 0.5f, 100.0f, 10u,
        1.05f, 2.0f, 0.01f, 0.05f, 0.005f,
        &previous, &output);
    assert(ok);
    assert(near(output, 0.21f));

    /* A corrupt or infeasible request cannot exceed the motor hard cap. */
    previous = 0.5f;
    ok = visual_trajectory_limit_motor_speed(
        50.0f, 1.0f, 10.0f, 0.5f, 100.0f, 10u,
        1.05f, 2.0f, 0.01f, 0.05f, 0.005f,
        &previous, &output);
    assert(ok);
    assert(near(output, 0.5f));

    /* Speed-cap increases are rate limited. */
    previous = 0.01f;
    ok = visual_trajectory_limit_motor_speed(
        0.4f, 1.0f, 0.0f, 0.5f, 2.0f, 10u,
        1.05f, 2.0f, 0.01f, 0.05f, 0.005f,
        &previous, &output);
    assert(ok);
    assert(near(output, 0.03f));

    /* Reducing the cap for endpoint settling is immediate. */
    previous = 0.5f;
    ok = visual_trajectory_limit_motor_speed(
        0.0f, 1.0f, 0.0f, 0.5f, 2.0f, 10u,
        1.05f, 2.0f, 0.01f, 0.05f, 0.005f,
        &previous, &output);
    assert(ok);
    assert(near(output, 0.01f));

    /* A remaining position error gets a bounded low-speed correction. */
    previous = 0.5f;
    ok = visual_trajectory_limit_motor_speed(
        0.0f, 1.0f, 0.02f, 0.5f, 2.0f, 10u,
        1.05f, 2.0f, 0.01f, 0.05f, 0.005f,
        &previous, &output);
    assert(ok);
    assert(near(output, 0.05f));

    /* Non-finite input fails closed at the hold speed. */
    previous = 0.5f;
    ok = visual_trajectory_limit_motor_speed(
        NAN, 1.0f, 0.0f, 0.5f, 2.0f, 10u,
        1.05f, 2.0f, 0.01f, 0.05f, 0.005f,
        &previous, &output);
    assert(!ok);
    assert(near(output, 0.01f));

    puts("visual trajectory speed tests passed");
    return 0;
}
