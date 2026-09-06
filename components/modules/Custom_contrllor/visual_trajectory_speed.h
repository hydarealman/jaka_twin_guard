#ifndef VISUAL_TRAJECTORY_SPEED_H
#define VISUAL_TRAJECTORY_SPEED_H

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Pure, host-testable speed-cap calculation for the DM position-speed mode.
 * Direction comes from the target position, so the returned cap is positive.
 */
static inline bool visual_trajectory_float_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static inline bool visual_trajectory_limit_motor_speed(
    float joint_velocity_rad_s,
    float joint_to_motor_ratio,
    float motor_position_error_rad,
    float hard_limit_rad_s,
    float rise_rate_rad_s2,
    uint32_t elapsed_ms,
    float tracking_margin,
    float error_gain,
    float min_hold_rad_s,
    float min_tracking_rad_s,
    float position_deadband_rad,
    float *previous_speed_rad_s,
    float *output_speed_rad_s)
{
    float requested_speed;
    float speed_limit;
    float previous_speed;
    float max_increase;

    if ((previous_speed_rad_s == NULL) || (output_speed_rad_s == NULL) ||
        !visual_trajectory_float_is_finite(joint_velocity_rad_s) ||
        !visual_trajectory_float_is_finite(joint_to_motor_ratio) ||
        !visual_trajectory_float_is_finite(motor_position_error_rad) ||
        !visual_trajectory_float_is_finite(hard_limit_rad_s) ||
        !visual_trajectory_float_is_finite(rise_rate_rad_s2) ||
        (fabsf(joint_to_motor_ratio) < 0.0001f) ||
        (hard_limit_rad_s <= 0.0f) || (rise_rate_rad_s2 <= 0.0f) ||
        (elapsed_ms == 0u) || (tracking_margin < 1.0f) ||
        (error_gain < 0.0f) || (min_hold_rad_s <= 0.0f) ||
        (min_tracking_rad_s < min_hold_rad_s) ||
        (position_deadband_rad < 0.0f)) {
        if (output_speed_rad_s != NULL) {
            *output_speed_rad_s = min_hold_rad_s > 0.0f
                ? min_hold_rad_s : 0.001f;
        }
        if (previous_speed_rad_s != NULL) {
            *previous_speed_rad_s = min_hold_rad_s > 0.0f
                ? min_hold_rad_s : 0.001f;
        }
        return false;
    }

    requested_speed = fabsf(joint_velocity_rad_s / joint_to_motor_ratio);
    motor_position_error_rad = fabsf(motor_position_error_rad);
    speed_limit = requested_speed * tracking_margin +
        motor_position_error_rad * error_gain;
    if ((motor_position_error_rad > position_deadband_rad) &&
        (speed_limit < min_tracking_rad_s)) {
        speed_limit = min_tracking_rad_s;
    }
    if (speed_limit < min_hold_rad_s) {
        speed_limit = min_hold_rad_s;
    }
    if (speed_limit > hard_limit_rad_s) {
        speed_limit = hard_limit_rad_s;
    }

    previous_speed = *previous_speed_rad_s;
    if ((!visual_trajectory_float_is_finite(previous_speed)) ||
        (previous_speed < min_hold_rad_s)) {
        previous_speed = min_hold_rad_s;
    }
    max_increase = rise_rate_rad_s2 * (float)elapsed_ms / 1000.0f;
    if (speed_limit > previous_speed + max_increase) {
        speed_limit = previous_speed + max_increase;
    }
    if (speed_limit > hard_limit_rad_s) {
        speed_limit = hard_limit_rad_s;
    }

    *previous_speed_rad_s = speed_limit;
    *output_speed_rad_s = speed_limit;
    return true;
}

#endif
