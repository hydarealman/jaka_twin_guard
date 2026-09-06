#include "visual_trajectory_h7_adapter.h"

#include "Custom_ctrl.h"
#include "arm_visual_control.h"
#include "bsp_usart.h"
#include "hand_task.h"
#include "hand_task_interface.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define URAD_PER_RAD                    1000000.0f
#define TRAJECTORY_MAX_ACCEL_URAD_S2    10000000
#define TRAJECTORY_START_TOLERANCE_URAD 50000
#define TRAJECTORY_FOLLOWING_ERROR_URAD 300000
#define TRAJECTORY_GOAL_TOLERANCE_URAD  50000

static arm_visual_control_t trajectory_control;
static visual_trajectory_joint_target_t trajectory_joint_target;

/*
 * USART10 AA55 v1 uses the ROS/URDF joint coordinate system directly:
 * joint_1..joint_6, radians, URDF zero and URDF positive direction.
 *
 * feedback_joint_angle[] and CC_handler targets are required to use the same
 * convention.  Do not repeat the legacy custom-controller sign/zero mapping
 * in this serial adapter; doing so reverses J2/J4/J5 and offsets J2 twice.
 */

static const int32_t ros_joint_min_urad[6] = {
    -2094395,  /* J1:  -120 deg */
           0,  /* J2:    0 deg */
    -3141593,  /* J3: -180 deg */
    -2879793,  /* J4: -165 deg */
    -1570796,  /* J5:  -90 deg */
    -3141593   /* J6: -180 deg */
};

static const int32_t ros_joint_max_urad[6] = {
     2094395,  /* J1: 120 deg */
     2530727,  /* J2: 145 deg */
           0,  /* J3:   0 deg */
     2879793,  /* J4: 165 deg */
     1570796,  /* J5:  90 deg */
     3141593   /* J6: 180 deg */
};

/*
 * Joint-side limits derived from the existing field-tested DM position-mode
 * motor caps and the current joint mapping ratios. They reject an infeasible
 * trajectory before motion. The motor output layer applies the caps again.
 */
static const int32_t ros_joint_max_velocity_urad_s[6] = {
     420000,  /* J1: below 0.5 * abs(0.8700) */
     490000,  /* J2: below 0.5 * abs(1.0008) */
     490000,  /* J3: below 0.5 * abs(0.9970) */
    1290000,  /* J4: below 3.0 * abs(0.4327) */
    1500000,  /* J5: conservative URDF-side cap */
    1500000   /* J6: conservative URDF-side cap */
};

static void h7_set_target_motion_active(bool active)
{
    taskENTER_CRITICAL();
    trajectory_joint_target.motion_active = active ? 1u : 0u;
    taskEXIT_CRITICAL();
}

static void h7_invalidate_target(void)
{
    taskENTER_CRITICAL();
    trajectory_joint_target.valid = 0u;
    trajectory_joint_target.motion_active = 0u;
    taskEXIT_CRITICAL();
}

bool Visual_Trajectory_H7_Read_Target(
    visual_trajectory_joint_target_t *target)
{
    uint32_t now;
    if (target == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *target = trajectory_joint_target;
    taskEXIT_CRITICAL();

    if (target->valid == 0u) {
        return false;
    }
    now = HAL_GetTick();
    target->age_ms = now - target->updated_ms;
    return true;
}



static int32_t rad_to_urad(fp32 value)
{
    return (int32_t)(value * URAD_PER_RAD);
}

static uint32_t h7_now_ms(void *user)
{
    (void)user;
    return HAL_GetTick();
}

static void h7_uart_transmit(void *user, const uint8_t *data, size_t length)
{
    (void)user;
    (void)USART10_Send((uint8_t *)data, (unsigned short)length);
}

static bool h7_read_feedback(
    void *user,
    int32_t position_urad[6],
    int32_t velocity_urad_s[6])
{
    static int32_t previous_position[6];
    static int32_t previous_velocity[6];
    static uint32_t previous_ms;
    static bool initialized;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - previous_ms;
    uint8_t i;
    (void)user;

    for (i = 0u; i < 6u; ++i) {
        position_urad[i] = rad_to_urad(hand_task_handler_ptr->feedback_joint_angle[i]);
    }
    if (initialized && (elapsed != 0u)) {
        for (i = 0u; i < 6u; ++i) {
            previous_velocity[i] = (int32_t)(
                ((int64_t)position_urad[i] - previous_position[i]) * 1000 / elapsed);
        }
    } else if (!initialized) {
        memset(previous_velocity, 0, sizeof(previous_velocity));
        initialized = true;
    }
    memcpy(velocity_urad_s, previous_velocity, sizeof(previous_velocity));
    if (elapsed != 0u) {
        memcpy(previous_position, position_urad, sizeof(previous_position));
        previous_ms = now;
    }
    return true;
}

static void h7_set_target(
    void *user,
    const int32_t position_urad[6],
    const int32_t velocity_urad_s[6])
{
    uint8_t i;
    (void)user;
    taskENTER_CRITICAL();
    for (i = 0u; i < VISUAL_TRAJECTORY_JOINT_COUNT; ++i) {
        trajectory_joint_target.position_rad[i] =
            (fp32)position_urad[i] / URAD_PER_RAD;
        trajectory_joint_target.velocity_rad_s[i] =
            (fp32)velocity_urad_s[i] / URAD_PER_RAD;
    }
    trajectory_joint_target.updated_ms = HAL_GetTick();
    trajectory_joint_target.sequence++;
    trajectory_joint_target.valid = 1u;

    /* Keep the legacy position mirror for the existing custom-control path. */
    CC_handler.joint_angle[0] = trajectory_joint_target.position_rad[0];
    CC_handler.joint_angle[1] = trajectory_joint_target.position_rad[1];
    CC_handler.joint_angle[2] = trajectory_joint_target.position_rad[2];
    CC_handler.joint_angle[3] = trajectory_joint_target.position_rad[3];
    CC_handler.joint_angle[4] = trajectory_joint_target.position_rad[4];
    CC_handler.G_angle = trajectory_joint_target.position_rad[5];
    CC_handler.get_cc_data_flag = 1;
    taskEXIT_CRITICAL();
}

static void h7_request_stop(void *user)
{
    int32_t position[6];
    int32_t velocity[6];
    (void)user;
    if (h7_read_feedback(NULL, position, velocity)) {
        memset(velocity, 0, sizeof(velocity));
        h7_set_target(NULL, position, velocity);
    }
}

static bool h7_stop_complete(void *user)
{
    int32_t position[6];
    int32_t velocity[6];
    uint8_t i;
    (void)user;
    if (!h7_read_feedback(NULL, position, velocity)) return false;
    for (i = 0u; i < 6u; ++i) {
        if ((velocity[i] > 50000) || (velocity[i] < -50000)) return false;
    }
    return true;
}

static bool h7_estop_active(void *user)
{
    (void)user;
    return false;
}

static bool h7_driver_fault_active(void *user)
{
    uint8_t i;
    (void)user;
    for (i = 0u; i < HAND_MOTOR_COUNT; ++i) {
        if ((hand_task_handler_ptr->motor_offline_flag[i] != 0u) ||
            (hand_task_handler_ptr->motor_stall_flag[i] != 0u)) return true;
    }
    return false;
}

static bool h7_claw_action(void *user, arm_claw_action_t action)
{
    (void)user;
    return hand_claw_request((hand_claw_request_t)action);
}

static bool h7_read_claw_state(
    void *user,
    arm_claw_state_t *state,
    uint8_t *flags)
{
    hand_claw_reported_state_t reported;
    (void)user;
    if ((state == NULL) || (flags == NULL)) {
        return false;
    }

    reported = hand_claw_get_reported_state();
    switch (reported) {
    case HAND_CLAW_REPORTED_OPENING:
        *state = ARM_CLAW_STATE_OPENING;
        break;
    case HAND_CLAW_REPORTED_OPEN:
        *state = ARM_CLAW_STATE_OPEN;
        break;
    case HAND_CLAW_REPORTED_CLOSING:
        *state = ARM_CLAW_STATE_CLOSING;
        break;
    case HAND_CLAW_REPORTED_CLOSED:
        *state = ARM_CLAW_STATE_CLOSED;
        break;
    case HAND_CLAW_REPORTED_FAULT:
        *state = ARM_CLAW_STATE_FAULT;
        break;
    case HAND_CLAW_REPORTED_UNKNOWN:
    default:
        *state = ARM_CLAW_STATE_UNKNOWN;
        break;
    }
    /* There is no position/current/contact sensor that verifies the endpoint. */
    *flags = 0u;
    return true;
}

static void set_ros_limits(
    arm_joint_safety_config_t *config,
    uint8_t index)
{
    config->direction = 1;
    config->zero_offset_urad = 0;
    config->min_position_urad = ros_joint_min_urad[index];
    config->max_position_urad = ros_joint_max_urad[index];
    config->max_velocity_urad_s = ros_joint_max_velocity_urad_s[index];
    config->max_acceleration_urad_s2 = TRAJECTORY_MAX_ACCEL_URAD_S2;
    config->start_tolerance_urad = TRAJECTORY_START_TOLERANCE_URAD;
    config->following_error_urad = TRAJECTORY_FOLLOWING_ERROR_URAD;
    config->goal_tolerance_urad = TRAJECTORY_GOAL_TOLERANCE_URAD;
}

static bool h7_trajectory_init(void)
{
    arm_visual_config_t config;
    arm_visual_hooks_t hooks;
    uint8_t i;
    memset(&config, 0, sizeof(config));
    memset(&hooks, 0, sizeof(hooks));
    memset(&trajectory_joint_target, 0, sizeof(trajectory_joint_target));

    for (i = 0u; i < 6u; ++i) {
        set_ros_limits(&config.joint[i], i);
    }
    config.max_trajectory_points = ARM_VISUAL_MAX_TRAJECTORY_POINTS;
    config.communication_timeout_ms = 1000u;
    config.execution_timeout_margin_ms = 5000u;
    config.following_error_duration_ms = 300u;
    config.completion_stable_ms = 200u;
    config.stopped_velocity_urad_s = 50000u;
    config.state_period_ms = 50u;
    config.result_retry_ms = 100u;
    config.claw_state_period_ms = 100u;
    config.claw_communication_timeout_ms = 500u;
    config.claw_action_timeout_ms = 2000u;
    config.result_max_retries = 5u;

    hooks.get_monotonic_ms = h7_now_ms;
    hooks.uart_transmit = h7_uart_transmit;
    hooks.read_joint_feedback = h7_read_feedback;
    hooks.set_joint_target = h7_set_target;
    hooks.request_controlled_stop = h7_request_stop;
    hooks.controlled_stop_complete = h7_stop_complete;
    hooks.estop_active = h7_estop_active;
    hooks.driver_fault_active = h7_driver_fault_active;
    hooks.request_claw_action = h7_claw_action;
    hooks.read_claw_state = h7_read_claw_state;
    return arm_visual_control_init(&trajectory_control, &config, &hooks, NULL);
}

void Visual_Trajectory_H7_Task(void *argument)
{
    uint8_t rx_data[128];
    (void)argument;
    osDelay(1500u);
    if (!h7_trajectory_init()) {
        for (;;) osDelay(1000u);
    }

    for (;;) {
        bool permitted =
            (hand_task_handler_ptr->ctrl_mode == HAND_MODE_CUSTOM_CTRL) &&
            !h7_driver_fault_active(NULL);
        bool motion_active;
        unsigned int available = USART10_GetDataCount();
        arm_visual_control_set_ready(&trajectory_control, permitted);
        if (available > sizeof(rx_data)) available = sizeof(rx_data);
        if (available != 0u) {
            unsigned int received = USART10_Recv(rx_data, (unsigned short)available);
            arm_visual_control_feed(&trajectory_control, rx_data, received);
        }
        arm_visual_control_tick(&trajectory_control);
        motion_active =
            (trajectory_control.trajectory_state == ARM_TRAJECTORY_EXECUTING) ||
            (trajectory_control.trajectory_state == ARM_TRAJECTORY_STOPPING);
        h7_set_target_motion_active(motion_active);
        if (!permitted) {
            h7_invalidate_target();
        }
        arm_visual_control_service(&trajectory_control);
        osDelay(1u);
    }
}
