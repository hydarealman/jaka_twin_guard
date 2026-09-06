#ifndef VISUAL_TRAJECTORY_H7_ADAPTER_H
#define VISUAL_TRAJECTORY_H7_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#define VISUAL_TRAJECTORY_JOINT_COUNT 6u
#define VISUAL_TRAJECTORY_ACTIVE_TARGET_MAX_AGE_MS 20u

typedef struct {
    float position_rad[VISUAL_TRAJECTORY_JOINT_COUNT];
    float velocity_rad_s[VISUAL_TRAJECTORY_JOINT_COUNT];
    uint32_t sequence;
    uint32_t updated_ms;
    uint32_t age_ms;
    uint8_t valid;
    uint8_t motion_active;
} visual_trajectory_joint_target_t;

/*
 * Thread-safe snapshot for HandTask. Position and velocity always come from
 * the same interpolation cycle. A valid inactive snapshot remains available
 * after success so the final pose can be held with a low settle speed.
 */
bool Visual_Trajectory_H7_Read_Target(
    visual_trajectory_joint_target_t *target);

void Visual_Trajectory_H7_Task(void *argument);

#endif
