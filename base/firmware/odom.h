/* Wheel odometry: the base's pose from encoder count changes.
 * No hardware calls; the laptop tests cover all of it. */
#ifndef ODOM_H
#define ODOM_H

#include <stdint.h>

typedef struct {
    float x;      /* m, forward from where odometry was last reset */
    float y;      /* m, to the left */
    float th;     /* rad, left positive, not wrapped (two turns read 12.57) */
    uint32_t t_ms; /* board time of the counts behind this pose */
} odom_pose;

void odom_reset(odom_pose *p);

/* Add one tick of wheel motion: count changes of each wheel (forward positive), and
 * metres per count of each wheel (they may differ after calibration). */
void odom_update(odom_pose *p, int32_t d_left, int32_t d_right, float mpc_left,
                 float mpc_right, float track, uint32_t t_ms);

#endif
