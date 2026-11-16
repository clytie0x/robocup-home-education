#include "odom.h"

#include <math.h>

void odom_reset(odom_pose *p)
{
    p->x = 0.0f;
    p->y = 0.0f;
    p->th = 0.0f;
}

void odom_update(odom_pose *p, int32_t d_left, int32_t d_right, float mpc_left,
                 float mpc_right, float track, uint32_t t_ms)
{
    float left = (float)d_left * mpc_left;
    float right = (float)d_right * mpc_right;
    float ds = (left + right) / 2.0f;
    float dth = (right - left) / track;
    /* Midpoint rule: during the tick the heading moved from th to th + dth, so the step
     * is taken along the heading halfway through. */
    float mid = p->th + dth / 2.0f;
    p->x += ds * cosf(mid);
    p->y += ds * sinf(mid);
    p->th += dth;
    p->t_ms = t_ms;
}
