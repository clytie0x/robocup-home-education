/* Every number the firmware uses, in one place.
 *
 * "MEASURE ON THE BOARD" marks a value nobody has measured yet. The placeholder lets the
 * code build and the laptop tests run; it must be replaced before this firmware drives
 * the robot (base/docs/firmware-design.md, sections 4 and 5). */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- Geometry ---- */

/* Wheel to wheel, centre of contact. The owner's number; refine with two 360 deg turns
 * against LiDAR scan matching (design 4.3). */
#define TRACK_M 0.31f

/* MEASURE ON THE BOARD. Tyres are marked 6X1.5 (6 in = 0.152 m). Refine by driving a
 * straight line and comparing with the LiDAR distance (design 4.2). */
#define WHEEL_DIAMETER_M 0.152f

/* MEASURE ON THE BOARD. Placeholder. Encoder counts per wheel turn with the gearbox, as
 * the timer counts them (both edges of both channels). Turn a wheel 10 times by hand and
 * read E# before and after (design 4.1). */
#define COUNTS_PER_WHEEL_TURN 1320.0f

#define PI_F 3.14159265f
#define METERS_PER_COUNT (PI_F * WHEEL_DIAMETER_M / COUNTS_PER_WHEEL_TURN)

/* MEASURE ON THE BOARD. +1 or -1, chosen so that positive PWM and rising counts both mean
 * "this wheel rolls forward". The two motors face opposite ways, so one side is usually -1. */
#define LEFT_MOTOR_SIGN 1
#define RIGHT_MOTOR_SIGN 1
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN 1

/* ---- Loop timing ---- */

#define CONTROL_HZ 200
#define CONTROL_DT (1.0f / CONTROL_HZ)
/* Wheel speed comes from the count change over this many ticks (20 ms). One tick alone
 * would show only a few counts, so the speed would jump in big steps. */
#define SPEED_WINDOW_TICKS 4

/* ---- Speed limits and ramps ---- */

/* MEASURE ON THE BOARD. Hard cap for each wheel. Set it a little below what 100 % PWM
 * gives on the floor (design 5), so the speed loop always has some PWM left to correct
 * with. M<vvv># can lower it, never raise it. */
#define MAX_WHEEL_SPEED_MPS 0.15f

/* Speeding up: the value in base/params.json. */
#define ACCEL_MPS2 0.25f
/* Slowing down, also for S# and the command timeout. Design draft value; at 0.12 m/s it
 * stops in 1.4 cm and 0.24 s. Owner to confirm (wheel slip on the competition floor). */
#define BRAKE_MPS2 0.5f

/* ---- Wheel speed loop: pwm% = KFF * target + KP * error + KI * integral ---- */

/* MEASURE ON THE BOARD. PWM percent per m/s, from the PWM ladder (design 5). The
 * placeholder assumes 100 % gives 0.13 m/s. With a good feed-forward the PI only
 * corrects the small rest. */
#define KFF_PCT_PER_MPS 770.0f
/* MEASURE ON THE BOARD. Tune with step logs from E#, wheels off the floor first. */
#define KP_PCT_PER_MPS 300.0f
#define KI_PCT_PER_M 1500.0f
/* KI * INTEGRAL_MAX_M = 30 % PWM: enough to make up for load and battery sag, small
 * enough that a blocked wheel cannot wind the integral up for long. */
#define INTEGRAL_MAX_M 0.02f
#define PWM_MAX_PCT 100.0f

/* ---- Safety ---- */

/* No valid F or B for this long: ramp to a stop. The Pi resends every 40 ms and its own
 * watchdog (wheels.TIMEOUT) is also 0.3 s. */
#define COMMAND_TIMEOUT_MS 300u
/* Encoder fault: PWM above this, but the wheel does not turn (or turns the wrong way)... */
#define FAULT_PWM_PCT 30.0f
/* ...for this long. Then both motors stop until S#. */
#define FAULT_MS 300u

#endif
