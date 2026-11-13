/* Every number the firmware uses, in one place.
 *
 * "MEASURE ON THE BOARD" marks a value nobody has measured yet. The placeholder lets the
 * code build and the laptop tests run; it must be replaced before this firmware drives
 * the robot (base/docs/firmware-design.md, sections 4 and 5). */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- Geometry ---- */

/* Derived, not measured directly. The old firmware's odometry used a 0.0762 m wheel
 * radius and 0.0872222 / 596 rad per encoder count (constants read from its binary), and
 * LiDAR scan matching on 2026-11-13 found true distance = old distance / 0.902. So one
 * count is 1.236e-5 m, about 38 600 counts per wheel turn. Check with a straight line
 * against the LiDAR (design 4.2); params.json's older 0.882 would give 2 % less. */
#define OLD_METERS_PER_COUNT (0.0762f * 0.0872222f / 596.0f)
#define METERS_PER_COUNT (OLD_METERS_PER_COUNT / 0.902f)

/* Wheel to wheel, centre of contact. Derived: the old firmware used 0.31 m, and its yaw
 * read 0.862 of the true turn while its distance read 0.902, so the true track is
 * 0.31 * 0.862 / 0.902 = 0.296 m (the original URDF says 0.259 m). Check with two 360 deg
 * turns against LiDAR scan matching (design 4.3). */
#define TRACK_M 0.296f

/* +1 or -1, so that positive PWM and rising counts both mean "this wheel rolls forward".
 * board.c already drives each motor forward for positive PWM, as the old firmware did.
 * The old firmware negated the left encoder (TIM2) and not the right one (TIM4). */
#define LEFT_MOTOR_SIGN 1
#define RIGHT_MOTOR_SIGN 1
#define LEFT_ENCODER_SIGN (-1)
#define RIGHT_ENCODER_SIGN 1

/* ---- Loop timing ---- */

#define CONTROL_HZ 200
#define CONTROL_DT (1.0f / CONTROL_HZ)
/* Wheel speed comes from the count change over this many ticks (20 ms). One tick alone
 * would show only a few counts, so the speed would jump in big steps. */
#define SPEED_WINDOW_TICKS 4

/* ---- Speed limits and ramps ---- */

/* Hard cap for each wheel; M<vvv># can lower it, never raise it. The old firmware gave
 * 0.12 to 0.13 m/s straight and 0.10 to 0.11 per wheel spinning at what looks like full
 * PWM, so 0.12 keeps the loop out of saturation in straight lines. MEASURE ON THE BOARD
 * with the PWM ladder (design 5) and raise it if the new PWM frequency gives more. */
#define MAX_WHEEL_SPEED_MPS 0.12f

/* Speeding up: the value in base/params.json. */
#define ACCEL_MPS2 0.25f
/* Slowing down, also for S# and the command timeout. Design draft value; at 0.12 m/s it
 * stops in 1.4 cm and 0.24 s. Owner to confirm (wheel slip on the competition floor). */
#define BRAKE_MPS2 0.5f

/* ---- Wheel speed loop: pwm% = KFF * target + KP * error + KI * integral ---- */

/* MEASURE ON THE BOARD. PWM percent per m/s, from the PWM ladder (design 5). The
 * placeholder assumes 100 % gives 0.13 m/s, as the old firmware's cap suggests. With a
 * good feed-forward the PI only corrects the small rest. */
#define KFF_PCT_PER_MPS 770.0f
/* MEASURE ON THE BOARD. Tune with step logs from E#, wheels off the floor first. */
#define KP_PCT_PER_MPS 300.0f
#define KI_PCT_PER_M 1500.0f
/* KI * INTEGRAL_MAX_M = 30 % PWM: enough to make up for load and battery sag, small
 * enough that a blocked wheel cannot wind the integral up for long. */
#define INTEGRAL_MAX_M 0.02f
#define PWM_MAX_PCT 100.0f
/* L298N PWM frequency. The old firmware used 100 Hz. 1 kHz gives a smoother current at
 * low speed and is still slow for the L298N's 2 us switching. Owner's choice. */
#define PWM_HZ 1000u

/* ---- Safety ---- */

/* No valid F or B for this long: ramp to a stop. The Pi resends every 40 ms and its own
 * watchdog (wheels.TIMEOUT) is also 0.3 s. */
#define COMMAND_TIMEOUT_MS 300u
/* Encoder fault: PWM above this, but the wheel does not turn (or turns the wrong way)... */
#define FAULT_PWM_PCT 30.0f
/* ...where "does not turn" is slower than this: a still wheel can jitter by a count. */
#define FAULT_MIN_SPEED_MPS 0.005f
/* ...for this long. Then both motors stop until S#. */
#define FAULT_MS 300u

#endif
