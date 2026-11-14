> Reference only: this is the spec from the earlier Rust project (clytie0x/robocup-home PR #37, commit 696ec641). Module and command names here refer to that Rust code; port the rules, scoresheet and test ideas to Python, not the Rust code.


# 5.1 Help Me Carry

This specification is the main-league RoboCup@Home Rulebook, Final version for RoboCup 2025, Revision 2025-04-28. The active rulebook is 2025.1. The file that was read has SHA-256 `d23a76efaf436e6286076b34628a7451b3e8fd41e38b79e54b4db739acf9fe11`. The section is 5.1 Help Me Carry on printed pages 39, 40, and 41. It is not an Education-league rule. The professor PDF section 3.1 document stays as it is. A pass in that older document is not a score on this sheet.

The PDF text was inflated and the text operators were decoded. Quotations below are those words. The attached text extract of PDF pages 45 to 47 was used only as a second reading. Where the symbol font draws a minus, the decoded byte is not a hyphen; the sheet magnitudes are the numbers in the same line.

A `sim` row can check a model. It does not certify a robot or a public place. Slice 1 does not run the crowd, the small object, the hard-to-see object, the barriers, the queue, or Deus ex Machina. Those rows stay `not-run`. Not attending and outstanding performance are `out-of-sim-scope`.

## Setup

Printed page 39. The test is outside the arena in a public space. The robot starts at a predefined location. The operator stands in front of the robot. At least two bags are placed near the car. The setup also names a small object on the ground, a hard-to-see object, retractable barriers, and other people walking by. Slice 1 places the start at `x = -2 m`, outside the arena half-plane `x >= 0`, and places two bags near the scripted car. It does not place the four obstacles.

## Procedure

Printed page 39.

1. Following the operator. The robot informs the operator when it is ready to follow. The operator walks naturally towards the car. After reaching the car, the operator points at the bag.
2. Obstacles. The robot faces four obstacles, in an arbitrary order: a small object on the ground, a hard-to-see object, a crowd outside, and a small area blocked with retractable barriers.
3. Picking up the bag. The robot picks up the bag pointed at by the operator.
4. Bag delivery. The robot takes the bag to the starting location. It may place the bag on the floor.
5. Optional goals. Re-reaching the starting location. Following the queue on the way back: after the car, some of the crowd return to the starting location in a queue, and the robot may join that queue.

The slice logs `ready-to-follow` before it moves. The operator follows a fixed polyline to the car. The pointing input is a scripted ray from the operator pose through one bag, then `perception::select`. That is not a camera reading of a gesture. The pick is a state flag named `abstract-pick`. It is not a gripper closure and not a physical grasp. The return walks the robot's accepted poses backwards. It does not read the car point.

## Additional rules

Printed page 40.

1. There is no real car. A fixed location outside the arena is the car.
2. The robot can reach the car only by following the operator. The location is unknown before the test.
3. Deus ex Machina. Score reductions apply when the robot loses the operator and is helped to find them again, by natural interaction, unnatural interaction, coming back, or touching the robot. Penalties apply only if the robot continues to score after the interaction. They can be applied more than once, up to 250 points.

`follow_intent` takes a `SensorView` and does not take a car. `dem_penalty` encodes the 250 cap and the later-score condition. The scenario does not call it, so the four penalty rows stay `not-run`.

## Referee and Organizing Committee

Printed page 40. Referees select the operator, three to four people for the crowd and the later queue, the bag positions, the assigned bag, the obstacle order, the car location, and the queue location. They instruct the operator to follow the robot's instructions, step over the small obstacle, and pass closely around the hard-to-see object. At least two hours before the test, the Organizing Committee announces the start, selects the bags, and organizes the barriers. Slice 1 uses a fixed script for the start, the car, and two bags. It does not claim those organizing steps happened.

## Time

Printed page 40. "The maximum time for this test is 5:00 minutes." The sim budget is 300000 ms. A step that would pass that budget is not taken. Rows already measured stay. Later rows stay at a measured zero.

## Scoresheet

Printed page 41. The sheet prints the token "beg". The procedure says bag. The queue line prints "on the way to the arena". The procedure says "on its way back" to the starting location. Both wordings are quoted as printed. The nine positive lines are 300, 15, 100, four times 50, 200, and 300. Their sum is 1115. Penalties and the outstanding line are outside that total.

| action | catalog | condition the model can check | component | status |
| --- | --- | --- | --- | --- |
| Following the person to the car | 300 | `plan_follow` returns `Arrived` while the operator script is at the car. The car is not an argument of `follow_intent`. | `navigation::plan_follow`, `physical::integrate` | implemented-in-slice1 |
| Perceiving the correct beg (visualize on screen or say which one) | 15 | The log says `perceived bag id` and that id is the assigned bag, and the scripted ray points at that same bag. | `perception::select` | implemented-in-slice1 |
| Picking up the correct bag | 100 | The abstract pick id is the assigned bag, and the ray points at that bag. | task flag in `physical::help_carry` | implemented-in-slice1 |
| Avoiding the crowd of people obstructing the path | 50 | A crowd body was on the path and the base did not overlap it. | not placed in slice 1 | not-run |
| Avoiding the small object on the ground | 50 | The small object was on the path and the base did not overlap it. | not placed in slice 1 | not-run |
| Avoiding the hard-to-see object | 50 | That object was on the path and the base did not overlap it. | not placed in slice 1 | not-run |
| Avoiding the area blocked with retractable barriers | 50 | The barrier area was on the path and the base did not enter it. | not placed in slice 1 | not-run |
| Re-reaching the starting location | 200 | The return ends within 0.05 m of the first accepted pose. | odometry crumbs and `physical::integrate` | implemented-in-slice1 |
| Joining and staying in the queue on the way to the arena | 300 | The robot joins the same people who formed the crowd, on the way back. The sheet's "arena" wording is kept. | not run | not-run |
| Dropping the bag | -50 | The script sets the drop flag after the abstract pick. | task flag | implemented-in-slice1 |
| Rediscovering the operator by natural interaction | -50 | A natural help event, counted only if scoring continues, and only inside the 250 cap. | `task_5_1::dem_penalty` is not called by the scenario | not-run |
| Rediscovering the operator by unnatural interaction | -100 | Same rule, catalog -100. | not called | not-run |
| Rediscovering the operator by asking them to come back | -150 | Same rule, catalog -150. | not called | not-run |
| Rediscovering the operator by direct contact | -200 | Same rule, catalog -200. | not called | not-run |
| Not attending (see sec. 3.9.1) | -500 | A competition-attendance decision. | none | out-of-sim-scope |
| Using alternative start signal (see sec. 3.7.8) | -100 | An authorized signal other than the default. Slice 1 only logs ready-to-follow. | not run | not-run |
| Outstanding performance (see sec. 3.9.3) | 111 | A committee award. | none | out-of-sim-scope |
| Total Score (excluding special penalties & standard bonuses) | 1115 | The nine positive catalog lines sum to 1115. A partial run does not receive 1115. | `task_5_1::score` | implemented-in-slice1 |

## Acceptance tests

These names are the tests in the tree. CI run [43209180479](https://github.com/clytie0x/robocup-home/actions/runs/43209180479), job [131152008954](https://github.com/clytie0x/robocup-home/actions/runs/43209180479/job/131152008954), on commit `74fb7f3dacedf9c7e2031c0681f9f6bb7c509536`, ran `cargo test --workspace --all-targets --locked` and the release repeat, plus the doc tests. Debug all-targets: 386 passed, 0 failed. Doctests: 8 passed, 0 failed. Release all-targets: 386 passed, 0 failed. Release doctests: 8 passed, 0 failed. The fixed scenario wrote the replay in that same job. The replay is [timeline.csv](../evidence/help-me-carry-slice1/timeline.csv), [trajectory.png](../evidence/help-me-carry-slice1/trajectory.png), and [score.txt](../evidence/help-me-carry-slice1/score.txt).

| test | what a pass must show | result | gate |
| --- | --- | --- | --- |
| `printed_positive_lines_sum_to_1115` | 300+15+100+4*50+200+300 = 1115 | pass | sim |
| `a_correct_bag_scores_perceive_and_pick` | 15 and 100, measured subtotal 615 when follow and return are also set | pass | sim |
| `the_wrong_pointed_bag_scores_neither_perceive_nor_pick` | both awards 0 | pass | sim |
| `a_drop_is_minus_50_and_a_missed_return_is_zero` | drop -50, return 0 | pass | sim |
| `timeout_keeps_earned_rows` | follow stays 300 when the return is false | pass | sim |
| `obstacles_queue_dem_and_alt_start_stay_not_run` | those rows are `not-run` and award 0; absent and outstanding are `out-of-sim-scope` | pass | sim |
| `dem_penalties_cap_at_250_and_only_after_later_points` | two -200 helps become -250 only when later points exist; the scenario row stays `not-run` | pass | sim |
| `the_success_run_follows_picks_and_returns` | ready log, perceived bag id 2, abstract pick, place on the floor, 615, elapsed at most 300000 ms, PNG decodes | pass | sim |
| `a_second_car_is_not_a_hardcoded_robot_goal` | a different car still scores the 300 | pass | sim |
| `the_follow_call_has_no_car_argument` | `follow_intent` and `return_step` take no car | pass | sim |
| `the_wrong_pointed_bag_scores_zero_for_perceive_and_pick` | the ray names bag 3, so perceive and pick are 0 | pass | sim |
| `a_timeout_stops_and_keeps_only_earned_rows` | time-up after the pick, return 0, follow 300, perceive 15, pick 100 | pass | sim |
| `a_dropped_bag_scores_minus_50` | drop -50, no place-on-floor | pass | sim |
| `a_return_that_does_not_finish_scores_no_200` | return award 0 without a timeout | pass | sim |

The picture command in that job was `cargo run -p physical --bin help-me-carry --release --locked`. It is deterministic: the script is fixed and does not draw a seed. Score file SHA-256 `cfd523fe69caf8dd22c033a14e359f7a5e1520d8749fffe8a4439dfb8da07c46`. Timeline SHA-256 `9f8c318ccf87afc994b8596052304425327231c763a02326b0f5aa021be0bf18`. PNG SHA-256 `fff57bd6acb80f75bcad0c648722becf944dfaa01f55bfa99a00176cf6ca811a`.

## Limits

The base step is kinematic contact of the sourced plates against an empty obstacle list. `sent` stays false. The ray is scripted. The pick is not a grasp. The four obstacle scores, the queue, Deus ex Machina, and the alternative start are not exercised. The 0.05 m crumb tolerance, the 0.2 m step, and the arena half-plane are scenario choices, not printed measurements. A sim result is not a hardware result.


