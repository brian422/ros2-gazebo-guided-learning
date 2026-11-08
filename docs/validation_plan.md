# Hazard inspection rover validation plan

## Purpose

This document preserves historical reported results dated 2026-08-08. It is
not a new validation of the current source. Text evidence may be a curated
summary rather than raw output; the original tested Git revision is unknown.
The November 2026 documentation revision does not retroactively verify those runs.

This plan converts project requirements into repeatable checks. Complete the
last two columns only after running each test in the specified environment.
Record failures honestly; a failed row is a diagnosis task, not a reason to
rewrite the expected result.

## Test environment record

Fill this before testing:

| Item | Required baseline | Actual environment |
|---|---|---|
| Ubuntu | 22.04.5 LTS Jammy | Ubuntu 22.04.5 LTS Jammy, reported by the student |
| Kernel | WSL2 `6.6.87.2-microsoft-standard-WSL2` | `6.6.87.2-microsoft-standard-WSL2`, reported by the student |
| Python | 3.10.12 through `python3` | Python 3.10.12 |
| ROS 2 | Humble | Humble |
| Gazebo | Harmonic / Gazebo Sim 8 | Gazebo Sim 8.14.0 |
| Project commit | Git commit hash | Not recorded for the historical runs |
| Test date | Local date and time | 2026-08-08, Asia/Taipei |

Useful environment commands:

```bash
grep -E '^(PRETTY_NAME|VERSION_ID|UBUNTU_CODENAME)=' /etc/os-release
uname -r
python3 --version
printenv ROS_DISTRO
gz sim --versions
git rev-parse --short HEAD
```

## Validation matrix

| ID | Requirement | Test method | Expected result | Actual result | Pass/fail |
|---|---|---|---|---|---|
| V-01 | All five packages are discoverable. | Run `colcon list`. | Description, sim, control, interfaces, and bringup packages are listed. | `colcon list` reported all five expected packages with their correct ament build types. | Pass |
| V-02 | Workspace builds on the required environment. | Run the Module 6 `colcon build --symlink-install --packages-up-to hazard_inspection_bringup` command. | Five packages finish; no package fails. | The final build finished description, interfaces, simulation, control, and bringup packages in 4.90 s with no failures. All five remained discoverable, and the bringup prefix resolved to the workspace install tree. | Pass |
| V-03 | Complete system starts through one command. | Run `ros2 launch hazard_inspection_bringup inspection_demo.launch.py`. | Gazebo, state publisher, spawn/bridges, four project nodes, and RViz start without a second launch command. | While the main launch was running, the graph contained the hazard sensor, controller, manager, logger, state publisher, bridge, and RViz nodes; Gazebo/RViz were visible in the supplied runtime captures. | Pass |
| V-04 | GUI selection is configurable. | Launch once with `gui:=false rviz:=false`. | Headless ROS/Gazebo system starts without Gazebo client or RViz. | Process inspection showed `gz sim -s -r` and no RViz process. The six required ROS nodes were active; `/clock` advanced and `/odom` published finite data using `odom` and `base_footprint` frames. | Pass |
| V-05 | Rover spawns in Gazebo. | Observe Gazebo after launch and inspect `/odom`. | Rover appears on the marked start area and finite odometry is published. | The privacy-safe screenshot shows the rover on the marked starting area. The archived odometry sample uses `odom` and `base_footprint` frames with finite pose and twist values. | Pass |
| V-06 | Robot receives bounded velocity commands. | During a mission run `ros2 topic echo /cmd_vel`; compare extrema with YAML limits. | `abs(linear.x) <= 0.35 m/s` and `abs(angular.z) <= 1.20 rad/s`. | Across 485 CSV rows, the maximum absolute commands were exactly 0.350000 m/s linear and 1.200000 rad/s angular; neither configured limit was exceeded. | Pass |
| V-07 | Odometry is published. | Run `ros2 topic hz /odom` and echo one message. | Repeated finite pose/twist messages use `odom` and rover child frame. | The sample contained finite pose/twist data with `frame_id: odom` and `child_frame_id: base_footprint`; the measured rate was 14.661 Hz over 90 messages. | Pass |
| V-08 | Laser scan is published. | Run `ros2 topic hz /scan` and view it in RViz. | Repeated `LaserScan` data is visible from `laser_frame`. | RViz showed the `/scan` display with status OK and visible red range points. The measured scan rate was 4.950 Hz over 27 messages. | Pass |
| V-09 | Joint state and robot-state information are published. | Inspect `/joint_states`, `/tf`, and `/tf_static`. | Wheel joint states update and robot links form a connected TF tree. | The joint-state sample contained both wheel joints with finite position, velocity, and effort values. The measured rate was 486.673 Hz, and the frame graph connected all robot links. | Pass |
| V-10 | Required TF chain is connected. | Run `tf2_echo odom base_link` and `view_frames`. | `odom → base_footprint → base_link` connects to wheels, caster, `laser_frame`, and `sensor_frame`. | `frames.pdf` contains the complete required hierarchy. Repeated `odom` to `sensor_frame` transforms were finite and valid after one initial listener-discovery message. | Pass |
| V-11 | Hazard follows robot position. | Echo `/hazard/level` at start and near the zone during a mission. | Value is near baseline at start and rises as distance to the configured zone decreases. | At approximately `(0.00, 0.00)`, hazard was 0.190728. At `(3.457343, 1.198117)`, only 0.110453 m from the configured zone center, hazard was 10.146841. | Pass |
| V-12 | Hazard noise is reproducible by configuration. | Run two equivalent launches with the same `random_seed` and compare the initial sequence under equivalent motion/time conditions. | Sequences are reproducible within the limits of identical callback timing; seed is queryable as a parameter. | Two independent nodes used seed 42, 2 Hz publication, and the same fixed odometry. Their capture files had an identical 81-sample common prefix; the first five values matched exactly. See `docs/evidence/seed_reproducibility.txt`. | Pass |
| V-13 | Start service activates the mission. | Call `/mission/start`. | Trigger response has `success=True`; state becomes `MOVING_TO_WAYPOINT`. | The START Trigger response reported success, and the captured state stream subsequently showed active `MOVING_TO_WAYPOINT` behavior. | Pass |
| V-14 | Abort service stops an active mission. | Start, then call `/mission/abort`. | Command becomes zero and state becomes `ABORTED`. | State changed from active waypoint motion to `ABORTED` with detail “Operator requested abort.” The first `ABORTED` CSV row and all subsequent abort rows contained zero linear and angular commands. | Pass |
| V-15 | Reset service clears mission progress. | From `ABORTED`, call `/mission/reset`. | State becomes `IDLE` and waypoint index is zero. | Captured status reported `IDLE`, waypoint index 0, and “Mission reset; waiting for a start request.” | Pass |
| V-16 | Rover follows the predefined route. | Run the default mission and observe Gazebo, RViz path, odometry, and state. | Rover visits station 1, passes the transit point, and approaches station 2. | The high-threshold capture showed waypoint 0 inspection, waypoint 1 transit, waypoint 2 inspection, and final completion. See `docs/evidence/completion_state.txt`. | Pass |
| V-17 | Inspection stops occur. | Observe `/mission/state` and `/cmd_vel` at station 1. | State enters `INSPECTING` for about 2 s and velocity remains zero. | `INSPECTING` was observed for approximately 2 s at both inspection waypoints. The corresponding CSV excerpt records zero linear and angular commands during `INSPECTING`. | Pass |
| V-18 | Hazard threshold stops normal motion. | Run default threshold 6.0; inspect states and CSV around crossing. | `HAZARD_DETECTED` follows moving/inspection; zero command is recorded before retreat. | At 64.200 s the state changed from waypoint motion to `HAZARD_DETECTED` with measured hazard 6.041. `RETREATING` began at 64.700 s after the configured 0.5 s stop interval. | Pass for state/stop ordering; CSV command row checked separately |
| V-19 | Hazard causes controlled retreat. | Continue default run after V-18. | State enters `RETREATING`, rover returns to the last safe route point, then enters `ABORTED`. | `RETREATING` began at 64.700 s; at 76.400 s the state became `ABORTED` with detail “Safe waypoint reached; mission remains aborted.” | Pass |
| V-20 | Completion branch exists. | Set threshold to 20.0, rebuild/relaunch, and start mission. | Both inspections complete and final state is `MISSION_COMPLETE`; restore threshold afterward. | Waypoints 0 and 2 entered `INSPECTING` for approximately 2 s, waypoint 1 was passed as transit, and the state became `MISSION_COMPLETE` at 69.400 s. See `docs/evidence/completion_state.txt`. | Pass |
| V-21 | Invalid hazard data leads to a safe state. | While active publish one `{data: .nan}` Float64 on `/hazard/level`. | Controller publishes zero, logs invalid data, and enters `ABORTED`. | A NaN Float64 was injected. State became `ABORTED` with detail “Mission aborted because hazard data is invalid,” and the next Twist contained six zero components. See `docs/evidence/invalid_hazard_stop.txt`. | Pass |
| V-22 | Missing/stale safety data stops motion. | In an isolated validation run, set a timeout below the corresponding publish interval or stop the relevant publisher. | Active mission publishes zero and enters `ABORTED` with stale/missing-data detail. | During active waypoint motion, `hazard_timeout` was set to 0.01 s, below the 10 Hz sensor interval. State became `ABORTED` with stale/missing-data detail, and the next Twist contained six zero components. The timeout and threshold were restored afterward. | Pass |
| V-23 | Planned route and status are visible in RViz. | Inspect `/mission/path` and `/mission/status_marker` displays. | Cyan fixed route and state/hazard text appear with robot, TF, scan, and odometry. | The privacy-safe integrated capture shows the rover, TF and scan, the cyan planned route, and readable `MOVING_TO_WAYPOINT` / `Hazard: 0.25` text. Both MissionPath and MissionStatus report `Status: Ok` on the correct topics. | Pass |
| V-24 | Logger records required fields. | Stop a mission cleanly and inspect newest CSV. | Header contains time, x, y, yaw, linear/angular velocity, hazard, and state; rows change during mission. | The header contains all eight required fields. Representative moving, inspecting, and complete rows contain changing pose, velocity, hazard, and state values. See `docs/evidence/csv_excerpt.txt`. | Pass |
| V-25 | Logger closes safely. | Stop launch with `Ctrl+C`, then open/tail the file. | File is readable, contains complete lines, and shutdown logs no unhandled file exception. | After shutdown, the CSV was readable and its final `MISSION_COMPLETE` row was complete. No logger file exception was present in the supplied launch output. | Pass |
| V-26 | Hazard calculation is unit tested. | Run `test_hazard_math.py`. | Center, far-field, noise clamp, and invalid-input cases pass. | Pytest collected 10 hazard-model cases; all 10 passed in 0.03 s. See `docs/evidence/test_results.txt`. | Pass |
| V-27 | Angle and waypoint calculations are unit tested. | Run `test_control_math.py`. | Angle wrapping and known 3–4–5 waypoint cases pass. | Pytest ran all 10 control-math cases as part of the 18-case control-package run; all passed. | Pass |
| V-28 | Velocity saturation is unit tested. | Run the saturation test. | Excess commands clamp to +0.35 and −1.20 in the chosen case. | The control-math suite containing the saturation case passed without failures. | Pass |
| V-29 | Hazard state transition is unit tested. | Run `test_state_machine.py`. | Moving → hazard → retreat → aborted cases pass. | Pytest ran all 8 state-machine cases; all passed. | Pass |
| V-30 | Invalid sensor/pose handling is unit tested. | Run invalid position, quaternion, waypoint, and abort tests. | Non-finite/zero-norm inputs are rejected and invalid-data abort reaches `ABORTED`. | Invalid-input cases in the hazard, control-math, and state-machine suites passed. | Pass |
| V-31 | All selected unit tests pass together. | Run `colcon test` for sim/control and `colcon test-result --verbose`. | 28 cases, zero errors and zero failures. | Separate colcon runs reported 10/10 simulation tests and 18/18 control tests, with no errors or failures; total 28. A final combined post-commit capture is recommended. | Pass with separate package-run evidence |
| V-32 | Documentation states scope honestly. | Review README and instructional material; video not supplied. | No claim of SLAM, AI, obstacle avoidance, field readiness, or safety certification. | Historical static-review result retained. The current overview states guided-learning scope and example limitations. The new learning review replaces application-drafting material. | Pass (static review) |

## Evidence naming convention

Use short, stable names under `docs/evidence/`. A text file can contain copied
CLI output; an image should show the relevant result without unrelated private
information.

```text
docs/evidence/
├── gazebo_world.png
├── rviz_integrated_view.png
├── frames.pdf
├── hazard_state_sequence.txt
├── test_results.txt
└── csv_excerpt.txt
```

For each artifact, record the commit hash and test date in this document or in
the file itself. Evidence from a different code revision should be identified
as such.

## Failure-handling procedure

When a row fails:

1. preserve the command and observed output;
2. state the smallest difference between expected and actual behavior;
3. decide whether the problem is code, configuration, environment, test
   method, or an incorrect requirement;
4. change only the smallest justified item;
5. rerun the failed test and relevant regression tests;
6. record the final result and retain a short note about the original failure.

Do not replace the entire project to fix one failed check. A well-explained
failure and focused correction can be stronger portfolio evidence than a claim
that everything worked immediately.

## Acceptance rule

The course project is ready to present when:

- all safety- and data-validity-related rows pass;
- all 28 unit-test cases pass;
- the default retreat and high-threshold completion branches are observed;
- one-command launch, TF, sensing, visualization, services, and CSV logging are
  demonstrated;
- any noncritical failed or conditional row has a documented limitation and
  next action;
- portfolio statements match the completed evidence.

Passing this educational plan does not certify the design for hazardous-field
operation or physical hardware.
