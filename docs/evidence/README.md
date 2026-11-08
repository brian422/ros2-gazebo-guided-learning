# Validation evidence index

This directory retains sanitized learning records dated 2026-08-08.
Text artifacts include curated summaries of reported runs, not necessarily
complete raw terminal output. The exact tested Git revision was not recorded.
These historical records are not a new validation of the current source.
Screenshots and summaries do not establish independent code authorship.

## Evidence record

| Evidence | Stored record | Original tested revision | Date | Status |
|---|---|---|---|---|
| Unit-test summary | [test_results.txt](test_results.txt) | Not recorded | 2026-08-08 | Pass: 10 simulation tests and 18 control tests |
| Active ROS node/topic/service graph | [ros_graph.txt](ros_graph.txt) | Not recorded | 2026-08-08 | Pass: required graph interfaces discovered |
| Default mission state sequence | [hazard_state_sequence.txt](hazard_state_sequence.txt) | Not recorded | 2026-08-08 | Pass for hazard stop and retreat; early inspection transition not captured |
| High-threshold completion result | [completion_state.txt](completion_state.txt) | Not recorded | 2026-08-08 | Pass: both inspections and final `MISSION_COMPLETE` observed |
| Sanitized mission-log excerpt | [csv_excerpt.txt](csv_excerpt.txt) | Not recorded | 2026-08-08 | Pass: required columns, changing rows, inspection stop, and final stopped state |
| Invalid-hazard safe stop | [invalid_hazard_stop.txt](invalid_hazard_stop.txt) | Not recorded | 2026-08-08 | Pass: NaN caused zero command and `ABORTED` |
| Operator abort and velocity limits | [operator_abort_velocity.txt](operator_abort_velocity.txt) | Not recorded | 2026-08-08 | Pass: limits respected; first and subsequent `ABORTED` rows commanded zero |
| Hazard position response | [hazard_position_response.txt](hazard_position_response.txt) | Not recorded | 2026-08-08 | Pass: near-baseline start and near-peak zone reading recorded |
| Seeded hazard reproducibility | [seed_reproducibility.txt](seed_reproducibility.txt) | Not recorded | 2026-08-08 | Pass: 81-sample common prefix matched across independent captures |
| Stale-hazard safe stop | [stale_hazard_stop.txt](stale_hazard_stop.txt) | Not recorded | 2026-08-08 | Pass: reduced timeout caused zero command and `ABORTED`; defaults restored |
| TF and sensor-runtime summary | [tf_sensor_runtime.txt](tf_sensor_runtime.txt) | Not recorded | 2026-08-08 | Pass: finite odometry, joint states, topic rates, and connected endpoint TF |
| Gazebo world screenshot | [gazebo_world.png](gazebo_world.png) | Not recorded | 2026-08-08 | Pass with runtime summary: world, rover spawn, and finite odometry verified |
| Integrated RViz screenshot | [rviz_integrated_view.png](rviz_integrated_view.png) | Not recorded | 2026-08-08 | Pass: robot, TF, scan, cyan route, and readable state/hazard marker visible |
| TF frame graph | [frames.pdf](frames.pdf) | Not recorded | 2026-08-08 | Pass: complete required tree is connected and legible |
| Final build and headless launch | [final_build_headless.txt](final_build_headless.txt) | Not recorded | 2026-08-08 | Pass: five packages built; server-only simulation ran without RViz |
| Demonstration video | External link in root README | Not recorded | Not recorded | Optional; not yet recorded |

## Sanitization rules

- Remove usernames, home-directory paths, emails, notifications, and unrelated
  terminal history.
- Keep commands, package names, topic names, state values, and measured values
  visible so the artifact remains technically useful.
- Prefer short text excerpts over full logs.
- Do not commit mission CSV files directly; `.gitignore` excludes them. Copy
  only selected rows into [csv_excerpt.txt](csv_excerpt.txt) after checking them.
- Record the tested Git commit so reviewers can connect evidence to source.

The stored results retain their original dates and limitations. The original
tested revision remains unrecorded; it must not be filled using a later
publication commit without source comparison or a new run. A demonstration
video has not been recorded. No historical results were regenerated during
the documentation review.
