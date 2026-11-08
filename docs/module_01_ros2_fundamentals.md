# Module 1 — ROS 2 workspace and communication fundamentals

**Estimated time:** 2–3 hours  
**Checkpoint:** two Python ROS 2 nodes exchange messages on one topic

If you are new to ROS 2 or the code feels overwhelming, begin with
[`module_01_beginner_start_here.md`](module_01_beginner_start_here.md). It
introduces the mental model and a shorter first study session before this full
lesson.

Do not continue to Module 2 until you can explain the data path and provide the
evidence listed near the end of this lesson.

For a detailed file-by-file and code-block explanation, including build and
runtime communication charts, read
[`module_01_file_and_code_guide.md`](module_01_file_and_code_guide.md).

## Learning objectives

By the end of this module, you should be able to:

1. Distinguish a workspace, package, Python module, ROS node, executable, topic,
   and message type.
2. Build one selected package with `colcon` and source its overlay.
3. Run Python nodes through ROS 2 entry points rather than direct file paths.
4. Inspect nodes and topics from a second terminal.
5. Change a publisher parameter and predict its effect before running it.
6. Explain why build, install, and log directories are not committed.

## Concepts introduced

- **Workspace:** a directory built as one unit by `colcon`; its source packages
  live below `src/`.
- **Package:** the smallest ROS 2 build and distribution unit. Here it is
  `hazard_inspection_control`.
- **Node:** one runtime process component with a unique graph name.
- **Publisher/subscriber:** decoupled endpoints exchanging typed messages.
- **Topic:** the named channel `/training/inspection_status`.
- **Message:** one instance of `std_msgs/msg/String`.
- **Entry point:** the mapping in `setup.py` that lets `ros2 run` find a Python
  `main()` function.
- **Overlay:** the local `install/setup.bash` environment layered over Humble.
- **QoS depth:** the queue capacity used if a subscriber briefly falls behind.

## Files created in this module

```text
hazard_inspection_ws/
├── .gitignore
├── LICENSE
├── README.md
├── docs/
│   ├── architecture.md
│   └── module_01_ros2_fundamentals.md
└── src/hazard_inspection_control/
    ├── package.xml
    ├── resource/hazard_inspection_control
    ├── setup.cfg
    ├── setup.py
    ├── config/module_01.yaml
    ├── launch/module_01_communication.launch.py
    └── hazard_inspection_control/
        ├── __init__.py
        ├── inspection_status_monitor.py
        └── inspection_status_publisher.py
```

## Understand each major file

### `package.xml`

Why it exists: it is the package manifest. It names the package, selects the
`ament_python` build type, and declares runtime dependencies.

How ROS 2 discovers it: `colcon` searches below `src/` for manifests. The
installed ament index then records the package.

Inputs/outputs: it is build metadata, not a runtime node. Inspect it with
`ros2 pkg xml hazard_inspection_control` after building and sourcing.

Intentional modifications for later:

1. Add a dependency only when a source file imports a new ROS message package.
2. Keep a generic project address or use a GitHub-provided no-reply address;
   do not publish a private email address.

### `setup.py`, `setup.cfg`, and the resource marker

Why they exist: `setup.py` installs the Python module, launch file, and YAML
configuration and registers two console executables. `setup.cfg` tells the
installer where scripts belong. The empty resource marker lets the ament index
discover the installed package.

How ROS 2 discovers them: `colcon` reads the manifest/build type and invokes the
Python packaging metadata. `ros2 run` resolves the installed console script.

Inputs/outputs: source files and data files go in; an installed package appears
under `install/hazard_inspection_control/`.

Independent test: after sourcing, run
`ros2 pkg executables hazard_inspection_control`.

Intentional modifications:

1. Add a third console script and observe that rebuilding is required.
2. Temporarily misspell an entry-point module path, predict when the failure
   appears, then restore it.

### `inspection_status_publisher.py`

Why it exists: it demonstrates a timer-driven publisher and validated ROS
parameters without depending on the later simulator.

How ROS 2 discovers it: `setup.py` maps `inspection_status_publisher` to the
module's `main()` function.

Input: parameters `station_name` and `publish_period_s`.  
Output: `std_msgs/msg/String` on `/training/inspection_status`.

Important code blocks:

- Constants name the topic and defaults so tunable values are not hidden.
- `declare_parameter()` creates a parameter contract visible to ROS tools.
- `_read_and_validate_parameters()` rejects an empty station or non-positive
  timer period before creating the timer.
- `create_publisher(..., 10)` creates a String publisher with a ten-message
  queue.
- `create_timer()` calls `_publish_status()` periodically.
- `main()` owns initialization, spinning, keyboard interruption, destruction,
  and safe shutdown.

Independent test:

```bash
ros2 run hazard_inspection_control inspection_status_publisher \
  --ros-args -p station_name:=pump_station_a -p publish_period_s:=0.5
```

Intentional modifications:

1. Change the status word from `READY` to `CHECKING` and explain why the
   subscriber needs no code change.
2. Add a `student_name` parameter and include it in the message without
   hard-coding it.

### `inspection_status_monitor.py`

Why it exists: it demonstrates the subscriber half of the topic contract.

How ROS 2 discovers it: its `setup.py` console entry point is
`inspection_status_monitor`.

Input: `std_msgs/msg/String` on `/training/inspection_status`.  
Output: a receipt count and message content in the terminal log.

Important code blocks:

- `create_subscription()` binds the type, topic, callback, and queue depth.
- Saving the returned subscription as `self._subscription` keeps it alive.
- `_status_callback()` increments state owned by the node and logs the payload.

Independent test: run the monitor alone, then inject a message with
`ros2 topic pub`; no publisher source code is required.

Intentional modifications:

1. Log only every fifth received message.
2. Count messages containing `READY` separately from other strings.

### `module_01.yaml`

Why it exists: parameters can be reviewed and version-controlled separately
from code. The node name scopes the values to the publisher.

How ROS 2 discovers it: the launch file resolves the installed package share
directory and passes this file to the publisher node.

Independent test: change the period to `0.5`, rebuild only if you are not using
`--symlink-install`, relaunch, and measure the topic rate.

Intentional modifications:

1. Rename `training_station` to an industrial location such as
   `compressor_station`.
2. Try a period of `0.25` and predict the measured rate.

### `module_01_communication.launch.py`

Why it exists: it starts both training nodes with one command while keeping the
node definitions explicit.

How ROS 2 discovers it: `setup.py` installs the file under the package share
directory; `ros2 launch` searches that directory.

Inputs: the installed YAML configuration.  
Outputs: two running nodes and their terminal logs.

Independent test: `ros2 launch hazard_inspection_control
module_01_communication.launch.py`.

Intentional modifications:

1. Change one node's `output` to `log` and observe the terminal difference.
2. Add a launch argument for the station name in a later exercise.

## Before running: make predictions

Write down your answers first:

1. At a `1.0` second publish period, how many messages should arrive in roughly
   ten seconds?
2. If the monitor starts five seconds before the publisher, what appears during
   those first five seconds?
3. If the topic name changes only in the publisher, will the monitor receive
   anything? Why?
4. Does changing a YAML file require Python source changes?

## Step-by-step commands

All commands below run inside Ubuntu WSL. The recommended build location is the
portable Linux path used throughout this course. A mounted Windows path usually
begins with `/mnt/c/`; keep that copy for editing or backup rather than builds.

### 1. Enter and inspect the workspace

```bash
cd ~/ros2_projects/hazard_inspection_ws
find . -maxdepth 4 -type f | sort
```

Expected: the files listed earlier in this lesson. The directory contains no
`build/`, `install/`, or `log/` yet.

### 2. Source the base ROS installation

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
```

Expected output:

```text
humble
```

### 3. Resolve declared dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Expected ending when dependencies are already present:

```text
#All required rosdeps installed successfully
```

### 4. Build only the Module 1 package

```bash
colcon build --symlink-install \
  --packages-select hazard_inspection_control
```

Expected output pattern (timing varies):

```text
Starting >>> hazard_inspection_control
Finished <<< hazard_inspection_control [<time>s]

Summary: 1 package finished [<time>s]
```

`--symlink-install` makes edits to installed Python and data files visible more
conveniently during development. Rebuild whenever packaging metadata or entry
points change.

### 5. Source the workspace overlay

Run this in every new terminal after sourcing Humble:

```bash
source install/setup.bash
ros2 pkg prefix hazard_inspection_control
ros2 pkg executables hazard_inspection_control
```

Expected executable lines:

```text
hazard_inspection_control inspection_status_monitor
hazard_inspection_control inspection_status_publisher
```

### 6. Run publisher and subscriber independently

Open **Terminal A**:

```bash
cd ~/ros2_projects/hazard_inspection_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hazard_inspection_control inspection_status_publisher
```

Expected output pattern:

```text
[INFO] [...] [inspection_status_publisher]: Publishing ...
[INFO] [...] [inspection_status_publisher]: Published: "READY | station=training_station | sequence=1"
```

Open **Terminal B**:

```bash
cd ~/ros2_projects/hazard_inspection_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hazard_inspection_control inspection_status_monitor
```

Expected output pattern:

```text
[INFO] [...] [inspection_status_monitor]: Waiting for messages on /training/inspection_status
[INFO] [...] [inspection_status_monitor]: Received #1: "READY | station=training_station | sequence=..."
```

Sequence may not begin at one because the publisher can run before the monitor.
Use `Ctrl+C` in each terminal for a clean shutdown.

### 7. Inspect the ROS graph from Terminal C

With both nodes still running:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic info /training/inspection_status --verbose
ros2 topic echo /training/inspection_status --once
ros2 topic hz /training/inspection_status
```

Stop `ros2 topic hz` with `Ctrl+C` after several samples. Expected behavior:

- node list includes `/inspection_status_publisher` and
  `/inspection_status_monitor`;
- the training topic type is `std_msgs/msg/String`;
- topic info reports one publisher and one subscriber (or two subscribers while
  `echo`/`hz` is active);
- `echo --once` prints one `data:` field;
- topic frequency is approximately `1.0 Hz`.

Inspect parameters:

```bash
ros2 param list /inspection_status_publisher
ros2 param get /inspection_status_publisher station_name
ros2 param get /inspection_status_publisher publish_period_s
```

### 8. Launch both nodes together

After stopping the independent nodes, run:

```bash
ros2 launch hazard_inspection_control module_01_communication.launch.py
```

Expected launch behavior: the launch service reports two started processes,
then publisher and monitor messages interleave in one terminal.

### 9. Test the monitor with a CLI-created publisher

Stop the launch. Run only the monitor in Terminal A, then in Terminal B run:

```bash
ros2 topic pub --once /training/inspection_status std_msgs/msg/String \
  "{data: 'MANUAL_CHECK | station=cli_station | sequence=999'}"
```

The monitor should log exactly one new receipt. This proves subscribers care
about the topic contract, not which source program produced the message.

### 10. Verify parameter validation

```bash
ros2 run hazard_inspection_control inspection_status_publisher \
  --ros-args -p publish_period_s:=0.0
```

Expected: startup fails with a clear message stating that
`publish_period_s` must be greater than zero. This is intentional evidence that
invalid configuration does not silently continue.

## Expected ROS behavior summary

| Item | Expected result |
|---|---|
| Publisher node | `/inspection_status_publisher` |
| Monitor node | `/inspection_status_monitor` |
| Topic | `/training/inspection_status` |
| Message type | `std_msgs/msg/String` |
| Normal publisher count | 1 |
| Normal subscriber count | 1 |
| Default rate | approximately 1 Hz |
| Default station | `training_station` |
| Invalid non-positive period | node refuses to start with a useful error |

## Common errors and troubleshooting

### `ros2: command not found`

Cause: Humble was not sourced or installed. Smallest correction:

```bash
source /opt/ros/humble/setup.bash
```

### `Package 'hazard_inspection_control' not found`

Cause: the package was not built, the build failed, or the overlay was not
sourced. Return to the workspace root, build, and run:

```bash
source install/setup.bash
```

### `No executable found`

Cause: stale packaging metadata or a missing entry point. Rebuild the selected
package and source the overlay again. Confirm names with
`ros2 pkg executables hazard_inspection_control`.

### Monitor receives nothing

Check all three parts of the contract: exact topic name, exact message type, and
whether a publisher exists:

```bash
ros2 topic info /training/inspection_status --verbose
```

ROS domain isolation can also hide nodes. In each terminal, compare:

```bash
echo "$ROS_DOMAIN_ID"
```

### YAML changes appear ignored

The YAML file is used by the launch file, not by a plain `ros2 run` unless you
pass `--params-file`. Stop old nodes, rebuild if needed, source the overlay, and
launch again.

### Build is slow under `/mnt/c`

WSL file access can be faster in the Linux filesystem. Moving the repository is
optional and changes its path; do not move it mid-module unless speed is a real
problem.

## Student exercise

Complete both parts without copying a second node:

1. Set `station_name` to `narrow_passage` and `publish_period_s` to `0.5` in
   `config/module_01.yaml`. Predict the new frequency first, launch, then record
   the measured `ros2 topic hz` average.
2. Add a declared integer parameter named `inspection_id` with default `1`,
   validate that it is positive, and include `inspection_id=<value>` in the
   published string.

Rebuild and verify. Be ready to explain which changes would be required if the
subscriber parsed fields rather than treating the payload as plain text.

## Debugging tasks

Perform these deliberately, one at a time, and then restore the correct code:

1. Change the monitor's topic constant to `/training/wrong_topic`. Use ROS CLI
   evidence to diagnose why both nodes run but no messages arrive.
2. Misspell `inspection_status_monitor` in the launch file executable field.
   Read the launch error and identify whether the problem is discovery, topic
   typing, or runtime logic.

## Evidence to collect

Save screenshots or text snippets showing:

1. successful `colcon build` summary;
2. `ros2 node list` with both nodes;
3. `ros2 topic info ... --verbose` with the correct type and endpoints;
4. one `ros2 topic echo --once` message;
5. `ros2 topic hz` near the predicted rate;
6. the intentional invalid-period error;
7. your completed exercise and its changed output.

## Questions to answer before Module 2

1. Why must every terminal source both Humble and the local overlay?
2. What connects the nodes: Python imports, process IDs, or a typed topic?
3. What does queue depth `10` mean, and what does it not guarantee?
4. Why is `/training/inspection_status` not used as the final mission-state
   interface?
5. Which file makes `ros2 run ... inspection_status_publisher` possible?
6. Why is parameter validation preferable to accepting a zero timer period?

## Completion checklist

- [ ] I built the package myself without unresolved errors.
- [ ] I ran each node independently and with the launch file.
- [ ] I found both nodes and the typed topic using ROS CLI tools.
- [ ] I measured the default and modified publication rates.
- [ ] I observed and understood the invalid-parameter failure.
- [ ] I completed both intentional code/configuration exercises.
- [ ] I can answer the six questions above in my own words.
- [ ] I saved the requested evidence.

## Suggested Git commits

If the workspace is not yet a Git repository:

```bash
git init
git add .gitignore LICENSE README.md docs src
git commit -m "chore: initialize ROS 2 workspace"
```

After completing and verifying the student exercise:

```bash
git add src/hazard_inspection_control docs/module_01_ros2_fundamentals.md
git commit -m "feat: add basic ROS 2 communication nodes"
```

Stop here and report your results before Module 2 is created.
