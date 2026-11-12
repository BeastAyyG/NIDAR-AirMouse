# AirMouse Gazebo Build — Completion and Verification Report

**Verification date:** 13 November 2026  
**Primary verification machine:** Jetson Orin Nano 8 GB Super  
**Scope:** Indoor Gazebo model, front depth camera, four optical-flow cameras, sensor-direction verification, and Gazebo-to-ROS 2 transport.

> **Summary:** The sensor-direction and ROS 2 bridge verification work is complete on the Jetson. All 16 signed-motion trials and all 11 configured ROS streams passed. PX4 receives flow messages in a separate disarmed SITL check. This is not a claim that the entire autonomous-flight project is complete or flight-safe.
>
> This document is suitable for a documentation-only GitHub update. It contains no SSH credentials, private keys, account details, Tailscale addresses, or machine-specific network identifiers. Uploading this Markdown does not upload the source code, compiled bridge, or raw evidence files.

## 1. Original checklist and final status

| Requested item | Status | What was actually completed |
|---|---|---|
| Review the AirMouse documents | **Complete for the clarified scope** | Read-only inspection of the existing Jetson project, roadmap, mission brief excerpts, workspace README, hardware handoff, and bridge implementation notes. A document index was created; originals were not moved or deleted. The rulebook was located, not comprehensively reviewed. |
| Add a front-facing depth camera | **Complete** | A Gazebo RGB-D camera was added to the X500 sensor rig and verified through actual Gazebo and ROS messages. |
| Add optical-flow sensors on the right, left, back and down | **Complete** | Four rendered camera feeds with independent image-based flow estimators. Each was tested in both directions on two axes. |
| Create the combined Gazebo model | **Complete** | X500 model with front RGB-D, four flow cameras, and a downward range sensor; a textured indoor building and separate test world are included. |
| Run PX4 SITL in an indoor building | **Executed, with qualifications** | Earlier runs took off indoors. After the flow-tracker fix, a separate PX4 ingestion check passed while disarmed. A new flight with the final tracker was not performed. |
| Run specifically on the teammate's laptop | **Not performed** | That machine was offline. The authoritative final sensor/bridge verification ran on the Jetson. Earlier development also used a Windows computer through WSL. |
| Verify model, depth and all-sided flow | **Complete for the defined tests** | Model runs; depth and other ROS streams flow; all four flow directions and scale are measured against geometry. This is not an unrestricted “100% working” claim. |

The user clarified that there is **no Monday deadline** for this work.

## 2. Verified environment

| Component | Jetson environment |
|---|---|
| Board | Jetson Orin Nano 8 GB Super |
| Architecture | ARM64 / aarch64 |
| Operating system | Ubuntu 22.04.5 LTS |
| Jetson software | L4T R36.4.7, JetPack 6.2.x family |
| Gazebo | Harmonic / Gazebo Sim 8.15.0 |
| ROS 2 | Humble |
| PX4 | Existing local tree reported `v1.18.0-beta1-547-g6041c961dd` |
| Bridge used for final tests | Private build linked to `gz-transport13` and `gz-msgs10` |
| Python environment | System Python with `PYTHONNOUSERSITE=1` for the launchers |

Earlier Windows/WSL development used Ubuntu 24.04 and PX4 v1.17.0. Those earlier tests must not be confused with the final Jetson results below.

## 3. Simulation assets delivered

### Drone and sensor model

The combined model is named `airmouse_x500_flow` and is based on the stock PX4 X500 model.

Body frame: **x forward, y left, z up**. Mount positions are relative to `base_link`.

| Sensor | Position in metres | View direction | Configuration |
|---|---|---|---|
| Front RGB-D | `(0.14, 0, 0.10)` | Forward, `+x` | 640×480, target 20 Hz, depth clip 0.2–8 m |
| Down optical flow | `(0.03, 0, -0.10)` | Down, `-z` | 100×100 camera, approximately 42° horizontal FOV, target 50 Hz |
| Left optical flow | `(0, 0.14, -0.03)` | Left, `+y` | Same flow-camera settings |
| Right optical flow | `(0, -0.14, -0.03)` | Right, `-y` | Same flow-camera settings |
| Back optical flow | `(-0.14, 0, -0.03)` | Back, `-x` | Same flow-camera settings |

A downward LW20/range-sensor configuration is also included. Range-finder accuracy was not a separate acceptance test in this task.

### Indoor building

- 12×12 m footprint with four rooms, partitions, door openings and obstacles.
- Ceiling at approximately 3.5 m to represent a covered indoor environment.
- Textured floor and walls so optical-flow algorithms have visible image features.
- Normal-gravity world for PX4 SITL.
- Separate gravity-free, velocity-commanded world for controlled sensor verification.

The controlled test world is deliberately not a realistic flight-control test: it isolates sensor measurements from controller and estimator behavior.

## 4. Optical-flow direction and scale verification

**Result: 16 of 16 trials passed on the Jetson.**

Each camera was tested along two body axes perpendicular to its viewing direction, in positive and negative directions:

| Camera | Signed translation trials | Passed |
|---|---|---|
| Down | `+x`, `-x`, `+y`, `-y` | 4/4 |
| Left | `+x`, `-x`, `+z`, `-z` | 4/4 |
| Right | `+x`, `-x`, `+z`, `-z` | 4/4 |
| Back | `+z`, `-z`, `+y`, `-y` | 4/4 |

### Method

1. Place the drone at a known position facing a textured planar surface.
2. Translate it without rotation.
3. Measure the actual `base_link` displacement from Gazebo pose messages.
4. Calculate the expected integrated angular image motion from displacement and surface range.
5. Compare expected and measured flow sign, scale and cross-axis leakage.
6. Count positive-quality samples only.

Ground-truth pose and geometry are used **only by the evaluator**. The flow estimator receives rendered images, not ground-truth motion.

The inherited X500 `base_link` is 0.24 m above its model origin. The evaluator explicitly accounts for this instead of using model-origin altitude as camera range.

### Final measured results

| Camera | Measured/expected scale ratio across its four trials | Lowest trial-mean quality |
|---|---|---|
| Down | 0.9792–0.9842 | 254.90/255 |
| Left | 1.0072–1.0105 | 254.91/255 |
| Right | 0.9906–0.9979 | 254.35/255 |
| Back | 0.9905–0.9975 | 254.29/255 |

- Maximum absolute scale error: **2.08%**.
- Maximum cross-axis leakage: **0.057%**.
- Acceptance limits: scale within **15%**, cross-axis leakage below **10%**, sufficient positive-quality samples, and sufficient actual translation.
- Flow directions matched the geometry prediction in every trial.

These near-maximum quality values apply to the controlled textured simulation, not real-world sensor performance.

### Flow output meaning

The outputs represent **image-axis angular motion**, not body-frame linear velocity.

- PX4-compatible `px4.msgs.OpticalFlow`: integrated image displacement in radians, integration duration and quality.
- Diagnostic ROS `/airmouse/flow/{down,left,right,back}` uses `geometry_msgs/Vector3`:
  - `x`: horizontal angular image rate, rad/s.
  - `y`: vertical angular image rate, rad/s.
  - `z`: quality from 0 to 255; **not a third flow axis**.
- The diagnostic ROS message has no timestamp and is not a drop-in EKF measurement interface.
- PX4's existing consumer uses downward flow. Side and back streams are not automatically fused into PX4.

## 5. Gazebo-to-ROS 2 bridge verification

**Result: all 11 configured streams passed during a 35-second live observation.**

These are received message counts, not target rates or a claim of real-time performance.

| ROS stream | ROS message type | Messages received |
|---|---|---:|
| `/clock` | `rosgraph_msgs/Clock` | 4,176 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 312 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 244 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 362 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 362 |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 180 |
| `/camera/imu` | `sensor_msgs/Imu` | 4,228 |
| `/airmouse/flow/down` | `geometry_msgs/Vector3` | 904 |
| `/airmouse/flow/left` | `geometry_msgs/Vector3` | 904 |
| `/airmouse/flow/right` | `geometry_msgs/Vector3` | 903 |
| `/airmouse/flow/back` | `geometry_msgs/Vector3` | 905 |

### What was checked

- Required streams delivered actual messages; merely appearing in `ros2 topic list` was not sufficient.
- Zero invalid messages were reported by the implemented checks.
- No backward timestamps were observed in timestamped streams.
- RGB images were 640×480 with `rgb8` encoding.
- Depth images were 640×480 with `32FC1` encoding and finite positive depth values.
- The final inspected depth image had approximately **87.61% finite pixels**.
- Camera intrinsics were valid; both focal lengths were approximately **467.74 pixels**.
- Point clouds were organized 640×480 with `x`, `y`, `z` and `rgb` fields and a valid payload layout.
- IMU linear-acceleration components and diagnostic flow values were finite.
- ROS image payload hashes were compared with Gazebo using matching timestamps: **312 RGB and 244 depth exact matches**.
- Flow values were compared across transports: **528 down, 532 left, 530 right and 528 back exact distinct value matches**.

Both CameraInfo outputs use Gazebo `/camera/camera_info`, because this RGB-D configuration shares its intrinsics. The old assumed `/camera/depth_camera_info` source was not used.

### Negative control

The same bridge checker was run for three seconds **before starting the bridge**, in the isolated test domain. It correctly failed and reported missing ROS streams. After starting the private Harmonic bridge, the normal check passed.

This guards against a false pass caused by checking advertised topics alone or accidentally receiving unrelated sensor traffic.

## 6. PX4 SITL integration check

A separate final check started PX4 SITL with the independent flow publisher, without arming or taking off.

Observed on PX4 `sensor_optical_flow`:

| Field | Observed value |
|---|---|
| Fresh message | Timestamp was approximately 8 ms old at inspection |
| Integration interval | 20,000 µs |
| Error count | 0 |
| Pixel flow | `[0, 0]` while stationary |
| Quality | 0 in this stationary, on-ground check |
| Gyro delta available | False |
| Embedded distance available | False |
| Vehicle state | Disarmed |

**Conclusion:** the final publisher-to-PX4 message path is connected. This check does not establish flight-quality flow at ground level, gyro compensation, range fusion, or stable flight with the final tracker.

Earlier simulation runs did take off indoors, including a Jetson run that subsequently descended and disarmed. Those runs predate the final tracker fix. The earlier landing cause was not confirmed, and no replacement flight result is claimed here.

## 7. Root causes found and fixed

### A. Shared state between four optical-flow cameras

The inspected bundled PX4-OpticalFlow code uses static previous-image/feature state and a shared rate-limiter timestamp. Multiple camera instances can therefore mix image histories and rate state.

**Fix:** independent per-camera Lucas–Kanade tracking in this simulation package. The stock PX4 tree was not patched. Camera-only includes avoid running the old custom tracker alongside the new publisher.

### B. Fortress bridge used with Harmonic simulator

The installed system Humble bridge was linked to `ignition-transport11` and `ignition-msgs8`, while Gazebo Harmonic uses `gz-transport13` and `gz-msgs10`. The bridge could announce mappings without delivering the required data.

**Fix:** compile a private Harmonic-compatible bridge from the existing local source with `GZ_VERSION=harmonic`. Its matching library dependencies were verified. The system bridge was not replaced.

### C. Floor texture hidden by coplanar geometry

A nominal 0.5 mm floor gap rounded away when coordinates were serialized to three decimal places, placing the textured floor at ground-plane height.

**Fix:** a representable 1 mm gap. Excessive lighting was also reduced.

### D. Incorrect evaluator reference height

Using the model origin as `base_link` underestimated downward-camera range because of the inherited 0.24 m X500 offset.

**Fix:** include the actual base-link offset in evaluator placement and geometry calculations.

### E. Invalid frames counted as evidence

The early check counted nonzero values even when the bundled tracker marked the sample quality as `-1`.

**Fix:** the authoritative direction check uses positive-quality measurements and checks sign and scale, not nonzero values alone. Older weak-quality reports are historical, not the final acceptance evidence.

## 8. Implementation and regression tests

Four synthetic-image unit tests passed:

1. Translation sign and scale for positive/negative movement on both image axes.
2. Four camera instances retain independent previous images.
3. Blank images return invalid/zero-quality flow rather than fabricated motion.
4. Duplicate or reset simulation timestamps are handled.

Shell syntax and Python compilation checks also passed for the final relevant launchers and verification scripts.

No comprehensive repository-wide coverage percentage or full autonomy regression result is claimed.

## 9. Delivered files and evidence locations

The deployed working folder on the Jetson is `~/airmouse_gazebo`. The current repository contains the corresponding `airmouse_gazebo/` folder.

| Relative path | Purpose |
|---|---|
| `models/airmouse_x500_flow/` | Combined X500 sensor model |
| `models/airmouse_depth_cam/` | Front RGB-D camera |
| `models/airmouse_flow_camera/` | Camera-only optical-flow include |
| `models/indoor_building/` | Textured indoor geometry |
| `worlds/indoor_building.sdf` | Normal-gravity PX4 world |
| `worlds/indoor_building_test.sdf` | Controlled sensor-verification world |
| `scripts/flow_tracker.py` | Independent image tracker |
| `scripts/flow_transport.py` | Gazebo flow publishing |
| `scripts/verify_flow_direction.py` | Signed-motion evaluator |
| `scripts/verify_ros_bridge.py` | Actual ROS message and content checks |
| `scripts/run_verification.sh` | Main verification entry point |
| `scripts/run_px4_sitl.sh` | Separate disarmed PX4 ingestion check |
| `config/gz_bridge.yaml` | Tested topic mapping |
| `.build/` and `.install/` | Private bridge build/install on Jetson; not portable compiled artifacts |
| `reports/flow_direction_jetson.json` | Final 16-trial evidence |
| `reports/ros_bridge_jetson.json` | Final 11-stream evidence |
| `reports/bridge_negative.json` | Bridge-off negative control |
| `reports/px4_flow_uorb.txt` | Final PX4 flow sample |
| `reports/px4_vehicle_status.txt` | Disarmed vehicle state |

The evidence files were retained in the working folders. A Markdown-only GitHub update publishes this report's summary, **not the underlying artifacts or implementation**. Do not claim the full build has been published unless those files are separately included.

## 10. Reproduction on the prepared Jetson

These commands require the deployed working folder and its existing dependencies; the Markdown file by itself is not an installer.

```bash
cd ~/airmouse_gazebo

# Signed-flow and ROS bridge checks, with process cleanup:
bash scripts/run_verification.sh

# Fast image-tracker regression checks:
PYTHONNOUSERSITE=1 python3 -m unittest discover -s scripts -p test_flow_tracker.py

# Separate PX4 ingestion check; remains disarmed:
bash scripts/run_px4_sitl.sh
```

If the private bridge must be rebuilt on that prepared Jetson:

```bash
bash scripts/build_harmonic_bridge.sh
```

- The main verifier uses a unique Gazebo partition, ROS domain 67 and localhost-only ROS discovery.
- Cleanup targets only the process groups created by the verifier, not arbitrary existing simulations.
- The PX4 smoke test refuses to start if another PX4 process already exists.
- No test simulation processes remained running at the final process check.

## 11. Existing project documents reviewed

A read-only document index was created in the deployed folder as `PROJECT-CONTEXT.md`.

Relevant originals include:

- `Mission Brief - NIDAR AirMouse.pdf`: GPS-denied indoor search, mapping, survivor localization and GCS live feed requirements; relevant excerpts inspected.
- `NIDAR-26-27-Rulebook-Ver_2_1.pdf`: located; not comprehensively reviewed or certified against this build.
- `AirMouse_Simulation_Development_Roadmap.md`: simulation-first requirements and expected ROS sensor topics.
- Two duplicate roadmap copies: identical MD5 at review time; retained rather than deleted.
- Existing `airmouse_autonomy/HANDOFF.md` and deployment/video/runtime documents.
- Existing `nidar_airmouse_sim/README.md` and bridge source notes explaining the older Fortress/Harmonic mismatch.

The broader project is **autonomous GPS-denied indoor search, mapping and survivor localization**. This task completes the simulation sensor and transport verification slice, not the entire roadmap.

## 12. Remaining boundaries and work outside this completion claim

The following have **not** been established by these results:

- Flight with the final independent tracker, sustained hover, or autonomous indoor navigation.
- Correct flow under arbitrary rotations or gyro-compensated optical flow.
- Real optical-flow hardware calibration or accuracy.
- Robustness on featureless surfaces, poor lighting, dropouts or all building layouts.
- Automatic PX4 fusion of the side/back sensors.
- Complete SLAM, collision avoidance, exploration, survivor detection or mission completion.
- A full optical-frame/body-frame TF integration: the current RGB-D message frame identifier is `camera_link`; downstream consumers still require the correct coordinate transform.
- Out-of-the-box IMU remapping for a PX4-spawned model ending in `_0`; the tested bridge configuration names the controlled test model.
- Performance on the offline teammate's machine.
- Compliance with every competition rule or physical-flight safety.

### Final acceptance statement

**Completed:** four-camera signed optical-flow verification, front RGB-D and associated ROS streams, a working Harmonic-compatible bridge, a disarmed PX4 flow-ingestion check, and a read-only project-document review.

**Not claimed:** a fully finished autonomous aircraft, physical-flight readiness, a completed entire-project roadmap, or an uploaded code release.
