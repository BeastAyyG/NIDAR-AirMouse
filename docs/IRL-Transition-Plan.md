# Simulation-to-real transition plan

## Executive answer: how close can it be to real life?

The project can be made close to real life at the system-architecture level:
PX4 remains the flight-critical layer, ROS 2 runs mapping/perception/planning
on a companion computer, QGroundControl remains an operator interface, and
the same high-level mission states and grid outputs can be retained.

The present implementation is not yet close at the autonomy or physical
validation level. It flies a known, collision-free scripted route in SITL and
uses a synthetic detector that already knows marker coordinates. A physical
vehicle will add estimator drift, sensor time offset, vibration, latency,
lighting changes, propwash, battery sag, RF loss, floor effects, wind, and
unexpected obstacles. Those are separate engineering gates, not details that
Gazebo automatically proves.

### Current maturity snapshot

| Capability | Honest status |
| --- | --- |
| PX4/Gazebo/ROS 2 integration | Working simulation baseline |
| Offboard command path | Working in SITL; keep the PX4 heartbeat and takeover rules |
| Maze geometry | Defined and checked for the current scripted route |
| Sensor interfaces | Simulated `/scan` and `/odom`; physical drivers absent |
| Survivor perception | Synthetic known-target proxy only |
| Mapping/localization | Interface exists; full SLAM and loop closure absent |
| Exploration | Fixed waypoints only; no unknown-maze frontier planner |
| Physical aircraft | Not built/bench-validated by this repository |
| Competition readiness | Not claimed; rulebook and safety review still required |

## Recommended real baseline

The engineering book's recommended direction is a small propeller-guarded
quadrotor with a Pixhawk-class H7 autopilot, an existing 8 GB Jetson companion,
a 2D LiDAR such as RPLIDAR C1, an RGB/stereo depth camera such as OAK-D Lite,
and downward optical flow/range assistance. Treat this as a candidate baseline,
not a locked bill of materials. Verify voltage, mass, connector, driver,
mounting, compute, thermal, and current availability before buying parts.

PX4 is retained here because it is the requested simulation stack and has a
documented ROS 2 offboard path. An ArduPilot variant is possible, but it should
be a separate adapter/configuration track rather than a mid-flight-stack swap.

### Suggested responsibility split

| Function | Flight controller | Companion computer |
| --- | --- | --- |
| Motor mixing and stabilization | Owns | Never bypasses in normal development |
| Arming and disarming | Owns | Requests and verifies only |
| Battery, link, estimator, geofence failsafe | Owns | Monitors and triggers abort policy |
| Local position estimate | Owns primary estimate | Consumes and/or supplies a validated external estimate |
| LiDAR and camera drivers | No | Owns |
| SLAM/occupancy map | No | Owns |
| Survivor confidence and grid tag | No | Owns |
| Exploration and high-level route | No | Owns, through safety supervisor |
| Manual takeover | RC/autopilot | Must not block it |

## Work packages before physical autonomy

### 1. Freeze the mission contract

Obtain the current official rulebook and record a versioned clarification log.
Confirm arena dimensions, wall height/thickness, minimum gaps, maximum flight
time, allowed external computing, survivor markings, grid convention, launch/
land rules, manual override rules, and telemetry restrictions. Do not infer
current rules from the attached engineering book.

### 2. Make the simulation representative

Before hardware, extend the SDF and test harness to include:

- all six survivor targets and a coverage-complete route or planner;
- vehicle-envelope inflation with a configurable uncertainty margin;
- LiDAR range noise, dropout, finite update rate, and blind spots;
- camera field of view, exposure/lighting variation, blur, and target occlusion;
- estimator drift, delayed timestamps, dropped ROS messages, and planner latency;
- raised, low, hanging, and thin obstacles;
- battery/mission timeout and return-to-home behavior;
- random seeds and rosbag/replay tests so failures are reproducible.

The current route's 0.68 m estimated clearance is useful for the present
nominal model, but it is not a safety margin for a real vehicle. The planner
must use measured envelope plus localization and control uncertainty.

### 3. Bench-integrate the electronics

With propellers removed:

1. Flash and parameterize the flight controller.
2. Confirm sensor orientation, serial/Ethernet links, power monitoring, and
   time synchronization.
3. Verify QGroundControl connection and RC/manual takeover.
4. Verify companion-to-PX4 commands are rejected when stale or invalid.
5. Record motor order, failsafe parameters, battery thresholds, and firmware
   versions in a machine-readable manifest.
6. Run the full ROS graph from a recorded bag before permitting arm requests.

### 4. Calibrate and characterize sensors

Calibrate IMU and compass as required by the flight controller, and measure
camera/LiDAR/range extrinsics relative to `base_link`. Validate actual scan
rate, minimum/maximum range, camera depth quality, optical-flow behavior, and
timestamp skew. Test on the intended floor, wall material, lighting, and
altitude. Save calibration files with hardware serial numbers; never silently
reuse simulation values.

### 5. Fly in controlled increments

Use this progression: prop-off command test, restrained/tethered hover where
legally and safely appropriate, open-area manual hover, supervised position
hold, short autonomous setpoint test, controlled obstacle test, then a mock
maze. At every step, keep an operator with a tested takeover path and stop if
the estimator, link, battery, or command stream is not healthy.

### 6. Replace the scripted route

The required autonomy pipeline is:

```text
sensor drivers
  -> timestamp/extrinsic validation
  -> state estimation + TF
  -> occupancy map / SLAM
  -> frontier or coverage planner
  -> vehicle-envelope collision checker
  -> safety supervisor
  -> PX4 offboard setpoints
```

Survivor detections should be confidence-scored and temporally confirmed,
projected into the map with covariance, de-duplicated, and converted to the
agreed grid cell only after the aircraft is localized well enough. A single
camera frame should not cause a final validated tag.

## Data and software contracts

Keep the current simulation interfaces stable where possible:

- `/scan` remains the 2D range input contract.
- `/odom` becomes a tested estimate or is replaced by a documented TF/state
  source; never hide a frame change.
- `/airmouse/detections/map_point` is the replaceable perception boundary.
- `/airmouse/survivors/grid_tags` remains machine-readable and de-duplicated.
- `/airmouse/mission_status` must expose health, state, timestamp, current
  estimate, planner status, battery, and abort reason.
- `/airmouse/abort` must be safe to call repeatedly and must lead to a tested
  autopilot action, not just a text state change.

Add schema/version fields before the first public release. Reject unknown
schema versions rather than guessing.

## Definition of done for a real release

Do not label the project `real-ready` until all of the following are evidenced:

- current rulebook and arena survey are checked in or linked with provenance;
- six-target simulation coverage passes under noise and randomized seeds;
- hardware-in-the-loop or bench replay verifies every command/failsafe path;
- manual takeover, geofence, battery, link-loss, estimator-loss, and mission
  timeout behavior are tested;
- sensor calibration, frame tree, time synchronization, and logs are versioned;
- real detector performance is measured on representative data;
- mock-maze flights repeatably complete with a safety pilot;
- a risk owner signs off each flight gate.

## References

- [PX4 ROS 2 offboard control](https://docs.px4.io/main/en/ros/ros2_offboard_control)
- [PX4 offboard control safety guidance](https://docs.px4.io/main/en/ros/offboard_control)
- [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
- [RPLIDAR product information](https://www.slamtec.com/en/C1)
- [Luxonis documentation](https://docs.luxonis.com/)
