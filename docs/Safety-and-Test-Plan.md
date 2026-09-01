# Safety and test plan

This document applies to the physical transition. A simulation finishing
`LAND/COMPLETE` is not permission to arm a real aircraft.

## Non-negotiable controls

- A trained operator must have a tested manual takeover path.
- Propellers remain removed for bench software and command tests.
- Use a controlled, legally permitted flight area and follow local aviation,
  venue, and team safety requirements.
- Use physical arming discipline, a visible disarm state, and an independent
  emergency stop/power procedure appropriate to the aircraft.
- Never rely on ROS 2, Wi-Fi, QGroundControl, or the companion computer as the
  only safety mechanism.
- Keep people outside the rotor envelope and establish a hard safety boundary.
- Stop testing if battery, estimator, link, temperature, vibration, or motor
  behavior leaves the approved envelope.

PX4's own offboard documentation warns that offboard control is dangerous and
requires a way to regain manual control. Treat this as a design requirement,
not a README note.

## Verification gates

| Gate | Test | Pass evidence |
| --- | --- | --- |
| S0 | Documentation/rule review | Current rulebook, arena survey, owner, and rollback plan |
| S1 | Static code/config review | No hard-coded personal paths, no real-arm default, explicit frames |
| S2 | Offline unit tests | Grid, coordinate conversion, route inflation, malformed-message tests |
| S3 | SITL nominal | PX4 precheck, takeoff, mission, two tags, RTL/land/disarm |
| S4 | SITL fault injection | Stale setpoint, lost ROS node, lost bridge, estimator loss, battery/time limit |
| S5 | Hardware prop-off | Telemetry, mode changes, arm/disarm request, RC takeover, sensor health |
| S6 | Restrained/controlled hover | Stable estimator and manual recovery with safety pilot |
| S7 | Open-area autonomy | Short setpoint mission with geofence and abort tests |
| S8 | Mock maze | Coverage, obstacle clearance, survivor confirmation, return/land repeatability |
| S9 | Release review | Logs, firmware/config hashes, calibration, risk sign-off, rule compliance |

No gate may be skipped because an earlier simulation passed.

## Required failure behavior

The safety supervisor and PX4 configuration must define and test responses for:

- missing or stale offboard heartbeat;
- malformed, out-of-frame, or out-of-geofence setpoints;
- ROS node, DDS agent, Wi-Fi, or companion-computer loss;
- estimator invalidity or position jump;
- LiDAR/camera/range sensor timeout or implausible data;
- low battery, high current, overheating, or excessive vibration;
- mission timeout or no-progress condition;
- manual takeover and emergency disarm.

The response must be an actual autopilot action or operator action with a
measurable timeout. Publishing `ABORT` text alone is not a failsafe.

## Simulation test cases

At minimum, add automated tests for:

1. Gazebo-to-PX4 ENU/NED conversion round trips.
2. Route collision checks with multiple vehicle radii and uncertainty margins.
3. Geofence rejection at every boundary.
4. LiDAR clear, stale, invalid, stop-distance, and emergency-distance cases.
5. Duplicate survivor detections and grid-cell de-duplication.
6. Detection confidence and temporal confirmation rules.
7. Stale timestamps and delayed messages.
8. Planner no-path, no-progress, and timeout handling.
9. RTL/land/disarm state transitions after LiDAR abort.
10. Coverage of all six markers under randomized sensor noise.
11. Clean startup/stop/restart without orphaned PX4, Gazebo, ROS, or agent processes.

## Evidence to save per flight

Record the date, operator, aircraft ID, firmware hash, parameter export,
companion software commit, sensor calibration versions, battery ID, arena
layout, weather/lighting notes, ROS bag or approved summary, QGroundControl
flight log, preflight checklist, anomalies, and release decision. Redact
personal data before public GitHub upload.

## Abort checklist

When the aircraft or mission is not healthy:

1. Take manual control or invoke the tested autopilot fallback.
2. Command RTL/land only if the current position and altitude make that safe;
   otherwise use the approved emergency action.
3. Disarm when clear and safe.
4. Isolate battery/power as required.
5. Preserve logs before restarting.
6. Record the failure and do not repeat the test until the owner signs off the
   corrective action.
