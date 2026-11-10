# AirMouse autonomy replay validation report

Offline replay of the ROS-independent autonomy cores (occupancy mapping, 
frontier exploration, local planning, survivor tracking) against a 
deterministic synthetic ground-truth world. Ground truth is used only to 
synthesize sensor observations and to score the result; it is never fed 
back into the autonomy pipeline.

## Mission summary

| Metric | Value |
|---|---|
| Mission complete | yes |
| Duration (sim s) | 87.3 |
| Explored (% of grid observed) | 99.7% |
| Steps | 1746 |
| Wall-clock runtime (s) | 1.82 |

## Map quality (ground-truth comparison)

| Metric | Value |
|---|---|
| Map F1 (OCCUPIED positive class) | 0.982 |
| Precision | 0.980 |
| Recall | 0.984 |
| True positives | 1086 |
| False positives | 22 |
| False negatives | 18 |
| Collisions (truth footprint) | 0 |

## Survivors

| Metric | Value |
|---|---|
| Survivors in truth | 3 |
| Tracks | 3 |
| Confirmed tracks | 3 |
| Truth positions matched | 3 |

| Track | x (m) | y (m) | State | Confidence | Sightings |
|---|---|---|---|---|---|
| 0 | 2.57 | 2.87 | confirmed | 1.00 | 13 |
| 1 | 12.94 | 2.98 | confirmed | 1.00 | 23 |
| 2 | 11.87 | 12.08 | confirmed | 1.00 | 21 |

## Room coverage

| Region | Explored | Status |
|---|---|---|
| NW | 100% | SEARCHED |
| NE | 100% | SEARCHED |
| SW | 100% | SEARCHED |
| SE | 100% | SEARCHED |

## Roadmap acceptance comparison

| Criterion | Target | Measured | Status |
|---|---|---|---|
| Map F1 (OCCUPIED vs truth) | >= 0.90 | 0.982 | PASS |
| Localization error | <= 0.20 m | truth baseline (0.000 m) | UNVERIFIED-OFFLINE |
| Mission completion | >= 95% of trials | 100% (1 offline replay) | PASS |
| Collisions | 0 | 0 | PASS |
| Unique survivors | <= 6 | 3 tracked / 3 in truth | PASS |
| Manual waypoints | 0 | 0 (autonomous goal selection) | PASS |
| Mission duration | < 1800 sim s | 87.3 s | PASS |
| Sensor availability | 100% nominal | not measured offline | UNVERIFIED-OFFLINE |
| Obstacle recall | >= 95% | map recall proxy 98.4% | UNVERIFIED-OFFLINE |

Items marked UNVERIFIED-OFFLINE require ROS 2 / Gazebo / PX4 runtime hardware and are not fabricated offline.

## UNVERIFIED-OFFLINE

- Localization error target (<= 0.20 m): the replay localizes from truth (DEV baseline). VIO / RGB-D SLAM integration is a Goal-4 runtime follow-up.
- Sensor availability (100% nominal): needs the Gazebo RGB-D + IMU bringup.
- Obstacle recall (>= 95%): needs the depth -> obstacle pipeline in simulation; the map recall above is an offline occupancy proxy only.
- colcon build, PX4 offboard/arming/landing, TF tree and rosbag capture: Goal-0 checklist on the Ubuntu 22.04 / ROS 2 Humble target machine.

## Artifacts

- `map.png` - occupancy map with survivor-track and room overlays
- `result.json` - full result (numpy arrays excluded)

Simulation success is not proof of real-world flight safety.
