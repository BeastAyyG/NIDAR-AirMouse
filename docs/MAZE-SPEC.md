# AirMouse maze specification

This is the measured specification of the current `PX4-gazebo-models/worlds/maze.sdf`.
All dimensions are metres. The Gazebo world uses ENU-style `x/y` coordinates
for this project. PX4 local position uses NED and is mapped as:

```text
px4_x_north = gazebo_y
px4_y_east  = gazebo_x
px4_z_down  = -gazebo_z
```

## Global envelope

| Item | Value |
| --- | ---: |
| Ground visual | 60 x 60 x 0.1 |
| Outer wall centerline rectangle | 30 x 30 |
| Outer wall thickness | 0.4 |
| Outer wall height | 6.0 |
| Outer collision extent | x/y = -15.2 to +15.2; z = 0 to 6 |
| Usable interior between inner wall faces | approximately -14.8 to +14.8 in x/y |
| Configured mission geofence | PX4 x/y = -14 to +14; z = -4 to +0.5 |
| Vehicle used in the route check | `gz_x500_lidar_2d`, approximate horizontal envelope radius 0.32 |

The outer walls are:

| Model | Center | Size |
| --- | --- | --- |
| `wall_outer_west` | (-15, 0, 3) | (0.4, 30, 6) |
| `wall_outer_east` | (15, 0, 3) | (0.4, 30, 6) |
| `wall_outer_south` | (0, -15, 3) | (30, 0.4, 6) |
| `wall_outer_north` | (0, 15, 3) | (30, 0.4, 6) |

## Internal walls

Each internal wall is a solid collision box. The listed size is `(x, y, z)`.

| Model | Center `(x,y,z)` | Size `(x,y,z)` |
| --- | --- | --- |
| `wall_a` | (-10, -8, 3) | (0.4, 14, 6) |
| `wall_b` | (-4, 8, 3) | (0.4, 14, 6) |
| `wall_c` | (2, -8, 3) | (0.4, 14, 6) |
| `wall_d` | (9, 8, 3) | (0.4, 14, 6) |
| `wall_e` | (-7, 4, 3) | (10, 0.4, 6) |
| `wall_f` | (6, -4, 3) | (10, 0.4, 6) |
| `wall_g` | (7, 4, 3) | (8, 0.4, 6) |
| `wall_h1` | (-8, -10, 3) | (12, 0.4, 6) |
| `wall_h2` | (9, -10, 3) | (8, 0.4, 6) |
| `wall_h3` | (-10, 10, 3) | (8, 0.4, 6) |
| `wall_h4` | (7, 10, 3) | (8, 0.4, 6) |
| `wall_v1` | (-4, -1, 3) | (0.4, 6, 6) |
| `wall_v2` | (10, -1, 3) | (0.4, 6, 6) |

## Survivor markers

There are six visual markers. Each is a red cylinder with a yellow head;
these are not a realistic human-survivor perception model.

| ID | Gazebo ENU position |
| --- | --- |
| `survivor_01` | (-12, -12, 0.45) |
| `survivor_02` | (-7, -6, 0.45) |
| `survivor_03` | (0, -12, 0.45) |
| `survivor_04` | (12, -12, 0.45) |
| `survivor_05` | (-12, 12, 0.45) |
| `survivor_06` | (12, 12, 0.45) |

The current synthetic detector uses a 2.5 m detection radius. The current
mission route validates two markers only: `survivor_02` and `survivor_03`.
The other four markers are test data for future coverage work, not an
assertion that they are reachable by the present vehicle and route.

## Current route and clearance

The configured PX4 local-NED waypoints are:

```text
(-6,  0, -2.5)
(-6, -7, -2.5)
(-6,  0, -2.5)
(-12, 0, -2.5)
( 0,  0, -2.5)
```

In Gazebo ENU this is:

```text
( 0,  0, 2.5) home
( 0, -6, 2.5)
(-7, -6, 2.5)
( 0, -6, 2.5)
( 0,-12, 2.5)
( 0,  0, 2.5)
```

An axis-aligned static geometry check using a 0.32 m vehicle-envelope radius
found no route collision. The minimum static gap from the route centerline to
a wall collision-box edge was 1.0 m, leaving an estimated 0.68 m clearance
after that envelope inflation. This is not a full dynamics, prop-wash,
estimator-error, or latency proof; run the planner against inflated obstacles
and test noise before changing the route.

Run the repeatable check from the repository root:

```bash
python3 tools/validate_maze.py
```

## Competition arena: `maze_compete.sdf`

A second SDF (`PX4-gazebo-models/worlds/maze_compete.sdf`) ships alongside
the practice course. It is a smaller 15 m x 15 m arena with five parallel
north-south corridors and a serpentine route that must reverse direction
four times. Every survivor in this arena is expected to be reachable.

The validator is parameterised by `--world`. Use `--world all` to check
both arenas in one run:

```bash
python3 tools/validate_maze.py --world all
python3 tools/validate_maze.py --world maze_compete
```

### Global envelope

| Item | Value |
| --- | ---: |
| Ground visual | 40 x 40 x 0.1 |
| Outer wall centerline rectangle | 15 x 15 |
| Outer wall thickness | 0.4 |
| Outer wall height | 6.0 |
| Outer collision extent | x/y = -7.7 to +7.7; z = 0 to 6 |
| Usable interior between inner wall faces | approximately -7.3 to +7.3 in x/y |
| Configured mission geofence (`airmouse_compete.yaml`) | PX4 x/y = -7 to +7; z = -4 to +0.5 |
| Vehicle used in the route check | `gz_x500_lidar_2d`, approximate horizontal envelope radius 0.32 |

The outer walls are:

| Model | Center | Size |
| --- | --- | --- |
| `wall_outer_west` | (-7.5, 0, 3) | (0.4, 15, 6) |
| `wall_outer_east` | (7.5, 0, 3) | (0.4, 15, 6) |
| `wall_outer_south` | (0, -7.5, 3) | (15, 0.4, 6) |
| `wall_outer_north` | (0, 7.5, 3) | (15, 0.4, 6) |

### Internal dividers

Five corridors centred on x = -6, -3, 0, +3, +6 (corridor half-width 1.3 m).
The four dividers alternate between a south gap + a north short stub, and a
mid-span gap, so the route must reverse four times to traverse every
corridor.

| Model | Center `(x,y,z)` | Size `(x,y,z)` | Description |
| --- | --- | --- | --- |
| `wall_v1a` | (-4.5, -4.6, 3) | (0.4, 5.8, 6) | Corridor divider 1 - south segment |
| `wall_v1b` | (-4.5, 3.0, 3) | (0.4, 2.6, 6) | Corridor divider 1 - north short stub |
| `wall_v2` | (-1.5, -0.2, 3) | (0.4, 9.0, 6) | Corridor divider 2 - mid-span gap |
| `wall_v3a` | (1.5, -4.6, 3) | (0.4, 5.8, 6) | Corridor divider 3 - south segment |
| `wall_v3b` | (1.5, 3.0, 3) | (0.4, 2.6, 6) | Corridor divider 3 - north short stub |
| `wall_v4` | (4.5, -0.2, 3) | (0.4, 9.0, 6) | Corridor divider 4 - mid-span gap |

### Survivor markers

Six markers, alternating between the southern and northern halves of
alternating corridors, so every marker must be visited to validate the
run.

| ID | Gazebo ENU position |
| --- | --- |
| `survivor_01` | (-6.7, -3.0, 0.45) |
| `survivor_02` | (-5.3, 3.0, 0.45) |
| `survivor_03` | (-3.7, -3.0, 0.45) |
| `survivor_04` | (0.7, 3.0, 0.45) |
| `survivor_05` | (2.3, -3.0, 0.45) |
| `survivor_06` | (6.7, 3.0, 0.45) |

### Configured route and clearance

The configured PX4 local-NED waypoints in `airmouse_compete.yaml` are:

```text
(-6,  0, -2.5)    (-6, -3, -2.5)    ( 6, -3, -2.5)
( 6, -6, -2.5)    (-6, -6, -2.5)    ( 0, -6, -2.5)
( 0, -3, -2.5)    (-6, -3, -2.5)    (-6,  0, -2.5)
( 6,  0, -2.5)    ( 6,  3, -2.5)    (-6,  3, -2.5)
(-6,  6, -2.5)    ( 6,  6, -2.5)    ( 6,  3, -2.5)
( 0,  3, -2.5)    ( 0,  0, -2.5)
```

In Gazebo ENU this is:

```text
( 0,  0, 2.5)    (-3,  0, 2.5)    (-3,  6, 2.5)
( 3,  6, 2.5)    ( 3,  0, 2.5)    ( 0,  0, 2.5)   <- corridor 0 lap
(-3, -6, 2.5)    ( 3, -6, 2.5)    (-3, -6, 2.5)  <- corridor +3
( 0, -6, 2.5)    (-6, -6, 2.5)    (-6,  0, 2.5)  <- corridor -3
( 0,  6, 2.5)    ( 3,  6, 2.5)    ( 3,  0, 2.5)   <- corridor 0 again
(-6,  6, 2.5)    ( 6,  6, 2.5)    ( 6,  3, 2.5)   <- corridor +6
( 3,  3, 2.5)    ( 0,  3, 2.5)    ( 0,  0, 2.5)   <- corridor -3 north + return
```

All 18 route segments are collision-free for the 0.32 m envelope. The
minimum static gap from any route centerline to a wall edge was 1.30 m,
leaving 0.98 m clearance after envelope inflation. All six survivors are
reachable from the home cell under flood-fill connectivity at 0.10 m
resolution.

### Running it

`run_airmouse_sim.sh` accepts `AIR_MOUSE_WORLD=maze_compete` and will copy
the matching SDF into `PX4-Autopilot/Tools/simulation/gz/worlds/`,
launch Gazebo with the correct world name, point `ros_gz_bridge` at
`/world/maze_compete/...`, and select `airmouse_compete.yaml` for the
ROS-side nodes:

```bash
AIR_MOUSE_WORLD=maze_compete AIR_MOUSE_PROFILE=fast ./run_airmouse_sim.sh
```

The headless mission walk can be run against either world without Gazebo:

```bash
python3 tools/headless_mission_smoke.py --world maze_compete --profile fast
python3 tools/headless_mission_smoke.py --world maze_compete --profile accurate
```

## What must change for a real maze

The dimensions above are a simulation contract. For a physical arena, record
the measured wall thickness, height, gaps, floor flatness, lighting, RF
conditions, and marker locations in a versioned survey file. The planner must
use the measured vehicle envelope plus a configurable uncertainty margin, not
just the nominal SDF centerlines.
