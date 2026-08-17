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

## What must change for a real maze

The dimensions above are a simulation contract. For a physical arena, record
the measured wall thickness, height, gaps, floor flatness, lighting, RF
conditions, and marker locations in a versioned survey file. The planner must
use the measured vehicle envelope plus a configurable uncertainty margin, not
just the nominal SDF centerlines.
