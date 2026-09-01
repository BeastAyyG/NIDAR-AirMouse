#!/usr/bin/env python3
"""Check maze geometry and the mission route for every AirMouse world.

This is a static geometry check, not a physics or planner proof. For each
world it verifies:

1.  the wall and survivor marker names match the expected set,
2.  every wall has a collision box, and every marker is visual-only,
3.  every waypoint is inside the configured geofence,
4.  every straight route leg clears the walls after they are inflated by the
    vehicle envelope radius,
5.  every survivor marker has at least its own radius of clearance to the
    nearest wall face,
6.  a flood fill over the inflated free space proves the home position, every
    route leg endpoint, and (for the competition arena) every survivor marker
    are in one connected component.

The route is read from the committed ROS 2 parameter file rather than being
duplicated here, so the geometry and the mission config cannot drift apart.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from airmouse_worlds import (  # noqa: E402
    CONNECTIVITY_RESOLUTION,
    REPO_ROOT,
    SURVIVOR_MARKER_RADIUS,
    WORLD_SPECS,
    FreeSpace,
    inflate,
    load_mission_profile,
    load_survivors_gz,
    load_walls,
    model_has_collision,
    route_gz,
    segment_intersects_rect,
    segment_rect_distance,
)


def validate(name: str, envelope_radius: float) -> int:
    spec = WORLD_SPECS[name]
    print(f"=== world: {spec.name} "
          f"{'(competition arena)' if spec.is_competition else '(practice course)'} ===")
    print(f"  world sdf        : {spec.sdf.relative_to(REPO_ROOT)}")
    print(f"  mission config   : {spec.config.relative_to(REPO_ROOT)}")
    print(f"  envelope radius  : {envelope_radius:.2f} m")

    profile = load_mission_profile(spec.config)
    walls = load_walls(spec.sdf, spec.name)
    survivors_gz = load_survivors_gz(spec.sdf)
    route = route_gz(profile)

    wall_names = {wall_name for wall_name, _ in walls}
    survivor_names = set(survivors_gz)
    failures: list[str] = []

    # 1. Name sets.
    if wall_names != spec.expected_walls:
        failures.append(
            f"wall set mismatch: expected {sorted(spec.expected_walls)}, got {sorted(wall_names)}"
        )
    if survivor_names != spec.expected_survivors:
        failures.append(
            f"survivor set mismatch: expected {sorted(spec.expected_survivors)}, "
            f"got {sorted(survivor_names)}"
        )

    # 2. Collision semantics: walls must collide, markers must not.
    walls_without_collision = sorted(
        wall_name for wall_name, has in model_has_collision(spec.sdf, "wall_").items() if not has
    )
    if walls_without_collision:
        failures.append(f"walls without a collision box: {walls_without_collision}")
    markers_with_collision = sorted(
        marker for marker, has in model_has_collision(spec.sdf, "survivor_").items() if has
    )
    if markers_with_collision:
        failures.append(f"survivor markers must be visual-only but have collision: {markers_with_collision}")

    # 3. Geofence containment.
    fence = profile.geofence
    for x, y, _z in [(0.0, 0.0, 0.0), *profile.waypoints_px4]:
        if not (fence["geofence_x_min"] <= x <= fence["geofence_x_max"]
                and fence["geofence_y_min"] <= y <= fence["geofence_y_max"]):
            failures.append(
                f"waypoint ({x:+.2f}, {y:+.2f}) is outside the geofence "
                f"x[{fence['geofence_x_min']:+.1f},{fence['geofence_x_max']:+.1f}] "
                f"y[{fence['geofence_y_min']:+.1f},{fence['geofence_y_max']:+.1f}]"
            )

    # 4. Route legs versus the inflated walls.
    minimum_gap = math.inf
    nearest_wall = ""
    for start, end in zip(route, route[1:]):
        for wall_name, rect in walls:
            gap = segment_rect_distance(start, end, rect)
            if gap < minimum_gap:
                minimum_gap, nearest_wall = gap, wall_name
            if segment_intersects_rect(start, end, inflate(rect, envelope_radius)):
                failures.append(
                    f"leg ({start[0]:+.2f},{start[1]:+.2f})->({end[0]:+.2f},{end[1]:+.2f}) "
                    f"intersects {wall_name} after {envelope_radius:.2f} m inflation"
                )

    # 5. Marker clearance.
    marker_clearance = {}
    for marker, (gx, gy, _gz) in sorted(survivors_gz.items()):
        clearance = min(segment_rect_distance((gx, gy), (gx, gy), rect) for _, rect in walls)
        marker_clearance[marker] = clearance
        if clearance < SURVIVOR_MARKER_RADIUS:
            failures.append(
                f"{marker} clearance {clearance:.2f} m is below its {SURVIVOR_MARKER_RADIUS:.2f} m radius"
            )

    # 6. Reachability through the inflated free space.
    space = FreeSpace(walls, envelope_radius)
    home_x, home_y = route[0]
    reachable = space.component_of(home_x, home_y)
    if not reachable:
        failures.append(f"home position ({home_x:.2f}, {home_y:.2f}) is not in free space")
        reachable_route = 0
    else:
        reachable_route = sum(
            1 for point in route if space.is_free(*point) and space.index_of(*point) in reachable
        )
        for point in route:
            if not space.is_free(*point):
                failures.append(f"route point ({point[0]:+.2f},{point[1]:+.2f}) is not in free space")
            elif space.index_of(*point) not in reachable:
                failures.append(
                    f"route point ({point[0]:+.2f},{point[1]:+.2f}) is unreachable from home"
                )

    reachable_survivors = []
    unreachable_survivors = []
    for marker, (gx, gy, _gz) in sorted(survivors_gz.items()):
        if reachable and space.is_free(gx, gy) and space.index_of(gx, gy) in reachable:
            reachable_survivors.append(marker)
        else:
            unreachable_survivors.append(marker)
    if unreachable_survivors and spec.require_reachable_survivors:
        failures.append(
            f"survivor markers unreachable from home: {unreachable_survivors}"
        )

    reachable_walls = len(walls)
    print(f"  walls / markers  : {reachable_walls} / {len(survivors_gz)}")
    print(f"  route points     : {len(route)} ENU points")
    print(f"  free cells       : {len(reachable)} at {CONNECTIVITY_RESOLUTION:.2f} m resolution")
    print(f"  min static gap   : {minimum_gap:.2f} m near {nearest_wall}")
    print(f"  post-envelope gap: {minimum_gap - envelope_radius:.2f} m")
    print(f"  marker clearance : min={min(marker_clearance.values()):.2f} m "
          f"max={max(marker_clearance.values()):.2f} m")
    print(f"  reachable route  : {reachable_route}/{len(route)}")
    print(f"  reachable markers: {len(reachable_survivors)}/{len(survivors_gz)}"
          + (f" (unreachable: {unreachable_survivors})" if unreachable_survivors else ""))

    expected_validated = sorted(spec.expected_validated)
    print(f"  expected validated in the mission walk: {expected_validated}")

    if failures:
        print(f"  RESULT           : FAIL")
        for failure in failures:
            print(f"    - {failure}")
        return 1

    print(f"  RESULT           : PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate the committed AirMouse maze geometry and mission route."
    )
    parser.add_argument(
        "--world",
        choices=[*sorted(WORLD_SPECS), "all"],
        default="all",
        help="which world to validate (default: all)",
    )
    parser.add_argument("--envelope-radius", type=float, default=0.32)
    args = parser.parse_args()

    names = sorted(WORLD_SPECS) if args.world == "all" else [args.world]
    failures = 0
    for name in names:
        try:
            failures += validate(name, args.envelope_radius)
        except (OSError, ValueError, KeyError, TypeError) as error:
            print(f"  RESULT           : FAIL: {type(error).__name__}: {error}")
            failures += 1
        print()

    if failures:
        print(f"{failures} world check(s) FAILED", file=sys.stderr)
    else:
        print(f"all {len(names)} world check(s) PASSED")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
