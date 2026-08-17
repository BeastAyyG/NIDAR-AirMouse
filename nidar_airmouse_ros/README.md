# NIDAR AirMouse simulation package

This package implements the simulation-side baseline from the engineering
book while preserving PX4 as the flight stack requested for this setup.

It provides:

- an auditable mission-state skeleton and `/airmouse/start`, `/airmouse/abort`,
  and `/airmouse/complete` services;
- persistent, de-duplicated map-point survivor tags projected into the explicit
  one-metre grid convention;
- a compact GCS/PX4 status topic;
- an offline `sim-smoke` grid acceptance test.

The nodes do not arm a real aircraft or claim that a camera detector is
implemented. They are the safe interfaces around which the real detector,
frontier planner, SLAM, and PX4 command adapter can be added.
