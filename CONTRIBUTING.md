# Contributing

## Before opening a change

- Keep simulation assumptions, measured hardware facts, and competition rules
  clearly labeled.
- Do not add personal absolute paths, credentials, private documents, or
  generated build/log folders.
- Update the relevant maze, interface, safety, and setup documentation when a
  contract changes.
- Add or update a test for coordinate conversions, grid logic, route clearance,
  or mission-state behavior when applicable.

## Pull requests

Describe:

1. What changed and why.
2. Whether the change affects simulation, physical hardware, or both.
3. Commands used to verify it.
4. Any new assumptions, rulebook dependencies, calibration data, or safety
   risks.

Never describe a SITL result as proof of physical flight readiness. Physical
flight changes require review against [`docs/Safety-and-Test-Plan.md`](docs/Safety-and-Test-Plan.md).
