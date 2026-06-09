# Robot Design Implications

This reference translates REBUILT from game description into robot-design and software concerns.

## Likely Robot Archetypes

- `cycle shooter`: focuses on collecting large quantities of `FUEL`, quickly indexing it, and scoring whenever the `HUB` is active
- `all-rounder`: balances scoring, traversal, and a reliable climb
- `defense-plus-climb`: sacrifices scoring ceiling for disruption, routing control, and strong endgame value
- `auto-weighted scorer`: emphasizes autonomous and fast early-match scoring windows

## Mechanism Pressure

- Intake: must gather loose balls cleanly from carpet and likely from dispersed field locations.
- Indexer or hopper: should tolerate variable ball flow and keep feeding stable under acceleration and obstacle crossing.
- Shooter: should prioritize repeatability and fast recovery more than pure peak speed.
- Climber: should be sequenced, sensor-aware, and able to transition from scoring mode without long dead time.

## Software Pressure

- Teleop drive code should support quick cycle routing and stable scoring alignment.
- Auto routines should optimize around early `FUEL` access, shot opportunities, and clean exits into teleop traffic.
- State machines should make scoring availability explicit because the hub is not always active.
- Endgame logic should support timed transition, approach alignment, climb sequence control, and safe interruption behavior.

## Useful Telemetry

- current match phase and time-to-endgame
- whether the alliance `HUB` is currently active according to the team’s game-state model
- `FUEL` count estimate or indexer occupancy
- shooter setpoint vs measured speed
- vision lock or shot-readiness state
- climb phase, latch state, or rung progress
- drivetrain pose, heading error, and approach-state indicators for `HUB` and `TOWER`

## Coding Heuristics

- Keep hardware ownership in subsystems and express game actions as small methods like `intakeFuel`, `stageFuel`, `shootWhenReady`, and `startClimbSequence`.
- Treat scoring and climb transitions as explicit command/state boundaries rather than ad hoc button logic.
- Build autos and driver aids around field landmarks and match-state changes, not just fixed timers.
- Prefer terminology that matches the game so future prompts like "tower auto-align" or "hub active window" map cleanly into code.

## Scope Note

This file is for design context. It does not replace the official rulebook, field drawings, or exact scoring tables.
