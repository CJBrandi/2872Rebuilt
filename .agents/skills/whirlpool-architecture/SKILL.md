---
name: whirlpool-architecture
description: Use when working in the 2872Rebuilt Whirlpool robot codebase on architecture, subsystem ownership, command flow, intake, single-rotor shooter, turret, motorized hood, superstructure behavior, shot calculation, vision, simulation, or physical debugging.
---

# Whirlpool Architecture

Use this skill for robot-specific code navigation and architecture decisions in `/Users/cjbrandi/2872Rebuilt`.

Whirlpool is modeled in code as:

- a swerve drive with pose estimation and vision fusion
- a foldable intake made of `Intake`, `Pivot`, and `Roller`
- a superstructure that owns `Shooter`, `Turret`, `Indexer`, and intake coordination
- a shooter with one flywheel/rotor path plus a motorized hood
- a trackable turret whose angle is buffered in `RobotState` for timestamped vision

## Start Here

1. Read `references/code-architecture.md` before changing ownership, command flow, or subsystem boundaries.
2. Read `references/action-flows.md` before changing driver buttons, auto behavior, shooting, intake, or simulation flow.
3. Read `references/physical-debugging.md` before diagnosing mechanism behavior, tuning, or hardware/sim mismatches.

## Ground Rules

- Treat `RobotContainer` as the composition and binding layer; keep mechanism behavior in subsystem classes.
- Treat `Superstructure` as the coordinator for shooter, hood, turret, indexer, and intake safety interactions.
- Keep `RobotState` as the shared state/cache for pose, velocity, fuel cluster selection, shot mode, and turret-angle history.
- Do not bypass `TurretLimits`, hood angle clamps, or homing guards when adding control paths.
- Prefer existing `LoggedTunableNumber` keys and AdvantageKit logs for tuning and debugging.
- Verify Java changes with the repo Gradle wrapper; use Java 17 when WPILib tooling requires it.
