---
name: frc-headless-robot-sim
description: Use when working inside a Java AdvantageKit WPILib robot project and headless GymSim robot simulation tests, YAML scenarios, direct RobotContainer expressions, or WPILOG evidence are needed.
---

# FRC Headless Robot Sim

Use GymSim when a robot-code change should be validated in desktop simulation with evidence from a `.wpilog`. This is a 2872Rebuilt project-local skill installed under `.agents/skills/`; do not reinstall it globally.

## Workflow

1. From anywhere inside the robot project, run `gymsim init` once.
2. Write a YAML scenario under `.gymsim/tests/` using direct expressions against `RobotContainer`.
3. Run `gymsim run .gymsim/tests/<name>.yml`.
4. Wait for the final result block; GymSim prints only after pass, fail, or timeout.
5. Use the reported `Log:` path plus `gymsim logs <run-id>` and `.gymsim/runs/<run-id>/events.jsonl`.
6. Change robot code from the log evidence, then rerun the scenario.

## References

- `references/setup.md`: CLI setup, `gymsim init`, run output, run directory contents, and log inspection.
- `references/robot-harness.md`: robot-side Java hooks, direct expression support, and AdvantageKit SIM log writer.
- `references/gym-sim-cli.md`: command summary and scenario format.
- `examples/whirlpool-smoke.yml`: starter Whirlpool smoke scenario.

## Direct Expressions

GymSim resolves `schedule.command`, `cancel.command`, `wait_command.command`, and `invoke.expression` against the live `RobotContainer` object. Private fields and methods are accessible in simulation, so use existing subsystem objects and helper methods directly instead of adding a command-name map.

The expression language is intentionally small: field/method chains, no-arg calls, and string, boolean, int, double, and enum arguments. Do not use variables, assignments, constructors, static calls, lambdas, operators, or multi-statement scripts.

## YAML Shape

GymSim intentionally supports a small terminal-friendly YAML subset: top-level keys, simple lists, and one-line inline maps.

```yaml
name: intake-smoke
timeout_s: 8
robot:
  alliance: blue1
steps:
  - disable
  - enable: teleop
  - invoke: { expression: "turret1.setTargetingMode(VISION)" }
  - schedule: { command: "intake.extendCommand()" }
  - wait_command: { command: "intake.extendCommand()", timeout_s: 3 }
assertions:
  - command_finished: "intake.extendCommand()"
  - log_exists: GymSim/Active
```

Use these step types: `disable`, `enable`, `invoke`, `schedule`, `cancel`, `wait_seconds`, `wait_command`, `joystick`, `log_marker`.

Use these assertion types: `log_exists`, `log_final`, `log_min`, `log_max`, `command_finished`, `no_driverstation_errors`.

## Rules

- GymSim is a Bash terminal CLI, not a Python service.
- The robot-side Java harness controls Driver Station and joystick sim state from request files.
- `gymsim run` and `gymsim test` stop the headless sim before printing their final result block.
- `Result: TIMEOUT` means the scenario exceeded `timeout_s` or a command wait timed out; the command exits with status `124`.
- Treat the `.wpilog` and `events.jsonl` as the source of truth.
- Do not claim sim behavior is fixed until `gymsim run` and the relevant log assertions pass.
