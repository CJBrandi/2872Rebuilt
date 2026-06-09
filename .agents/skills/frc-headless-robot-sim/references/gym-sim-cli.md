# GymSim

GymSim is a terminal-only Bash CLI for headless Java WPILib + AdvantageKit robot simulation.

It installs a small robot-side Java harness, starts GradleRIO simulation, sends YAML scenario steps as file requests, records `.wpilog` output, and summarizes/asserts the result from the terminal.

## Quick Start

```bash
export PATH="/Users/cjbrandi/GymSim/bin:$PATH"
gymsim --help
```

Inside a robot project:

```bash
gymsim init
gymsim run .gymsim/tests/smoke.yml
gymsim logs <run-id>
```

Setup details:

- [CLI setup](/Users/cjbrandi/GymSim/docs/setup.md)
- [Minimum robot harness](/Users/cjbrandi/GymSim/docs/robot-harness.md)

## Commands

- `gymsim init [path]`: detect the robot root, install the Java harness, and patch `Robot.java`.
- `gymsim start [path]`: launch `./gradlew simulateJavaRelease` with `GYMSIM_RUN_DIR` and `GYMSIM_LOG_DIR`.
- `gymsim test <file.yml> [path]`: send a scenario to the running sim and evaluate assertions.
- `gymsim run <file.yml> [path]`: start, test, collect logs, and stop.
- `gymsim logs [run-id] [path]`: print compact `.wpilog` and event summaries.
- `gymsim stop [path]`: disable, request shutdown, and stop the sim process.
- `gymsim skill install`: install the bundled Codex skill globally; do not use this for 2872Rebuilt's project-local copy.

`gymsim run` and `gymsim test` do not stream progress output. They print once after the scenario finishes, fails, or times out:

```text
Result: PASS
Scenario: .gymsim/tests/smoke.yml
Run: 20260531-191033
Log: /path/to/.gymsim/runs/20260531-191033/logs/akit_....wpilog
```

Timeouts are reported as `Result: TIMEOUT` and exit with status `124`. The simulation is disabled and stopped before the final result is printed.

## Scenario Format

GymSim supports a small YAML subset that is easy to parse from Bash: top-level keys, simple lists, and one-line inline maps.

```yaml
name: whirlpool-smoke
timeout_s: 8
robot:
  alliance: blue1
steps:
  - disable
  - enable: autonomous
  - invoke: { expression: "turret1.setTargetingMode(VISION)" }
  - schedule: { command: "getAutonomousCommand()" }
  - wait_command: { command: "getAutonomousCommand()", timeout_s: 5 }
  - log_marker: done
assertions:
  - command_finished: "getAutonomousCommand()"
  - log_exists: GymSim/Active
  - no_driverstation_errors
```

`schedule.command` and `wait_command.command` are Java-ish expressions resolved against the live `RobotContainer`. `invoke.expression` runs a setup expression without scheduling a command. GymSim supports field/method chains plus primitive, string, boolean, double, int, and enum arguments; it is not a general Java scripting engine.
