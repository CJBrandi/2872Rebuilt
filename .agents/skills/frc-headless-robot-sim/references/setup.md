# GymSim Setup

GymSim is a terminal-only Bash CLI for headless Java WPILib simulation. It does not run a Python backend or server.

## Install the CLI

Add the GymSim `bin` directory to your shell path:

```bash
export PATH="/Users/cjbrandi/GymSim/bin:$PATH"
```

Or symlink it into a directory that is already on `PATH`:

```bash
mkdir -p "$HOME/.local/bin"
ln -sf /Users/cjbrandi/GymSim/bin/gymsim "$HOME/.local/bin/gymsim"
```

Verify:

```bash
gymsim --help
```

## Prepare a Robot Repo

From anywhere inside a Java Command-based AdvantageKit robot project:

```bash
gymsim init
```

`init` finds the robot root by walking upward until it sees:

- `gradlew`
- `build.gradle`
- `src/main/java/frc/robot/Robot.java`

It then installs `src/main/java/frc/robot/gymsim/GymSimRuntime.java` and patches the minimal hooks documented in [robot-harness.md](/Users/cjbrandi/GymSim/docs/robot-harness.md).

## Run a Scenario

Create a scenario file:

```yaml
name: smoke
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
```

Command expressions are resolved against the live `RobotContainer`, so scenarios can call existing robot-code methods and subsystem fields without adding a command-name map to the robot project. `invoke` is for setup calls that do not return a WPILib `Command`.

Run it:

```bash
gymsim run .gymsim/tests/smoke.yml
```

The command stays quiet while the test is running. After the test concludes, GymSim stops the simulation and prints the final result:

```text
Result: PASS
Scenario: .gymsim/tests/smoke.yml
Run: 20260531-191033
Log: /path/to/.gymsim/runs/20260531-191033/logs/akit_....wpilog
```

If the scenario exceeds `timeout_s` or a `wait_command` timeout, the result is `TIMEOUT` and the command exits with status `124`. Failed assertions use `Result: FAIL`.

GymSim creates `.gymsim/runs/<run-id>/` with:

- `requests/*.json`: commands sent to the robot harness
- `events.jsonl`: harness events and command lifecycle records
- `logs/*.wpilog`: AdvantageKit simulation logs
- `sim.stdout.log` and `sim.stderr.log`: Gradle simulation output

Inspect the result:

```bash
gymsim logs <run-id>
```

## 2872Rebuilt Project-Local Codex Skill

This copy is installed project-locally in 2872Rebuilt:

```text
/Users/cjbrandi/2872Rebuilt/.agents/skills/frc-headless-robot-sim
```

Do not run `gymsim skill install` for 2872Rebuilt unless you intentionally want a separate global copy.
