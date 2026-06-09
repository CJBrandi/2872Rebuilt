# Minimum Robot Harness

This is the minimum code a robot repo needs for GymSim.

## 1. Java Harness File

GymSim installs this file:

```text
src/main/java/frc/robot/gymsim/GymSimRuntime.java
```

The harness only activates when `GYMSIM_RUN_DIR` is set. In normal robot runs it returns `null` and does nothing.

At runtime it:

- polls `.gymsim/runs/<run-id>/requests/*.json`
- applies Driver Station state through `DriverStationSim`
- applies simple joystick state through `JoystickSim`
- schedules and cancels real WPILib commands with `CommandScheduler`
- resolves scenario expressions against the live `RobotContainer`
- writes `.gymsim/runs/<run-id>/events.jsonl`
- records `GymSim/...` AdvantageKit outputs

## 2. Direct `RobotContainer` Expressions

GymSim does not require a command-name map in `RobotContainer`. Scenario command values are Java-ish expressions evaluated against the live `RobotContainer` object:

```yaml
steps:
  - invoke: { expression: "turret1.setTargetingMode(VISION)" }
  - schedule: { command: "getAutonomousCommand()" }
  - wait_command: { command: "getAutonomousCommand()", timeout_s: 5 }
assertions:
  - command_finished: "getAutonomousCommand()"
```

The expression root is `RobotContainer`. GymSim uses reflection with private access enabled, so private subsystem fields and helper methods can be referenced in desktop simulation without adding test-only getters.

Supported expression features:

- field and method chains such as `turret1.setTargetingMode(VISION)`
- no-arg and argument method calls such as `getAutonomousCommand()`
- string, boolean, int, double, and enum arguments

Unsupported expression features include variables, assignments, constructors, static calls, lambdas, operators, and multi-statement scripts.

## 3. `Robot.java` Runtime Hook

Add the import and field:

```java
import frc.robot.gymsim.GymSimRuntime;

private GymSimRuntime gymSimRuntime;
```

Start the runtime after `RobotContainer` is created:

```java
robotContainer = new RobotContainer();
gymSimRuntime = GymSimRuntime.start(robotContainer);
```

Poll it from `robotPeriodic()`:

```java
if (gymSimRuntime != null) {
  gymSimRuntime.periodic();
}
```

## 4. AdvantageKit SIM Log Writer

In the SIM logging setup, add a WPILOG writer when `GYMSIM_LOG_DIR` is set:

```java
case SIM:
  String gymSimLogDir = System.getenv("GYMSIM_LOG_DIR");
  if (gymSimLogDir != null && !gymSimLogDir.isBlank()) {
    Logger.addDataReceiver(new WPILOGWriter(gymSimLogDir));
  }
  Logger.addDataReceiver(new NT4Publisher());
  break;
```

This is what lets `gymsim logs` and YAML log assertions inspect the simulated robot as a real `.wpilog`.

## 5. No Build Server Required

GymSim V1 does not require a WebSocket server or Python service. The Bash CLI starts GradleRIO simulation with:

```bash
JAVA_HOME=/Users/cjbrandi/wpilib/2026/jdk \
GYMSIM_RUN_DIR=.gymsim/runs/<run-id> \
GYMSIM_LOG_DIR=.gymsim/runs/<run-id>/logs \
./gradlew simulateJavaRelease
```

All test control is sent through JSON request files that the Java harness reads inside the simulation process.
