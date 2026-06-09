---
name: frc-reference
description: Use when working on FRC robot code, WPILib Java APIs, Javadocs, REV SPARK, CTRE Phoenix 6, Limelight, vendordeps, command-based programming, swerve, PID, .wpilog logs, NetworkTables, robot simulation, autonomous behavior, or telemetry debugging.
---

# FRC-Reference

Use this as the project skill for FRC docs, Java API lookup, and telemetry-driven debugging. Java is the primary target. Prefer official online docs and Javadocs for API details, and prefer measured telemetry over guesses for robot behavior.

## Documentation Workflow

1. Inspect the robot project before answering implementation questions:
   - `build.gradle`, `settings.gradle`, `gradle.properties`
   - `vendordeps/*.json`
   - imports under `src/main/java`
   - constants and subsystem files related to the question
2. Identify the active year and libraries:
   - WPILib year from GradleRIO or WPILib version
   - REV from `REVLib` vendordep or `com.revrobotics` imports
   - CTRE from `Phoenix6` vendordep or `com.ctre.phoenix6` imports
   - Limelight from `LimelightHelpers`, NetworkTables table names, or vision subsystem code
3. Search official online docs before giving API-specific guidance.
4. For Java API questions, search Javadocs first, then prose docs/examples.
5. Cite exact URLs for claims about current APIs, constructors, deprecations, units, or examples.
6. If docs are missing or ambiguous, say what was verified and what is inferred from project code or general FRC knowledge.

Read `references/vendors.md` when you need vendor URLs, Javadoc roots, package names, or search templates.

## Javadoc Rules

- Treat Javadocs as authoritative for method signatures, overloads, constructors, deprecations, return types, and package names.
- Prefer current/release Javadocs unless the project pins an older vendordep.
- Search by class and package first: `SparkMax`, `SparkMaxConfig`, `TalonFX`, `CANcoder`, `CommandScheduler`, `SwerveDrivePoseEstimator`, `NetworkTableInstance`, `LimelightHelpers`.
- Do not invent method names. If a method cannot be found in Javadocs, search again by package/class and then say it was not found.

Common Javadoc entry points:

- WPILib: `https://github.wpilib.org/allwpilib/docs/release/java/index.html`
- REVLib: `https://codedocs.revrobotics.com/java/index.html`
- CTRE Phoenix 6 latest: `https://api.ctr-electronics.com/phoenix6/latest/java/`
- CTRE Phoenix 6 stable/release: `https://api.ctr-electronics.com/phoenix6/stable/java/` or `https://api.ctr-electronics.com/phoenix6/release/java/`

## Vendor Scope

Include:

- WPILib and command-based Java
- REV SPARK MAX, SPARK Flex, NEO, NEO 550, encoders, REVLib simulation
- CTRE Phoenix 6 TalonFX, TalonFXS, CANcoder, Pigeon2, CANrange, CANdi, CANdle
- Limelight docs, LimelightHelpers, and Limelight NetworkTables keys

For other vendor or vision stacks, only look them up when the user explicitly asks or the project code already depends on them.

## Telemetry Workflow

Use telemetry as evidence. Report findings with field names, timestamps, values, and the exact query or code path used.

1. Identify the telemetry source:
   - `.wpilog` file
   - live NetworkTables robot connection
   - WPILib simulation NetworkTables connection
   - project logging code such as AdvantageKit
2. Enumerate fields before analysis.
3. Select a time window or robot mode from driver station, enabled, autonomous, teleop, or test fields.
4. Query only the fields needed for the goal.
5. Compute transitions, statistics, thresholds, or tracking error.
6. Report concrete observations, likely causes, and next steps.
7. Disconnect sessions, disable sim/robot test control, and clean up temporary helpers.

Read `references/telemetry-patterns.md` for field patterns, time-window analysis, Limelight keys, and common investigations.

## NetworkTables and Simulation

- Confirm target host before connecting: robot IP, `roborio-TEAM-frc.local`, or `127.0.0.1` for sim.
- List tables/keys first; do not assume exact key names.
- Reads are allowed whenever useful.
- Writes are allowed for the requested task, including live/sim control, but log each key/value written and restore/disable when done.
- Avoid writing robot-owned status keys such as chooser `active`; write chooser `selected` when changing selection.

For headless sim control, add or verify an NT-controlled simulation hook in `Robot.simulationPeriodic()` when the project needs external enable/mode control:

```java
var nt = NetworkTableInstance.getDefault();
DriverStationSim.setEnabled(nt.getEntry("/Sim/Enable").getBoolean(false));
DriverStationSim.setAutonomous(nt.getEntry("/Sim/Autonomous").getBoolean(false));
DriverStationSim.setTest(nt.getEntry("/Sim/Test").getBoolean(false));
DriverStationSim.notifyNewData();
```

Required imports:

```java
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
```

Use the project's GradleRIO task, usually `./gradlew simulateJava` on macOS/Linux or `.\gradlew.bat simulateJava` on Windows. Connect to `127.0.0.1`, enumerate NT fields, set `/Sim/Autonomous`, `/Sim/Test`, and `/Sim/Enable` as needed, then set `/Sim/Enable=false` when done.

## Answer Shape

For implementation answers:

- State the verified library/year when known.
- Link the Javadoc or official doc page used.
- Provide minimal Java code that matches the project style.
- Include imports when adding new classes.
- Explain units and coordinate frames for motion, pose, vision, and odometry APIs.
- Mention any vendordep/API mismatch before suggesting code.

For telemetry reports:

- Source and time window analyzed.
- Fields inspected.
- Key numerical results or transitions.
- Anomalies with timestamps and values.
- Any writes performed.
- Recommended code/config changes and a verification step.

If a parser, docs page, or NT client is unavailable, state what is missing and propose the smallest local helper or tool needed. Do not pretend telemetry or documentation was inspected.
