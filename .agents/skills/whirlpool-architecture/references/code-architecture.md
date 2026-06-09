# Whirlpool Code Architecture

## Composition root

`src/main/java/frc/robot/RobotContainer.java` constructs the robot by runtime mode:

- `REAL`: TalonFX/Pigeon/PhotonVision IO implementations.
- `SIM`: simulation IO plus `FuelSim`, simulated AprilTag/detection inputs, and sim inventory shooting.
- `REPLAY`: inert IO implementations for log replay.

Keep `RobotContainer` focused on wiring, chooser setup, and driver bindings. New mechanism behavior should usually live in the relevant subsystem, not directly in button lambdas.

## Shared state

`RobotState` is the shared robot state holder. It owns:

- `robotPose` and `robotVelocity`
- best fuel cluster state for pickup
- `autoEmpty`, used as the shoot/feed request
- requested and active `TurretShooterMode`
- a timestamped turret-angle buffer for vision pose estimation

The turret publishes observations every periodic cycle, including while disabled. Vision camera config reads `RobotState.getTurretAngleAtTime(timestampSeconds)` so turret-mounted camera compensation can use the angle at image capture time.

## Superstructure ownership

`Superstructure` owns the high-level interaction between:

- `Shooter`: flywheel plus motorized hood
- `Turret`: yaw control and safety limits
- `Indexer`: feed path and auto-unjam behavior
- `Intake`: foldable intake coordination for trench auto-deploy

`Superstructure.periodic()` is the normal place for shot-mode resolution and coordinated behavior. It:

- resolves requested mode to active mode, with `Manual/Enabled` overriding to `MANUAL`
- configures `ShotCalculator` for SOTM, aim, or manual
- auto-deploys the intake near trench crossings when needed
- forces the hood to minimum near trench crossings once the hood is homed
- sends shot calculator goals to shooter and turret when not manual
- feeds the indexer only when `isReadyToShoot()` is true

Readiness requires `RobotState.autoEmpty`, no trench hood stow, shooter ready, and turret at goal.

## Shooter and hood

`Shooter` owns `Flywheel` and `Hood`; neither is a WPILib subsystem. The shooter converts shot calculator outputs into component goals:

- flywheel target is an exit velocity in meters per second
- hood target is derived from shot pitch using `pitchToHoodAngleRad`
- hood velocity is inverted with `pitchToHoodVelocityRadPerSec`

Manual shooter mode uses shared tunables:

- `Manual/Enabled`
- `Manual/FlywheelRPM`
- `Manual/PitchDeg`

`Hood` has its own profile, clamps, homing sequence, static characterization, and homed gate. Real closed-loop hood motion does not run unless homed; sim is treated as homed through `Shooter.isHoodHomed()`.

## Turret

`Turret` owns yaw control, trapezoid profiling, target conversion, safety limits, static characterization, and visualization. It supports two goal styles:

- field-relative target via `setTargetFieldRelativeAngle`
- direct robot-relative target via `setTargetTurretAngle`

Field-relative targeting subtracts robot heading and robot angular velocity from the target before profiling. `TurretLimits` selects an equivalent angle near the current setpoint and clamps to configured hard limits.

Manual turret mode uses:

- `Manual/Enabled`
- `Manual/YawDeg`

Do not add a turret command path that bypasses `TurretLimits` or the existing profile synchronization on closed-loop entry.

## Intake

`Intake` owns `Pivot` and `Roller`. The pivot convention is:

- 0 degrees is ground/deployed
- 90 degrees is stowed
- `groundAngle` is -8 degrees
- `stowedAngle` is 90 degrees

`Intake` stores the requested pivot goal, while `Superstructure` may override the commanded goal with trench auto-deploy. The roller is controlled by `deploy`, `stow`, `toggleIntake`, and direct override paths.

## Indexer

`Indexer` owns feed velocity control, manual RPM mode, trapezoid-profiled velocity setpoints, and an internal auto-unjam state machine. `Superstructure` should request `runIntakeVelocity()` or `stop()`; jam recovery stays inside `Indexer`.

## Shot calculation

`ShotCalculator` is a singleton utility that loads `src/main/deploy/hub_lookup_vector.json`. It computes:

- target selection
- shoot-on-the-move compensation
- pitch, exit velocity, turret yaw, and feedforward angular velocities
- stability gating telemetry

`ShotVectorCompensator` contains the field/shooter-frame vector math. Keep coordinate-frame changes localized there or in `ShotCalculator`.

## Constants and IO

Hardware IDs and reductions are in `Constants`:

- intake pivot: CAN 49, roller: CAN 50
- turret: CAN 60, range configured as 25 to -380 degrees
- flywheel leader/follower: CAN 51 and 53
- hood: CAN 52
- indexer: CAN 54, vortex ID 55

Mechanism IO follows the repo pattern: `*IO`, `*IOTalonFX`, and `*IOSim`.
