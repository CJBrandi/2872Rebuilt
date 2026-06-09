# Whirlpool Action Flows

## Robot startup

1. `RobotContainer` builds mode-specific IO.
2. Vision camera 0 receives a timestamped turret-angle supplier from `RobotState`.
3. `Autos` is constructed with drive, intake, and superstructure.
4. Auto and characterization choosers are populated.
5. `configureButtonBindings()` installs the driver controls.

In sim, `FuelSim` is registered with the robot pose, velocity, intake bounding box, and `superstructure::addFuelSimIntaked`.

## Driver controls

Current `RobotContainer` bindings:

- default drive: `DriveCommands.joystickDriveWithSnakeMode`
- `B`: reset heading to zero while preserving translation
- POV up: stow intake, request manual turret/shooter mode, point turret to 180 degrees
- POV down: deploy intake and request SOTM
- left trigger: toggles intake roller on press and release
- `Y`: temporary roller ejection override while held
- `X`: hood homing command
- right trigger: sets `RobotState.autoEmpty` true while held

Keep driver bindings thin. If a button starts needing multi-step mechanism logic, move that logic into a subsystem method or command class.

## Shooting flow

1. Driver or sim sets `RobotState.autoEmpty`.
2. `Superstructure.periodic()` resolves active mode.
3. `ShotCalculator` computes shot parameters from pose, velocity, target selection, and lookup data.
4. `Shooter.setGoals()` receives exit velocity, pitch angle, and pitch velocity.
5. `Turret.setTargetFieldRelativeAngle()` receives yaw and yaw velocity.
6. `Shooter.periodic()` runs flywheel and hood control.
7. `Turret.periodic()` converts field-relative yaw to turret-relative yaw and runs the profile.
8. `Superstructure.isReadyToShoot()` gates the indexer.
9. `Indexer.runIntakeVelocity()` feeds only when the gate is true; otherwise `Indexer.stop()` is requested.

Sim shooting additionally calls `Superstructure.launchFuelSim()` from the sim default command when `autoEmpty` is active and the shot period has elapsed.

## Manual mode flow

`Manual/Enabled` overrides the requested turret shooter mode to `MANUAL`. In manual:

- `Shooter` uses `Manual/FlywheelRPM` and `Manual/PitchDeg`
- `Turret` uses `Manual/YawDeg`
- `Indexer` can use `Manual/IndexerRPM`
- `ShotCalculator` is configured with SOTM disabled and AUTO targeting, but superstructure does not apply shot calculator goals to shooter/turret in manual

Use manual mode for controlled mechanism tests. Avoid mixing manual targets with autonomous shot-target code in the same path.

## Intake and trench flow

`Intake` records requested pivot goal. `Superstructure` decides whether trench auto-deploy should override the commanded goal:

1. If the requested intake position does not require auto-deploy, the override is false.
2. If the intake is stowed and the projected rear intake tip enters a trench crossing envelope, auto-deploy can become active.
3. Once active, it remains active only while the trench envelope remains active.

The hood has separate trench protection: once homed, `Superstructure` may force the hood to its minimum angle near trench crossings.

## Vision and turret tracking flow

`Turret.periodic()` publishes current turret angle observations into `RobotState`. The tag camera config for camera 0 reads the buffered angle by timestamp. This means turret tracking changes can affect vision pose estimation, not just shooter aiming.

When changing turret sensor conventions, also check:

- `Turret.getFieldRelativeAngle()`
- `RobotState.addTurretObservation`
- `TagCameraConfig.withTurretAngleAtTimestamp`
- camera transforms in `VisionConstants`

## Autonomous flow

`Autos` receives drive, intake, and superstructure. `RobotContainer.getAutonomousCommand()` returns the selected chooser command. Auto behavior should use subsystem APIs and avoid duplicating driver-button internals.

## Characterization and homing

The characterization chooser exposes:

- drive wheel radius/feedforward/SysId routines
- elevator characterization and homing
- hood homing and static characterization
- turret static characterization
- indexer static characterization
- intake pivot static characterization

Hood closed-loop control depends on homing on real hardware. Run or verify homing before diagnosing hood aim misses.
