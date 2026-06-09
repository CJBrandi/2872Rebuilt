# Whirlpool Action Flows

## Robot startup

1. `RobotContainer` builds mode-specific IO.
2. Vision camera 0 receives a timestamped turret-angle supplier from `RobotState`.
3. `Autos` is constructed with drive, intake, and superstructure.
4. Auto and characterization choosers are populated.
5. `configureButtonBindings()` installs the one-controller crazy bindings.

In sim, `FuelSim` is registered with the robot pose, velocity, intake bounding box, and `superstructure::addFuelSimIntaked`.

## Driver controls

Current one-controller crazy bindings:

- default drive: `DriveCommands.joystickDrive`
- left trigger: deploy intake and run intake roller while held; release stops the roller
- neutral-zone trigger: requests ferry targeting while the robot is in the neutral zone
- right bumper: stow intake
- right trigger: sets `RobotState.autoEmpty` true while held
- `Y`: reset heading to zero while preserving translation
- `A`: aim-drive at the alliance hub while held
- `B`: temporary roller ejection override while held
- `Start+B`: reverse the indexer while held

Keep driver bindings thin. If a button starts needing multi-step mechanism logic, move that logic into a subsystem method or command class.

## Shooting flow

1. Driver or sim sets `RobotState.autoEmpty`.
2. `Superstructure.periodic()` resolves active mode.
3. `Superstructure` enables static or shoot-on-the-move compensation automatically from robot velocity.
4. `ShotCalculator` computes shot parameters from pose, velocity, target selection, and lookup data. If `RobotState.ferryShotRequested` is active in the neutral zone, target selection uses the closer ferry point.
5. `Shooter.setGoals()` receives exit velocity, pitch angle, and pitch velocity.
6. `Turret.setTargetFieldRelativeAngle()` receives yaw and yaw velocity.
7. `Shooter.periodic()` runs flywheel and hood control.
8. `Turret.periodic()` converts field-relative yaw to turret-relative yaw and runs the profile.
9. `Superstructure.isReadyToShoot()` gates the indexer.
10. `Indexer.runIntakeVelocity()` feeds only when the gate is true; otherwise `Indexer.stop()` is requested.

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
