# Whirlpool Physical Debugging

## Mechanism map

Whirlpool's scoring mechanism is represented by:

- foldable intake: `Intake`, `Pivot`, `Roller`
- feed path: `Indexer`
- single flywheel/rotor shooter: `Shooter` -> `Flywheel`
- motorized hood: `Shooter` -> `Hood`
- trackable turret: `Turret`

The code names the single rotor path `Flywheel`; keep that naming unless the source files are renamed.

## First checks

Before changing control logic, verify:

- runtime mode from `Constants.currentMode`
- CAN IDs and bus names in `Constants`
- IO implementation selected by `RobotContainer`
- homed state for hood on real hardware
- requested vs active turret shooter mode in `RobotState`
- AdvantageKit logs under `Shooter/*`, `Turret/*`, `Superstructure/*`, `Intake/*`, and `RobotState/*`

## Hood

The hood is limited to `15.5 deg` through `45.5 deg` in `Hood`. Targets outside that range are clamped and logged.

Important logs:

- `Shooter/Hood/TargetAngleDeg`
- `Shooter/Hood/MeasuredAngleDeg`
- `Shooter/Hood/AtGoal`
- `Shooter/Hood/Homed`
- `Shooter/Hood/SetpointClamped`

Real closed-loop hood control only runs after homing. If the hood does not move to a normal shot target, check homing before retuning PID.

## Flywheel

`Flywheel` accepts target exit velocity in meters per second and converts it to wheel velocity with the tunable efficiency coefficient.

Important logs:

- `Shooter/Flywheel/TargetExitVelocityMps`
- `Shooter/Flywheel/TargetWheelVelocityRPM`
- `Shooter/Flywheel/SetpointVelocityRPM`
- `Shooter/Flywheel/MeasuredVelocityRPM`
- `Shooter/Flywheel/Efficiency`

If shots are consistently short or long with stable aim, check lookup data and flywheel efficiency before changing turret or hood math.

## Turret

`Turret` has hard angle limits through `TurretLimits`, using constants currently ordered as 25 and -380 degrees. The helper sorts them into min/max.

Important logs:

- `Turret/TargetFieldRelativeAngleDeg`
- `Turret/TargetTurretAngleDeg`
- `Turret/FieldRelativeAngleDeg`
- `Turret/Profile/SetpointAngleDeg`
- `Turret/ActualAngleDeg`
- `Turret/Safety/*`

If turret aim wraps the wrong way, inspect the safety selection logs before changing angle math. The code intentionally chooses an equivalent angle near the current setpoint.

## Intake

The intake pivot convention is 0 degrees deployed and 90 degrees stowed. `Superstructure` can override the requested stowed goal to deploy for trench clearance.

Important logs:

- `Intake/RequestedPivotGoalDeg`
- `Intake/CommandedPivotGoalDeg`
- `Intake/TrenchAutoDeployEnabled`
- `Superstructure/TrenchIntakeDeployActive`
- `Superstructure/TrenchIntakeEnvelopeActive`

If the intake deploys unexpectedly, check trench auto-deploy logs before blaming the button binding.

## Indexer

`Indexer` owns jam detection and unjam pulses internally. Keep stall recovery in `Indexer`; callers should not duplicate reverse-pulse logic.

Important logs:

- `Superstructure/Indexer/TargetVelocityRPM`
- `Superstructure/Indexer/MeasuredVelocityRPM`
- `Superstructure/Indexer/Unjam/State`
- `Superstructure/Indexer/Unjam/Stalled`

If the indexer does not feed while shooting, inspect `Superstructure/ReadyToShoot`, shooter readiness, turret at-goal, and `RobotState/AutoEmpty`.

## Shot misses

For shot accuracy, follow the source path:

1. `RobotState.robotPose` and `robotVelocity`
2. `ShotCalculator` target selection and lookup load state
3. shoot-on-the-move compensation and stability gate
4. `Shooter` pitch-to-hood conversion and flywheel exit velocity
5. `Turret` field-relative conversion and limit selection
6. indexer feed gate timing

Useful logs:

- `ShotCalculator/VectorLookupLoaded`
- `ShotCalculator/ShotMode`
- `ShotCalculator/ShotStable`
- `ShotCalculator/PitchAngleDeg`
- `ShotCalculator/ExitVelocity`
- `ShotCalculator/RequiredFieldVelocity`
- `ShotCalculator/ShooterFieldVelocity`
- `Superstructure/ReadyToShoot`

Avoid tuning a symptom at the mechanism layer until the upstream pose, lookup, and compensation values are verified.

## Simulation

Sim mode registers a robot body, intake box, air resistance, starting fuel, and a timed launch loop in `RobotContainer`. Sim shooting uses `ShotCalculator` output directly in `Superstructure.launchFuelSim()`.

Do not describe the sim as full robot physics. It is a focused fuel, intake, and launch simulation tied to the robot pose and selected mechanisms.
