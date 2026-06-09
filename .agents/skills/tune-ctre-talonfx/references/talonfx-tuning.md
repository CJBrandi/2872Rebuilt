# CTRE TalonFX Tuning Reference

## Control Modes

| Mode | Use For | Primary Gains | Output Units |
|---|---|---|---|
| `VelocityVoltage` | Existing voltage flywheel/roller loops | `kS`, `kV`, `kP` | volts |
| `VelocityTorqueCurrentFOC` | Exploring torque-current velocity loops | `kS`, `kV`, `kP` | amps |
| `MotionMagicVoltage` | Existing arm/elevator/turret position loops | `kP`, `kD`, `kV`, `kA`, optional `kG` | volts |
| `MotionMagicTorqueCurrentFOC` | Exploring torque-current profiled position loops | `kP`, `kD`, `kV`, `kA`, optional `kG` | amps |

All Phoenix 6 gains are canonical: output per unit of mechanism error, velocity, or acceleration. Confirm whether sensor units are motor rotations or mechanism rotations before calculating starts.

## Velocity Tuning

Start with safe setpoints: low around 10 percent of expected max and high around 70-80 percent of expected max.

1. Set `kI` and `kD` to zero.
2. Set `kS` near the output that almost breaks friction.
3. Set small `kP` so bad feedforward is visible.
4. Tune `kV` at the high setpoint until actual velocity is near reference.
5. Tune `kS` at the low setpoint.
6. Repeat high/low until `kS` and `kV` stabilize.
7. Raise `kP` until settling improves, then back off if oscillation or current spikes appear.

For torque-current velocity, `kD` is usually not useful. Use `kP` for disturbance recovery.

## Motion Magic Tuning

Use conservative cruise velocity and acceleration. Never start with full mechanism travel.

1. Configure `MotionMagicCruiseVelocity`, `MotionMagicAcceleration`, and optional jerk.
2. Set feedforwards low and increase `kP` until the mechanism moves decisively.
3. Increase `kD` until overshoot is controlled.
4. Tune `kA` to reduce lag during acceleration/deceleration.
5. Tune `kV` when the profile shape is right but tracking is offset during cruise.
6. Revisit `kP`/`kD` after feedforward changes.

Torque-current position loops need damping more than voltage loops because current controls torque/acceleration.

## Current Limits

For torque-current control:

- Use `TorqueCurrent.PeakForwardTorqueCurrent` and `PeakReverseTorqueCurrent`.
- Also configure stator/supply current limits for Phoenix 2026 behavior, with the torque peaks treated as the direct torque cap.
- Abort torque-current tests if the TalonFX reports `UnlicensedFeatureInUse`.

For voltage control:

- Use `CurrentLimits.StatorCurrentLimit`, `StatorCurrentLimitEnable`, `SupplyCurrentLimit`, and `SupplyCurrentLimitEnable`.
- Use `Voltage.PeakForwardVoltage` and `PeakReverseVoltage` when limiting max output.

After tracking is acceptable, lower current caps until settling time, steady error, or brownout risk becomes unacceptable; keep margin above the last passing value.

## Telemetry Scoring

Accept gains only when the run is safe and repeatable:

- RMS error and steady-state error are inside tolerance.
- Settling time meets the mechanism goal.
- Overshoot stays inside safe travel or safe velocity margin.
- No sustained oscillation after settling.
- Peak and average stator/supply current are reasonable.
- Battery voltage remains healthy.
- No unlicensed, brownout, hardware limit, soft limit, or temperature faults appear.
