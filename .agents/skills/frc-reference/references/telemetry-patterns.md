# FRC Telemetry Patterns

## AdvantageKit-Style Field Names

Projects vary, so enumerate fields first. Common patterns:

- `/RealOutputs/<Subsystem>/<Field>`
- `/AdvantageKit/RealOutputs/<Subsystem>/<Field>`
- `/RobotState/<Field>`
- `/AdvantageKit/RobotState/<Field>`
- `/AdvantageKit/DriverStation/Enabled`
- `/AdvantageKit/DriverStation/Autonomous`
- `/AdvantageKit/DriverStation/Teleop`
- `/AdvantageKit/DriverStation/Test`
- `/AdvantageKit/DriverStation/Alliance`

## NetworkTables Keys

Common live/sim keys:

- `/SmartDashboard/...`
- `/Shuffleboard/...`
- `/LiveWindow/...`
- `/Sim/Enable`
- `/Sim/Autonomous`
- `/Sim/Test`
- chooser options: `<chooser>/options`
- chooser selected value: `<chooser>/selected`
- chooser robot-owned active value: `<chooser>/active`

Write chooser `selected`, not `active`.

## Limelight NT Keys

Common table names:

- `limelight`
- `limelight-front`
- `limelight-back`
- project-specific `limelight-*`

Common keys:

- `tv`: valid target flag
- `tx`: horizontal offset
- `ty`: vertical offset
- `ta`: target area
- `tl`: pipeline latency
- `cl`: capture latency
- `pipeline`: selected pipeline
- `botpose`: robot pose in field space
- `botpose_wpiblue`: robot pose in WPILib blue origin
- `botpose_wpired`: robot pose in WPILib red origin
- `botpose_orb_wpiblue`: MegaTag2-style blue-origin pose when available
- `botpose_orb_wpired`: MegaTag2-style red-origin pose when available

## Common Investigations

### Low Voltage

Fields to search:

- battery voltage
- brownout
- total current
- drivetrain current
- compressor current
- enabled/autonomous/teleop state

Analysis:

- Find intervals where voltage is below thresholds such as 12.0, 11.0, 10.5, and 9.5 V.
- Correlate dips with mechanism commands or drivetrain acceleration.
- Report duration, minimum voltage, and what else changed at the same time.

### Autonomous Path Accuracy

Fields to search:

- desired pose
- estimated pose
- odometry pose
- chassis speeds
- module states
- path name or auto routine
- enabled/autonomous state

Analysis:

- Find autonomous enable time.
- Compare desired vs estimated pose over the auto window.
- Report max translation error, max heading error, final error, and timestamp of worst error.

### Shooter or Mechanism PID

Fields to search:

- setpoint
- velocity/position measurement
- applied voltage/output
- current
- closed-loop reference
- at-setpoint flag

Analysis:

- Compute error over time.
- Measure time to settle and steady-state error.
- Check saturation: output pinned while error remains large.

### State Machine Timing

Fields to search:

- current state
- wanted/requested state
- command name
- sensors used for transition guards

Analysis:

- Build transition table with timestamp, from, to, and duration.
- Compare transition timing with sensor values.
- Flag impossible transitions, rapid oscillation, or states that never exit.
