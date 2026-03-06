// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Pivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Pivot/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Pivot/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/Pivot/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Intake/Pivot/MaxVelocityDegreesPerSec", 120);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Intake/Pivot/MaxAccelerationDegreesPerSec2", 360);
  private static final LoggedTunableNumber staticVelocityThresh =
      new LoggedTunableNumber("Intake/Pivot/staticVelocityThresh", 0.1);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Intake/Pivot/Tolerance", 45);
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled", 0.0);
  private static final LoggedTunableNumber manualPivotAngleDeg =
      new LoggedTunableNumber("Manual/IntakePivotDeg", 90.0);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Intake/Pivot/HomingVolts", 2.0);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Intake/Pivot/HomingVelocityThreshRadPerSec", 0.3);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Intake/Pivot/HomingTimeSecs", 0.1);

  static {
    switch (Constants.getCurrentMode()) {
      case SIM -> {
        kP.initDefault(7000);
        kD.initDefault(2000);
        kS.initDefault(1.2);
        kG.initDefault(0.0);
      }
      default -> {
        kP.initDefault(3300);
        kD.initDefault(50);
        kS.initDefault(3);
        kG.initDefault(0);
      }
    }
  }

  // Hardware
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  // Control
  @Getter
  @AutoLogOutput(key = "Intake/Pivot/MeasuredAngle")
  private Rotation2d angle = new Rotation2d();

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> Intake.stowedAngle.getRadians();
  private boolean profileInitialized = false;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @Getter
  @AutoLogOutput(key = "Intake/Pivot/AtGoal")
  private boolean atGoal = false;

  // Homed state is explicit and can be controlled directly.
  @Getter @Setter private boolean homed = false;
  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  // Disconnected alerts
  private final Alert motorDisconnectedAlert =
      new Alert("Intake pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert encoderDisconnectedAlert =
      new Alert("Intake pivot encoder disconnected!", Alert.AlertType.kWarning);

  public Pivot(PivotIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));

    if (Constants.getCurrentMode() != Mode.REAL) {
      homed = true;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected && Constants.getCurrentMode() == Mode.REAL);
    encoderDisconnectedAlert.set(
        !inputs.encoderConnected && Constants.getCurrentMode() == Mode.REAL);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Get current angle
    angle = inputs.internalPosition;
    if (!profileInitialized) {
      setpoint = new State(angle.getRadians(), 0.0);
      profileInitialized = true;
    }

    // Run profile
    boolean manualMode = manualModeEnabled.get() > 0.5;

    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && homed
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Intake/Pivot/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Units.radiansToDegrees(Math.abs(angle.getRadians() - setpoint.position)) > tolerance.get();

    shouldEStop =
        outOfTolerance && shouldRunProfile
            || angle.getRadians() < Intake.minAngle.getRadians()
            || angle.getRadians() > Intake.maxAngle.getRadians();

    if (shouldRunProfile) {
      double goalRadians =
          manualMode
              ? Rotation2d.fromDegrees(manualPivotAngleDeg.get()).getRadians()
              : goal.getAsDouble();

      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(
                  goalRadians, Intake.minAngle.getRadians(), Intake.maxAngle.getRadians()),
              0.0);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      io.runPosition(
          Rotation2d.fromRadians(setpoint.position),
          kS.get() * Math.signum(setpoint.velocity) + kG.get() * angle.getCos());

      // Check at goal
      atGoal = EqualsUtil.epsilonEquals(setpoint.position, goalState.position);

      // Log state
      Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionRad", setpoint.position);
      Logger.recordOutput(
          "Intake/Pivot/Profile/SetpointPositionDeg", Math.toDegrees(setpoint.position));
      Logger.recordOutput("Intake/Pivot/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
      Logger.recordOutput("Intake/Pivot/Profile/GoalPositionRad", goalState.position);
      Logger.recordOutput(
          "Intake/Pivot/Profile/GoalPositionDeg", Math.toDegrees(goalState.position));
      Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityRadPerSec", goalState.velocity);
      Logger.recordOutput(
          "Intake/Pivot/Profile/GoalVelocityDegPerSec", Math.toDegrees(goalState.velocity));
    } else {
      // Reset setpoint
      setpoint = new State(angle.getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionRad", 0.0);
      Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionDeg", 0.0);
      Logger.recordOutput("Intake/Pivot/Profile/SetpointVelocityRadPerSec", 0.0);
      Logger.recordOutput("Intake/Pivot/Profile/GoalPositionRad", 0.0);
      Logger.recordOutput("Intake/Pivot/Profile/GoalPositionDeg", 0.0);
      Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityRadPerSec", 0.0);
      Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityDegPerSec", 0.0);
    }

    // Log state
    Logger.recordOutput("Intake/Pivot/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Intake/Pivot/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput("Intake/Pivot/ManualMode", manualMode);
    Logger.recordOutput("Intake/Pivot/Manual/TargetAngleDeg", manualPivotAngleDeg.get());
    Logger.recordOutput("Intake/Pivot/MeasuredPositionDeg", angle.getDegrees());
    Logger.recordOutput("Intake/Pivot/MeasuredVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput(
        "Intake/Pivot/MeasuredVelocityDegPerSec", Math.toDegrees(inputs.velocityRadPerSec));
    Logger.recordOutput("Intake/Pivot/Homed", homed);
  }

  public void setGoal(Supplier<Rotation2d> goal) {
    setGoal(() -> goal.get().getRadians());
  }

  public void setGoal(DoubleSupplier goal) {
    atGoal = false;
    this.goal = goal;
  }

  public double getGoal() {
    return goal.getAsDouble();
  }

  private void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void runOpenLoop(double amps) {
    io.runOpenLoop(amps);
  }

  public void stop() {
    io.stop();
  }

  public void setPosition(double degrees) {
    io.setPosition(degrees);
  }

  public void homeToStowed() {
    io.homeToStowed();
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) {
                io.stop();
                return;
              }
              io.runVolts(Math.abs(homingVolts.get()));
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
              Logger.recordOutput("Intake/Pivot/Homing", true);
              Logger.recordOutput("Intake/Pivot/HomingVelocityRadPerSec", inputs.velocityRadPerSec);
            },
            this)
        .until(() -> homed)
        .andThen(
            () -> {
              homeToStowed();
              setpoint = new State(Intake.stowedAngle.getRadians(), 0.0);
              homed = true;
            })
        .finallyDo(
            () -> {
              stopProfile = false;
              io.stop();
              Logger.recordOutput("Intake/Pivot/Homing", false);
            });
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Intake/Pivot/StaticCharacterizationOutput", state.characterizationOutput);
            },
            this)
        .until(() -> inputs.velocityRadPerSec >= staticVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Intake/Pivot/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
