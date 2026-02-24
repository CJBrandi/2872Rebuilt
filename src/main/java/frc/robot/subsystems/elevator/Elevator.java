// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public enum Target {
    UP(Constants.ElevatorConstants.upPositionMeters),
    TRANSITION(Constants.ElevatorConstants.transitionPositionMeters),
    DOWN(Constants.ElevatorConstants.downPositionMeters),
    AUTO(Constants.ElevatorConstants.autoPositionMeters);

    public final double positionMeters;

    Target(double positionMeters) {
      this.positionMeters = positionMeters;
    }
  }

  public static final double sprocketRadius = Units.inchesToMeters(1.756 / 2.0);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");
  // Fast profile (TRANSITION -> DOWN, DOWN -> UP)
  public static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/Fast/MaxVelocityMetersPerSec", 0.5);
  public static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/Fast/MaxAccelerationMetersPerSec2", 1.0);
  // Slow profile (UP -> TRANSITION)
  public static final LoggedTunableNumber slowMaxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/Slow/MaxVelocityMetersPerSec", 0.15);
  public static final LoggedTunableNumber slowMaxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/Slow/MaxAccelerationMetersPerSec2", 0.3);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Elevator/HomingVelocityThresh", 5.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.2);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/Tolerance", 1);

  static {
    switch (Constants.getCurrentMode()) {
      case REAL -> {
        kP.initDefault(700);
        kI.initDefault(5);
        kD.initDefault(25);
        kS.initDefault(2.5);
        kG.initDefault(10);
        kA.initDefault(0);
      }
      case SIM, REPLAY -> {
        kP.initDefault(3000);
        kI.initDefault(10);
        kD.initDefault(800);
        kS.initDefault(5);
        kG.initDefault(50);
        kA.initDefault(0.0);
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  private TrapezoidProfile fastProfile;
  private TrapezoidProfile slowProfile;
  @Getter private State setpoint = new State();
  private double previousSetpointVelocityMetersPerSec = 0.0;
  @Getter private Target target = Target.DOWN;
  private Target previousTarget = Target.DOWN;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @AutoLogOutput(key = "Elevator/HomedPositionRad")
  private double homedPosition = 0.0;

  @AutoLogOutput @Getter private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private boolean stowed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    fastProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
    slowProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                slowMaxVelocityMetersPerSec.get(), slowMaxAccelerationMetersPerSec2.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }
    if (maxVelocityMetersPerSec.hasChanged(hashCode())
        || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      fastProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
    }
    if (slowMaxVelocityMetersPerSec.hasChanged(hashCode())
        || slowMaxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      slowProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  slowMaxVelocityMetersPerSec.get(), slowMaxAccelerationMetersPerSec2.get()));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && homed
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);
    // Check if out of tolerance
    boolean outOfTolerance = Math.abs(getPositionMeters() - setpoint.position) > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Select profile based on movement direction:
      // Slow profile: UP -> TRANSITION, AUTO -> DOWN
      // Fast profile: TRANSITION -> DOWN, DOWN -> UP, DOWN -> AUTO
      boolean useSlowProfile =
          (previousTarget == Target.UP && target == Target.TRANSITION)
              || (previousTarget == Target.AUTO && target == Target.DOWN);
      TrapezoidProfile activeProfile = useSlowProfile ? slowProfile : fastProfile;
      Logger.recordOutput("Elevator/Profile/UsingSlowProfile", useSlowProfile);

      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(
                  target.positionMeters,
                  ElevatorConstants.downPositionMeters,
                  ElevatorConstants.upPositionMeters),
              0.0);
      setpoint = activeProfile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      if (setpoint.position < 0.0 || setpoint.position > ElevatorConstants.upPositionMeters) {
        setpoint =
            new State(
                MathUtil.clamp(
                    setpoint.position,
                    ElevatorConstants.downPositionMeters,
                    ElevatorConstants.upPositionMeters),
                0.0);
      }
      double accelerationMetersPerSec2 =
          (setpoint.velocity - previousSetpointVelocityMetersPerSec) / Constants.loopPeriodSecs;
      previousSetpointVelocityMetersPerSec = setpoint.velocity;

      io.runPosition(
          setpoint.position / sprocketRadius + homedPosition,
          kS.get() * Math.signum(setpoint.velocity)
              + (target != Target.UP ? kG.get() : 0)
              + (kA.get() * accelerationMetersPerSec2));
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Stop running elevator down when in stow
      if (stowed && atGoal) {
        io.stop();
      }

      // Log state
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    } else {
      // Reset setpoint
      setpoint = new State(getPositionMeters(), 0.0);
      previousSetpointVelocityMetersPerSec = 0.0;

      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }
    if (isEStopped) {
      io.stop();
    }

    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec", inputs.velocityRadPerSec * sprocketRadius);
    Logger.recordOutput("Elevator/Target", getTarget());
  }

  public Command setTarget(Target target) {
    return Commands.runOnce(
        () -> {
          atGoal = false;
          this.previousTarget = this.target;
          this.target = target;
        });
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public Command staticCharacterization(double currentRampRateAmpsPerSec) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationCurrentAmps = currentRampRateAmpsPerSec * timer.get();
              System.out.println("Current Ramp Rate: " + state.characterizationCurrentAmps);
              io.runOpenLoop(state.characterizationCurrentAmps);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationCurrentAmps", state.characterizationCurrentAmps);
            })
        .until(() -> inputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Elevator/CharacterizationCurrentAmps", state.characterizationCurrentAmps);
            });
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
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(
            () -> {
              homedPosition = inputs.positionRad;
              homed = true;
            })
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  /** Get position of elevator in meters with 0 at home */
  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return (inputs.positionRad - homedPosition) * sprocketRadius;
  }

  public double getGoalMeters() {
    return target.positionMeters;
  }

  private static class StaticCharacterizationState {
    public double characterizationCurrentAmps = 0.0;
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void runOpenLoop(double amps) {
    io.runOpenLoop(amps);
  }
}
