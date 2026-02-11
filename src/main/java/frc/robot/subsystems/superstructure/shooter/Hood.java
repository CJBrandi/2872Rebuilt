package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Hood (pitch) control for the shooter. Handles position control with motion profiling. This is not
 * a Subsystem - it's managed by Shooter.
 */
public class Hood {

  // Hood angle limits (radians)
  private static final double MIN_ANGLE_RAD = Math.toRadians(15.5);
  private static final double MAX_ANGLE_RAD = MIN_ANGLE_RAD + Math.toRadians(34.5);

  // PID gains (output in Amps for TorqueCurrentFOC)
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Hood/kP", 2000);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Hood/kD", 0.0);
  // Feedforward gains (Amps)
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Hood/kS", -2.0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Shooter/Hood/kG", 3.0);

  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Shooter/Hood/MaxVelocityDegPerSec", 90);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Shooter/Hood/MaxAccelerationDegPerSec2", 180);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Shooter/Hood/StaticCharacterizationVelocityThreshRadPerSec", 0.1);

  // Homing parameters
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Shooter/Hood/HomingVolts", -1.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Shooter/Hood/HomingVelocityThreshRadPerSec", 0.3);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Shooter/Hood/HomingTimeSecs", 0.1);

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private TrapezoidProfile profile;
  @Getter private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  @Getter private double targetAngleRad = MIN_ANGLE_RAD;
  @Getter private double targetVelocityRadPerSec = 0.0;
  private boolean closedLoop = false;
  private boolean stopProfile = false;

  // Homing state
  @Getter private boolean homed = false;
  private double homedPosition = 0.0;
  private Debouncer homingDebouncer;

  @Getter private boolean atGoal = false;

  public Hood(HoodIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    setpoint = new TrapezoidProfile.State(MIN_ANGLE_RAD, 0.0);
  }

  /** Called by Shooter.periodic() */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);

    // Update PID gains if changed
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get());
    }

    // Update profile constraints if changed
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }

    // Run closed loop control (skip if stopProfile is set for characterization)
    if (closedLoop && !stopProfile) {
      // Clamp target to limits
      double clampedTarget = MathUtil.clamp(targetAngleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);

      // Create goal state with feedforward velocity
      var goalState = new TrapezoidProfile.State(clampedTarget, targetVelocityRadPerSec);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      // Calculate feedforward in Amps: kS for static friction, kV for velocity, kG for gravity
      double feedforwardAmps =
          kS.get() * Math.signum(setpoint.velocity) + Math.signum(setpoint.velocity) > 0
              ? kG.get() * Math.cos(setpoint.position)
              : 0; // Gravity compensation

      io.runPosition(setpoint.position, feedforwardAmps);

      // Check if at goal
      atGoal =
          EqualsUtil.epsilonEquals(
                  setpoint.position, goalState.position, Units.degreesToRadians(0.5))
              && EqualsUtil.epsilonEquals(
                  setpoint.velocity, goalState.velocity, Units.degreesToRadians(5.0));
    } else {
      atGoal = false;
    }

    // Logging
    Logger.recordOutput("Shooter/Hood/TargetAngleDeg", Math.toDegrees(targetAngleRad));
    Logger.recordOutput("Shooter/Hood/SetpointAngleDeg", Math.toDegrees(setpoint.position));
    Logger.recordOutput("Shooter/Hood/MeasuredAngleDeg", Math.toDegrees(inputs.positionRad));
    Logger.recordOutput("Shooter/Hood/AtGoal", atGoal);
    Logger.recordOutput("Shooter/Hood/ClosedLoop", closedLoop);
    Logger.recordOutput("Shooter/Hood/Homed", homed);
  }

  /**
   * Sets the target hood angle.
   *
   * @param angleRad Target angle in radians
   * @param velocityRadPerSec Feedforward velocity in rad/s
   */
  public void setTargetAngle(double angleRad, double velocityRadPerSec) {
    closedLoop = true;
    this.targetAngleRad = angleRad;
    this.targetVelocityRadPerSec = velocityRadPerSec;
  }

  /**
   * Sets the target hood angle with no feedforward velocity.
   *
   * @param angleRad Target angle in radians
   */
  public void setTargetAngle(double angleRad) {
    setTargetAngle(angleRad, 0.0);
  }

  /** Returns the current hood angle. */
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.positionRad);
  }

  /** Returns the current hood angular velocity in rad/s. */
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public void runVolts(double volts) {
    closedLoop = false;
    io.runVolts(volts);
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    io.runOpenLoop(output);
  }

  public void stop() {
    closedLoop = false;
    targetVelocityRadPerSec = 0.0;
    io.stop();
  }

  /** Resets the hood position and profile setpoint. */
  public void resetPosition(double radians) {
    io.setPosition(radians);
    setpoint = new TrapezoidProfile.State(radians, 0.0);
  }

  public boolean isMotorConnected() {
    return inputs.motorConnected;
  }

  /** Returns the minimum allowed hood angle in radians. */
  public static double getMinAngleRad() {
    return MIN_ANGLE_RAD;
  }

  /** Returns the maximum allowed hood angle in radians. */
  public static double getMaxAngleRad() {
    return MAX_ANGLE_RAD;
  }

  /** State class for static characterization. */
  private static class StaticCharacterizationState {
    public double characterizationCurrentAmps = 0.0;
  }

  /**
   * Creates a command for static characterization that ramps current until motion is detected.
   *
   * @param currentRampRateAmpsPerSec Rate at which to increase current (amps per second)
   * @return Command that runs the characterization
   */
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
              System.out.println("Hood Current Ramp: " + state.characterizationCurrentAmps);
              io.runOpenLoop(state.characterizationCurrentAmps);
              Logger.recordOutput(
                  "Shooter/Hood/StaticCharacterizationCurrentAmps",
                  state.characterizationCurrentAmps);
            })
        .until(
            () -> Math.abs(inputs.velocityRadPerSec) >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              io.stop();
              Logger.recordOutput(
                  "Shooter/Hood/CharacterizationResultAmps", state.characterizationCurrentAmps);
            });
  }

  /**
   * Creates a command for homing the hood by running into the hard stop.
   *
   * @return Command that runs the homing sequence
   */
  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
              Logger.recordOutput("Shooter/Hood/Homing", true);
              Logger.recordOutput("Shooter/Hood/HomingVelocity", inputs.velocityRadPerSec);
            })
        .until(() -> homed)
        .andThen(
            () -> {
              homedPosition = inputs.positionRad;
              resetPosition(MIN_ANGLE_RAD);
              homed = true;
              Logger.recordOutput("Shooter/Hood/HomedPosition", homedPosition);
            })
        .finallyDo(
            () -> {
              stopProfile = false;
              io.stop();
              Logger.recordOutput("Shooter/Hood/Homing", false);
            });
  }
}
