package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
  private static final double MIN_ANGLE_RAD = Math.toRadians(20.0);
  private static final double MAX_ANGLE_RAD = Math.toRadians(70.0);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Hood/kP", 5.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Hood/kD", 0.1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Hood/kS", 0.2);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/Hood/kV", 0.1);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Shooter/Hood/kG", 0.3);

  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Shooter/Hood/MaxVelocityDegPerSec", 200.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Shooter/Hood/MaxAccelerationDegPerSec2", 400.0);

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private TrapezoidProfile profile;
  @Getter private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  @Getter private double targetAngleRad = MIN_ANGLE_RAD;
  @Getter private double targetVelocityRadPerSec = 0.0;
  private boolean closedLoop = false;

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

    // Run closed loop control
    if (closedLoop) {
      // Clamp target to limits
      double clampedTarget = MathUtil.clamp(targetAngleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);

      // Create goal state with feedforward velocity
      var goalState = new TrapezoidProfile.State(clampedTarget, targetVelocityRadPerSec);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      // Calculate feedforward: kS for static friction, kV for velocity, kG for gravity
      double feedforward =
          kS.get() * Math.signum(setpoint.velocity)
              + kV.get() * setpoint.velocity
              + kG.get() * Math.cos(setpoint.position); // Gravity compensation

      io.runPosition(Rotation2d.fromRadians(setpoint.position), feedforward);

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
    Logger.recordOutput("Shooter/Hood/TargetAngleRad", targetAngleRad);
    Logger.recordOutput("Shooter/Hood/TargetAngleDeg", Math.toDegrees(targetAngleRad));
    Logger.recordOutput("Shooter/Hood/TargetVelocityRadPerSec", targetVelocityRadPerSec);
    Logger.recordOutput("Shooter/Hood/SetpointAngleRad", setpoint.position);
    Logger.recordOutput("Shooter/Hood/SetpointAngleDeg", Math.toDegrees(setpoint.position));
    Logger.recordOutput("Shooter/Hood/SetpointVelocityRadPerSec", setpoint.velocity);
    Logger.recordOutput("Shooter/Hood/ActualAngleRad", inputs.position.getRadians());
    Logger.recordOutput("Shooter/Hood/ActualAngleDeg", inputs.position.getDegrees());
    Logger.recordOutput("Shooter/Hood/ActualVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput("Shooter/Hood/AtGoal", atGoal);
    Logger.recordOutput("Shooter/Hood/ClosedLoop", closedLoop);
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
    return inputs.position;
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
}
