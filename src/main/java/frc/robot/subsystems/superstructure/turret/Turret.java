package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Turret control. This is not a Subsystem - it's managed by Superstructure. */
public class Turret {

  // Turret angle limits (relative to robot, in radians)
  private static final double MIN_ANGLE_RAD = 0.0;
  private static final double MAX_ANGLE_RAD = (3 * Math.PI) / 2; // 270 degrees

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 5.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.2);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.1);
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Turret/MaxVelocityDegreesPerSec", 1500.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Turret/MaxAccelerationDegreesPerSec2", 2500.0);

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private TrapezoidProfile profile;
  @Getter private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  // Target angles set by Superstructure
  @Getter private Rotation2d targetFieldRelativeAngle = null; // null = no target
  @Getter private Rotation2d targetTurretAngle = null; // Calculated turret-relative angle
  @Getter private double targetVelocityRadPerSec = 0.0; // Feedforward velocity from ShotCalculator

  // Wrap-around tracking - remembers last goal to pick closest valid solution
  private double lastGoalAngle = (MIN_ANGLE_RAD + MAX_ANGLE_RAD) / 2.0;

  private boolean closedLoop = false;

  @Getter
  @AutoLogOutput(key = "Turret/Profile/AtGoal")
  private boolean atGoal = false;

  public Turret(TurretIO io) {
    turretIO = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
  }

  public void periodic() {
    turretIO.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Update PID gains if changed
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      turretIO.setPID(kP.get(), 0.0, kD.get());
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

    // Handle disabled state - brake and reset profile
    if (DriverStation.isDisabled()) {
      turretIO.setBrakeMode(true);
      turretIO.stop();
      setpoint = new TrapezoidProfile.State(inputs.motorEncoderPosition.getRadians(), 0.0);
      lastGoalAngle = inputs.motorEncoderPosition.getRadians();
      atGoal = false;

      // Still publish turret observation for vision even when disabled
      RobotState.getInstance()
          .addTurretObservation(
              new RobotState.TurretObservation(
                  Timer.getFPGATimestamp(), inputs.motorEncoderPosition));

      logState();
      TurretVisualizer.update(setpoint.position);
      return;
    }

    // Publish turret observation to RobotState
    RobotState.getInstance()
        .addTurretObservation(
            new RobotState.TurretObservation(
                Timer.getFPGATimestamp(), inputs.motorEncoderPosition));

    // Run closed loop control if enabled and we have a target
    if (closedLoop && targetFieldRelativeAngle != null) {
      // Convert field-relative angle to robot-relative angle
      double robotAngleRad = RobotState.getInstance().getRobotPose().getRotation().getRadians();
      double robotAngularVelocity =
          RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle =
          Rotation2d.fromRadians(targetFieldRelativeAngle.getRadians() - robotAngleRad);
      double robotRelativeGoalVelocity = targetVelocityRadPerSec - robotAngularVelocity;

      // Find the best angle among all valid ±2π wraps (closest to current goal)
      double bestAngle = findBestAngle(robotRelativeGoalAngle.getRadians());
      lastGoalAngle = bestAngle;
      targetTurretAngle = Rotation2d.fromRadians(bestAngle);

      // Create goal state with feedforward velocity
      var goalState = new TrapezoidProfile.State(bestAngle, robotRelativeGoalVelocity);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      // Calculate feedforward: kS for static friction, kV for velocity
      double feedforward = kS.get() * Math.signum(setpoint.velocity) + kV.get() * setpoint.velocity;

      turretIO.runPosition(Rotation2d.fromRadians(setpoint.position), feedforward);

      // Check if at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, bestAngle, Units.degreesToRadians(1.0))
              && EqualsUtil.epsilonEquals(
                  setpoint.velocity, robotRelativeGoalVelocity, Units.degreesToRadians(10.0));

      Logger.recordOutput("Turret/GoalAngleRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
    } else if (closedLoop && targetTurretAngle != null) {
      // Direct turret angle control (no field-relative conversion)
      double bestAngle = findBestAngle(targetTurretAngle.getRadians());
      lastGoalAngle = bestAngle;

      var goalState = new TrapezoidProfile.State(bestAngle, targetVelocityRadPerSec);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      double feedforward = kS.get() * Math.signum(setpoint.velocity) + kV.get() * setpoint.velocity;

      turretIO.runPosition(Rotation2d.fromRadians(setpoint.position), feedforward);

      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, bestAngle, Units.degreesToRadians(1.0))
              && EqualsUtil.epsilonEquals(
                  setpoint.velocity, targetVelocityRadPerSec, Units.degreesToRadians(10.0));

      Logger.recordOutput("Turret/GoalAngleRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", targetVelocityRadPerSec);
    } else {
      atGoal = false;
    }

    logState();
    TurretVisualizer.update(setpoint.position);
  }

  /**
   * Finds the best turret angle among all valid ±2π wraps. Picks the angle closest to the last goal
   * to minimize travel and avoid unnecessary wrapping.
   */
  private double findBestAngle(double robotRelativeGoalRad) {
    boolean hasBestAngle = false;
    double bestAngle = lastGoalAngle;

    // Check all possible wraps: -2, -1, 0, +1, +2 rotations
    for (int i = -2; i <= 2; i++) {
      double potentialAngle = robotRelativeGoalRad + 2.0 * Math.PI * i;

      // Skip if outside turret limits
      if (potentialAngle < MIN_ANGLE_RAD || potentialAngle > MAX_ANGLE_RAD) {
        continue;
      }

      // First valid angle becomes the initial best
      if (!hasBestAngle) {
        bestAngle = potentialAngle;
        hasBestAngle = true;
        continue;
      }

      // Pick the angle closest to last goal (minimizes travel)
      if (Math.abs(potentialAngle - lastGoalAngle) < Math.abs(bestAngle - lastGoalAngle)) {
        bestAngle = potentialAngle;
      }
    }

    // If no valid angle found, clamp to limits
    if (!hasBestAngle) {
      bestAngle = MathUtil.clamp(robotRelativeGoalRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
    }

    return bestAngle;
  }

  private void logState() {
    Logger.recordOutput("Turret/ClosedLoop", closedLoop);
    Logger.recordOutput(
        "Turret/TargetFieldRelativeAngleDeg",
        targetFieldRelativeAngle != null ? targetFieldRelativeAngle.getDegrees() : 0.0);
    Logger.recordOutput(
        "Turret/TargetTurretAngleDeg",
        targetTurretAngle != null ? targetTurretAngle.getDegrees() : 0.0);
    Logger.recordOutput("Turret/TargetVelocityRadPerSec", targetVelocityRadPerSec);
    Logger.recordOutput("Turret/Profile/SetpointAngleRad", setpoint.position);
    Logger.recordOutput(
        "Turret/Profile/SetpointAngleDeg", Units.radiansToDegrees(setpoint.position));
    Logger.recordOutput("Turret/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
    Logger.recordOutput("Turret/ActualAngleDeg", inputs.motorEncoderPosition.getDegrees());
    Logger.recordOutput("Turret/ActualVelocityRadPerSec", inputs.velocityRadPerSec);
    Logger.recordOutput("Turret/FieldRelativeAngleDeg", getFieldRelativeAngle().getDegrees());
    Logger.recordOutput("Turret/LastGoalAngleDeg", Units.radiansToDegrees(lastGoalAngle));
  }

  /**
   * Sets the target field-relative angle for the turret. The turret will automatically convert this
   * to a turret-relative angle based on the robot's current heading.
   *
   * @param fieldAngle Field-relative angle to point at
   * @param velocityRadPerSec Feedforward velocity in rad/s (from ShotCalculator)
   */
  public void setTargetFieldRelativeAngle(Rotation2d fieldAngle, double velocityRadPerSec) {
    closedLoop = true;
    this.targetFieldRelativeAngle = fieldAngle;
    this.targetTurretAngle = null; // Clear direct turret angle
    this.targetVelocityRadPerSec = velocityRadPerSec;
  }
  /**
   * Sets the target turret-relative angle directly (no field-relative conversion).
   *
   * @param turretAngle Turret-relative angle
   * @param velocityRadPerSec Feedforward velocity in rad/s
   */
  public void setTargetTurretAngle(Rotation2d turretAngle, double velocityRadPerSec) {
    closedLoop = true;
    this.targetFieldRelativeAngle = null; // Clear field-relative angle
    this.targetTurretAngle = turretAngle;
    this.targetVelocityRadPerSec = velocityRadPerSec;
  }

  /**
   * Sets the target turret-relative angle directly with no feedforward velocity.
   *
   * @param turretAngle Turret-relative angle
   */
  public void setTargetTurretAngle(Rotation2d turretAngle) {
    setTargetTurretAngle(turretAngle, 0.0);
  }

  /** Returns the current turret angle relative to the robot. */
  public Rotation2d getTurretAngle() {
    return inputs.motorEncoderPosition;
  }

  /** Returns the current field-relative angle the turret is pointing. */
  public Rotation2d getFieldRelativeAngle() {
    double robotAngleRad = RobotState.getInstance().getRobotPose().getRotation().getRadians();
    return Rotation2d.fromRadians(inputs.motorEncoderPosition.getRadians() + robotAngleRad);
  }

  /** Returns the current turret angular velocity in rad/s. */
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  /** Runs the turret in open loop. */
  public void runOpenLoop(double output) {
    closedLoop = false;
    targetFieldRelativeAngle = null;
    targetTurretAngle = null;
    turretIO.runOpenLoop(output);
  }

  /** Runs the turret at a specified voltage. */
  public void runVolts(double volts) {
    closedLoop = false;
    targetFieldRelativeAngle = null;
    targetTurretAngle = null;
    turretIO.runVolts(volts);
  }

  /** Stops the turret and clears targets. */
  public void stop() {
    closedLoop = false;
    targetFieldRelativeAngle = null;
    targetTurretAngle = null;
    targetVelocityRadPerSec = 0.0;
    turretIO.stop();
  }

  /** Resets the turret encoder position and profile setpoint to the specified angle in degrees. */
  public void resetPosition(double degrees) {
    turretIO.setPosition(degrees);
    setpoint = new TrapezoidProfile.State(Math.toRadians(degrees), 0.0);
    lastGoalAngle = Math.toRadians(degrees);
  }

  /** Returns whether the motor is connected. */
  public boolean isMotorConnected() {
    return motorConnectedDebouncer.calculate(inputs.motorConnected);
  }
}
