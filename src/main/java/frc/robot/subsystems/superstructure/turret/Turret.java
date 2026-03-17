package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Turret/MaxVelocityDegreesPerSec");
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Turret/MaxAccelerationDegreesPerSec2");
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Turret/StaticCharacterizationVelocityThreshRadPerSec");

  // Manual mode tunables
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled");
  private static final LoggedTunableNumber manualAngleDeg =
      new LoggedTunableNumber("Manual/YawDeg");

  static {
    switch (Constants.getCurrentMode()) {
      case REAL -> {
        kP.initDefault(300);
        kD.initDefault(0);
        kS.initDefault(0.2);
        kV.initDefault(0.0);
        maxVelocityDegPerSec.initDefault(360);
        maxAccelerationDegPerSec2.initDefault(720);
      }
      case SIM, REPLAY -> {
        kP.initDefault(8000);
        kD.initDefault(1.0);
        kS.initDefault(0.2);
        kV.initDefault(0.0);
        maxVelocityDegPerSec.initDefault(9000);
        maxAccelerationDegPerSec2.initDefault(18000);
      }
    }
    staticCharacterizationVelocityThresh.initDefault(0.1);
    manualModeEnabled.initDefault(0.0);
    manualAngleDeg.initDefault(-180);
  }

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

  // Tracks the last commanded goal for logging/debugging.
  private double lastGoalAngle = 0.0;

  @AutoLogOutput(key = "Turret/StaticCharacterizationActive")
  private boolean staticCharacterizationActive = false;

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
      turretIO.stop();
      double clampedAngle = TurretLimits.clampRadians(inputs.motorEncoderPosition.getRadians());
      setpoint = new TrapezoidProfile.State(clampedAngle, 0.0);
      lastGoalAngle = clampedAngle;
      atGoal = false;

      // Still publish turret observation for vision even when disabled
      RobotState.getInstance()
          .addTurretObservation(
              new RobotState.TurretObservation(
                  Timer.getFPGATimestamp(), inputs.motorEncoderPosition));

      logState();
      TurretVisualizer.update(inputs.motorEncoderPosition.getRadians());
      return;
    }

    // Publish turret observation to RobotState
    RobotState.getInstance()
        .addTurretObservation(
            new RobotState.TurretObservation(
                Timer.getFPGATimestamp(), inputs.motorEncoderPosition));

    boolean manualMode = manualModeEnabled.get() > 0.5;
    // Manual mode - use tunable angle as target
    if (manualMode && !staticCharacterizationActive) {
      setTargetTurretAngle(Rotation2d.fromDegrees(manualAngleDeg.get()));
    }

    if (closedLoop
        && !staticCharacterizationActive
        && targetFieldRelativeAngle != null
        && !manualMode) {
      // Convert field-relative angle to robot-relative angle
      double robotAngleRad = RobotState.getInstance().getRobotPose().getRotation().getRadians();
      double robotAngularVelocity =
          RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle =
          Rotation2d.fromRadians(targetFieldRelativeAngle.getRadians() - robotAngleRad);
      double robotRelativeGoalVelocity = targetVelocityRadPerSec - robotAngularVelocity;

      double bestAngle =
          selectFieldRelativeAngleWithinLimits(robotRelativeGoalAngle.getRadians())
              .selectedRadians();
      lastGoalAngle = bestAngle;
      targetTurretAngle = Rotation2d.fromRadians(bestAngle);

      // Create goal state with feedforward velocity
      var goalState = new TrapezoidProfile.State(bestAngle, robotRelativeGoalVelocity);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      setpoint = clampSetpointToLimits(setpoint);

      // Calculate feedforward: kS for static friction, kV for velocity
      double feedforward = kS.get() * Math.signum(setpoint.velocity) + kV.get() * setpoint.velocity;

      turretIO.runPosition(Rotation2d.fromRadians(setpoint.position), feedforward);

      // Check if at goal
      atGoal = EqualsUtil.epsilonEquals(setpoint.position, bestAngle, Units.degreesToRadians(5.0));

      Logger.recordOutput("Turret/GoalAngleRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
    } else if (closedLoop && !staticCharacterizationActive && targetTurretAngle != null) {
      // Direct turret angle control (no field-relative conversion)
      double bestAngle =
          selectAbsoluteAngleWithinLimits(targetTurretAngle.getRadians()).selectedRadians();
      lastGoalAngle = bestAngle;
      targetTurretAngle = Rotation2d.fromRadians(bestAngle);

      var goalState = new TrapezoidProfile.State(bestAngle, targetVelocityRadPerSec);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      setpoint = clampSetpointToLimits(setpoint);

      double feedforward = kS.get() * Math.signum(setpoint.velocity) + kV.get() * setpoint.velocity;

      turretIO.runPosition(Rotation2d.fromRadians(setpoint.position), feedforward);

      atGoal = EqualsUtil.epsilonEquals(setpoint.position, bestAngle, Units.degreesToRadians(5.0));

      Logger.recordOutput("Turret/GoalAngleRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", targetVelocityRadPerSec);
    } else {
      atGoal = false;
    }

    logState();
    TurretVisualizer.update(inputs.motorEncoderPosition.getRadians());
  }

  private TurretLimits.Selection selectAbsoluteAngleWithinLimits(double requestedAngleRad) {
    var selection = TurretLimits.selectAbsoluteAngleRadians(requestedAngleRad);
    logAngleSelection("absolute", selection);
    return selection;
  }

  private TurretLimits.Selection selectFieldRelativeAngleWithinLimits(double requestedAngleRad) {
    var selection = TurretLimits.selectFieldRelativeAngleRadians(requestedAngleRad);
    logAngleSelection("field_relative", selection);
    return selection;
  }

  private void logAngleSelection(String mode, TurretLimits.Selection selection) {
    Logger.recordOutput("Turret/Safety/SelectionMode", mode);
    Logger.recordOutput("Turret/Safety/RequestedAngleDeg", selection.requestedDeg());
    Logger.recordOutput("Turret/Safety/CandidateAngleDeg", selection.candidateDeg());
    Logger.recordOutput("Turret/Safety/SelectedAngleDeg", selection.selectedDeg());
    Logger.recordOutput("Turret/Safety/SelectionClamped", selection.clamped());
  }

  private TrapezoidProfile.State clampSetpointToLimits(TrapezoidProfile.State state) {
    double clampedPosition = TurretLimits.clampRadians(state.position);
    boolean clamped = Math.abs(clampedPosition - state.position) > 1e-9;
    Logger.recordOutput("Turret/Safety/SetpointClamped", clamped);

    if (!clamped) {
      return state;
    }

    double velocity = state.velocity;
    // If clamped at a hard limit and velocity is pushing farther out, zero velocity.
    if (TurretLimits.commandWouldPushPastLimit(clampedPosition, velocity)) {
      velocity = 0.0;
    }
    return new TrapezoidProfile.State(clampedPosition, velocity);
  }

  private void logState() {
    Logger.recordOutput("Turret/ClosedLoop", closedLoop);
    Logger.recordOutput("Turret/ManualMode", manualModeEnabled.get() > 0.5);
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
    Logger.recordOutput("Turret/Safety/MinAngleDeg", TurretLimits.MIN_ANGLE_DEG);
    Logger.recordOutput("Turret/Safety/MaxAngleDeg", TurretLimits.MAX_ANGLE_DEG);
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

  /** Returns whether the motor is connected. */
  public boolean isMotorConnected() {
    return motorConnectedDebouncer.calculate(inputs.motorConnected);
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
              staticCharacterizationActive = true;
              closedLoop = false;
              timer.restart();
            },
            () -> {
              // Keep closed-loop off even if other code paths set targets while characterizing.
              closedLoop = false;
              state.characterizationCurrentAmps = currentRampRateAmpsPerSec * timer.get();
              System.out.println("Turret Current Ramp: " + state.characterizationCurrentAmps);
              turretIO.runCurrent(state.characterizationCurrentAmps);
              Logger.recordOutput(
                  "Turret/StaticCharacterizationCurrentAmps", state.characterizationCurrentAmps);
            })
        .until(
            () -> Math.abs(inputs.velocityRadPerSec) >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              staticCharacterizationActive = false;
              closedLoop = true;
              timer.stop();
              turretIO.stop();
              Logger.recordOutput(
                  "Turret/CharacterizationResultCurrentAmps", state.characterizationCurrentAmps);
            });
  }
}
