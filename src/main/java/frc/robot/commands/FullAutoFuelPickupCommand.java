package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fuelpickup.FuelPickupZonePolicy;
import frc.robot.subsystems.fuelpickup.FuelPickupZonePolicy.AllowedXRange;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.intake.Roller;
import frc.robot.util.FieldConstants;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Full autonomous fuel pickup with shift-aware zone constraints. */
public class FullAutoFuelPickupCommand extends Command {
  private static final double EPSILON = 1e-6;

  private static final LoggedTunableNumber cruiseSpeedMps =
      new LoggedTunableNumber("FuelPickupAuto/CruiseSpeedMps", 2.0);
  private static final LoggedTunableNumber maxLinearAccelMps2 =
      new LoggedTunableNumber("FuelPickupAuto/MaxLinearAccelMps2", 1.0);
  private static final LoggedTunableNumber searchOmegaRadPerSec =
      new LoggedTunableNumber("FuelPickupAuto/SearchOmegaRadPerSec", 0.4);
  private static final LoggedTunableNumber maxOmegaRadPerSec =
      new LoggedTunableNumber("FuelPickupAuto/MaxOmegaRadPerSec", 1.0);
  private static final LoggedTunableNumber maxOmegaAccelRadPerSec2 =
      new LoggedTunableNumber("FuelPickupAuto/MaxOmegaAccelRadPerSec2", 2.0);
  private static final LoggedTunableNumber headingKp =
      new LoggedTunableNumber("FuelPickupAuto/HeadingKp", 5.0);
  private static final LoggedTunableNumber headingKd =
      new LoggedTunableNumber("FuelPickupAuto/HeadingKd", 0.4);
  private static final LoggedTunableNumber targetSmoothingAlpha =
      new LoggedTunableNumber("FuelPickupAuto/TargetSmoothingAlpha", 0.35);
  private static final LoggedTunableNumber boundaryMarginM =
      new LoggedTunableNumber("FuelPickupAuto/BoundaryMarginM", 0.05);

  enum AutoMode {
    CHASE,
    RECOVER,
    SEARCH
  }

  private record HeadingControlResult(double omegaRadPerSec, double headingErrorRad) {}

  private final Drive drive;
  private final Pivot pivot;
  private final Roller roller;

  private double commandedFieldVx = 0.0;
  private double commandedFieldVy = 0.0;
  private double commandedOmega = 0.0;
  private double lastHeadingErrorRad = 0.0;
  private Translation2d smoothedCluster = null;
  private AutoMode mode = AutoMode.SEARCH;

  public FullAutoFuelPickupCommand(Drive drive, Pivot pivot, Roller roller) {
    this.drive = drive;
    this.pivot = pivot;
    this.roller = roller;
    addRequirements(drive, pivot, roller);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotRelativeSpeeds = RobotState.getInstance().getRobotVelocity();
    Translation2d fieldRelativeVelocity =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());
    commandedFieldVx = fieldRelativeVelocity.getX();
    commandedFieldVy = fieldRelativeVelocity.getY();
    commandedOmega = robotRelativeSpeeds.omegaRadiansPerSecond;
    lastHeadingErrorRad = 0.0;
    smoothedCluster = null;
    mode = AutoMode.SEARCH;
  }

  @Override
  public void execute() {
    pivot.setGoal(() -> Intake.groundAngle);
    roller.runIntake();

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    boolean ownAllianceShiftActive = HubShiftUtil.isOwnAllianceShiftActive();
    AllowedXRange allowedXRange =
        FuelPickupZonePolicy.getAllowedXRange(alliance, ownAllianceShiftActive);
    double margin = Math.max(0.0, boundaryMarginM.get());

    Pose2d robotPose = drive.getPose();
    boolean outsideAllowedX = isRobotOutsideAllowedX(robotPose.getX(), allowedXRange, margin);

    Translation2d targetCluster = null;
    if (RobotState.getInstance().isHasBestFuelCluster()) {
      Translation2d rawTargetCluster = RobotState.getInstance().getBestFuelCluster();
      smoothedCluster = smoothTarget(smoothedCluster, rawTargetCluster, targetSmoothingAlpha.get());
      targetCluster = smoothedCluster;
    } else {
      smoothedCluster = null;
    }

    boolean hasAllowedCluster =
        targetCluster != null
            && FuelPickupZonePolicy.isClusterAllowedX(targetCluster.getX(), allowedXRange);

    mode = selectMode(outsideAllowedX, hasAllowedCluster);

    Translation2d desiredFieldVelocity = Translation2d.kZero;
    double desiredOmega = 0.0;
    double headingErrorDeg = 0.0;

    if (mode == AutoMode.RECOVER) {
      double recoveryTargetX = computeRecoveryTargetX(robotPose.getX(), allowedXRange, margin);
      desiredFieldVelocity =
          computeCruiseVelocity(
              robotPose.getTranslation(),
              new Translation2d(recoveryTargetX, robotPose.getY()),
              cruiseSpeedMps.get());
      desiredFieldVelocity =
          enforceFieldBounds(desiredFieldVelocity, robotPose, allowedXRange, margin);
      HeadingControlResult headingControl =
          computeHeadingControl(
              desiredFieldVelocity, robotPose.getRotation(), headingKp.get(), headingKd.get());
      desiredOmega = headingControl.omegaRadPerSec();
      headingErrorDeg = Math.toDegrees(headingControl.headingErrorRad());
    } else if (mode == AutoMode.CHASE) {
      desiredFieldVelocity =
          computeCruiseVelocity(robotPose.getTranslation(), targetCluster, cruiseSpeedMps.get());
      desiredFieldVelocity =
          enforceFieldBounds(desiredFieldVelocity, robotPose, allowedXRange, margin);
      HeadingControlResult headingControl =
          computeHeadingControl(
              desiredFieldVelocity, robotPose.getRotation(), headingKp.get(), headingKd.get());
      desiredOmega = headingControl.omegaRadPerSec();
      headingErrorDeg = Math.toDegrees(headingControl.headingErrorRad());
    } else {
      desiredFieldVelocity = Translation2d.kZero;
      desiredOmega = searchOmegaRadPerSec.get();
      lastHeadingErrorRad = 0.0;
    }

    double maxOmega = Math.max(0.0, maxOmegaRadPerSec.get());
    desiredOmega = MathUtil.clamp(desiredOmega, -maxOmega, maxOmega);

    double maxLinearStep = Math.max(0.0, maxLinearAccelMps2.get()) * Constants.loopPeriodSecs;
    double maxOmegaStep = Math.max(0.0, maxOmegaAccelRadPerSec2.get()) * Constants.loopPeriodSecs;
    commandedFieldVx = slew(commandedFieldVx, desiredFieldVelocity.getX(), maxLinearStep);
    commandedFieldVy = slew(commandedFieldVy, desiredFieldVelocity.getY(), maxLinearStep);
    commandedOmega = slew(commandedOmega, desiredOmega, maxOmegaStep);
    commandedOmega = MathUtil.clamp(commandedOmega, -maxOmega, maxOmega);

    ChassisSpeeds commandedRobotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            commandedFieldVx, commandedFieldVy, commandedOmega, robotPose.getRotation());
    drive.runVelocity(commandedRobotSpeeds);

    Logger.recordOutput("FuelPickupAuto/Mode", mode.name());
    Logger.recordOutput(
        "FuelPickupAuto/AllowedXRange", new double[] {allowedXRange.minX(), allowedXRange.maxX()});
    Logger.recordOutput("FuelPickupAuto/ShiftActive", ownAllianceShiftActive);
    Logger.recordOutput("FuelPickupAuto/ShiftName", HubShiftUtil.getShiftName());
    Logger.recordOutput("FuelPickupAuto/ShiftRemainingSec", HubShiftUtil.getShiftRemainingSec());
    Logger.recordOutput(
        "FuelPickupAuto/TargetCluster",
        targetCluster == null ? new Translation2d() : targetCluster);
    Logger.recordOutput(
        "FuelPickupAuto/DesiredFieldVelocity",
        new double[] {desiredFieldVelocity.getX(), desiredFieldVelocity.getY()});
    Logger.recordOutput(
        "FuelPickupAuto/CommandedRobotSpeeds",
        new double[] {
          commandedRobotSpeeds.vxMetersPerSecond,
          commandedRobotSpeeds.vyMetersPerSecond,
          commandedRobotSpeeds.omegaRadiansPerSecond
        });
    Logger.recordOutput("FuelPickupAuto/HeadingErrorDeg", headingErrorDeg);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    roller.stop();
    pivot.setGoal(() -> Intake.stowedAngle);
    commandedFieldVx = 0.0;
    commandedFieldVy = 0.0;
    commandedOmega = 0.0;
    lastHeadingErrorRad = 0.0;
    smoothedCluster = null;
    mode = AutoMode.SEARCH;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  static AutoMode selectMode(boolean outsideAllowedX, boolean hasAllowedCluster) {
    if (outsideAllowedX) {
      return AutoMode.RECOVER;
    }
    if (hasAllowedCluster) {
      return AutoMode.CHASE;
    }
    return AutoMode.SEARCH;
  }

  static boolean isRobotOutsideAllowedX(
      double robotX, AllowedXRange allowedXRange, double marginMeters) {
    double margin = Math.max(0.0, marginMeters);
    return robotX < allowedXRange.minX() - margin || robotX > allowedXRange.maxX() + margin;
  }

  static double computeRecoveryTargetX(
      double robotX, AllowedXRange allowedXRange, double boundaryMarginMeters) {
    double margin = Math.max(0.0, boundaryMarginMeters);
    double minTarget = allowedXRange.minX() + margin;
    double maxTarget = allowedXRange.maxX() - margin;
    if (maxTarget < minTarget) {
      minTarget = allowedXRange.minX();
      maxTarget = allowedXRange.maxX();
    }

    if (robotX < allowedXRange.minX()) {
      return minTarget;
    }
    if (robotX > allowedXRange.maxX()) {
      return maxTarget;
    }
    return MathUtil.clamp(robotX, minTarget, maxTarget);
  }

  static Translation2d computeCruiseVelocity(
      Translation2d robotPosition, Translation2d targetPosition, double cruiseSpeedMetersPerSec) {
    Translation2d delta = targetPosition.minus(robotPosition);
    double magnitude = delta.getNorm();
    if (magnitude < EPSILON) {
      return Translation2d.kZero;
    }

    return delta.div(magnitude).times(Math.max(0.0, cruiseSpeedMetersPerSec));
  }

  static Rotation2d headingOppositeTravelDirection(
      Translation2d travelFieldVelocity, Rotation2d fallbackHeading) {
    if (travelFieldVelocity.getNorm() < EPSILON) {
      return fallbackHeading;
    }
    return travelFieldVelocity.getAngle().plus(Rotation2d.kPi);
  }

  private HeadingControlResult computeHeadingControl(
      Translation2d travelFieldVelocity,
      Rotation2d currentHeading,
      double headingKp,
      double headingKd) {
    Rotation2d headingTarget = headingOppositeTravelDirection(travelFieldVelocity, currentHeading);
    double headingErrorRad =
        MathUtil.angleModulus(headingTarget.minus(currentHeading).getRadians());
    double headingErrorDerivativeRadPerSec =
        (headingErrorRad - lastHeadingErrorRad) / Constants.loopPeriodSecs;
    lastHeadingErrorRad = headingErrorRad;
    double omega =
        Math.max(0.0, headingKp) * headingErrorRad
            + Math.max(0.0, headingKd) * headingErrorDerivativeRadPerSec;
    return new HeadingControlResult(omega, headingErrorRad);
  }

  private Translation2d smoothTarget(Translation2d previous, Translation2d raw, double alpha) {
    double clampedAlpha = MathUtil.clamp(alpha, 0.0, 1.0);
    if (previous == null) {
      return raw;
    }
    return previous.times(1.0 - clampedAlpha).plus(raw.times(clampedAlpha));
  }

  private Translation2d enforceFieldBounds(
      Translation2d desiredFieldVelocity,
      Pose2d robotPose,
      AllowedXRange allowedXRange,
      double marginMeters) {
    double margin = Math.max(0.0, marginMeters);
    double xMin = allowedXRange.minX() + margin;
    double xMax = allowedXRange.maxX() - margin;
    if (xMax < xMin) {
      xMin = allowedXRange.minX();
      xMax = allowedXRange.maxX();
    }

    double yMin = margin;
    double yMax = FieldConstants.fieldWidth - margin;
    if (yMax < yMin) {
      yMin = 0.0;
      yMax = FieldConstants.fieldWidth;
    }

    double vx = desiredFieldVelocity.getX();
    double vy = desiredFieldVelocity.getY();
    double x = robotPose.getX();
    double y = robotPose.getY();

    if (x <= xMin && vx < 0.0) {
      vx = 0.0;
    }
    if (x >= xMax && vx > 0.0) {
      vx = 0.0;
    }
    if (y <= yMin && vy < 0.0) {
      vy = 0.0;
    }
    if (y >= yMax && vy > 0.0) {
      vy = 0.0;
    }

    return new Translation2d(vx, vy);
  }

  private static double slew(double currentValue, double targetValue, double maxStep) {
    double delta = targetValue - currentValue;
    if (delta > maxStep) {
      return currentValue + maxStep;
    }
    if (delta < -maxStep) {
      return currentValue - maxStep;
    }
    return targetValue;
  }
}
