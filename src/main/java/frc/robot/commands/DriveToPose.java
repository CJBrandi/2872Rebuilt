package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber approachDistanceThreshold =
      new LoggedTunableNumber("DriveToPose/ApproachDistanceThreshold");
  private static final LoggedTunableNumber approachVelocityScale =
      new LoggedTunableNumber("DriveToPose/ApproachVelocityScale");

  static {
    drivekP.initDefault(1.8);
    drivekD.initDefault(0.2);
    thetakP.initDefault(3.0);
    thetakD.initDefault(0.1);
    driveMaxVelocity.initDefault(1.0);
    driveMaxAcceleration.initDefault(2.0);
    thetaMaxVelocity.initDefault(Units.degreesToRadians(120));
    thetaMaxAcceleration.initDefault(Units.degreesToRadians(360));
    driveTolerance.initDefault(0.02);
    thetaTolerance.initDefault(Units.degreesToRadians(1.0));
    approachDistanceThreshold.initDefault(0.05);
    approachVelocityScale.initDefault(1);
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double previousDriveVelocityScalar = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    updateTunables(true);
    Logger.recordOutput("DriveToPose/Target", target.get());
    Pose2d currentPose = RobotState.getInstance().getRobotPose();
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getRobotVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    double initialDistance =
        currentPose.getTranslation().getDistance(target.get().getTranslation());
    double initialApproachVelocity =
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX());
    driveController.reset(initialDistance, initialApproachVelocity);
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    previousDriveVelocityScalar =
        MathUtil.clamp(initialApproachVelocity, -driveMaxVelocity.get(), driveMaxVelocity.get());
  }

  @Override
  public void execute() {
    running = true;

    updateTunables(false);

    // Get current pose and target pose
    Pose2d currentPose = RobotState.getInstance().getRobotPose();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);

    // Apply velocity scaling for gentle approach
    boolean inApproachZone = currentDistance < approachDistanceThreshold.get();
    if (inApproachZone) {
      // Scale velocity based on distance to target
      double approachFactor =
          MathUtil.clamp(currentDistance / approachDistanceThreshold.get(), 0.1, 1.0);
      driveVelocityScalar *= approachFactor * approachVelocityScale.get();
    }

    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    // Enforce velocity and acceleration limits on the commanded linear speed.
    driveVelocityScalar =
        MathUtil.clamp(driveVelocityScalar, -driveMaxVelocity.get(), driveMaxVelocity.get());
    double maxDeltaVelocity = driveMaxAcceleration.get() * Constants.loopPeriodSecs;
    driveVelocityScalar =
        MathUtil.clamp(
            driveVelocityScalar,
            previousDriveVelocityScalar - maxDeltaVelocity,
            previousDriveVelocityScalar + maxDeltaVelocity);
    previousDriveVelocityScalar = driveVelocityScalar;

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed - no additional slowing based on proximity
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Command speeds
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/VelocitySetpoint", driveController.getSetpoint().velocity);
    Logger.recordOutput("DriveToPose/VelocityCommand", driveVelocityScalar);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/InApproachZone", inApproachZone);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(thetaController.getSetpoint().position)));
    Logger.recordOutput("DriveToPose/Goal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d());
    Logger.recordOutput("DriveToPose/Goal", new Pose2d());
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  private void updateTunables(boolean force) {
    if (force
        || driveMaxVelocity.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())
        || approachDistanceThreshold.hasChanged(hashCode())
        || approachVelocityScale.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }
  }
}
