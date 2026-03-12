package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autos {
  private final Drive drive;
  private final Intake intake;
  private final Superstructure superstructure;

  public Autos(Drive drive, Intake intake, Superstructure superstructure) {
    this.drive = drive;
    this.intake = intake;
    this.superstructure = superstructure;
  }

  public Command TEST() {
    PathPlannerPath cycle;
    try {
      cycle = PathPlannerPath.fromChoreoTrajectory("MTEST");
    } catch (Exception e) {
      System.out.println("Failed to load path: " + e.getMessage());
      return Commands.runOnce(
          () ->
              RobotState.getInstance()
                  .setTurretShooterRequestedMode(RobotState.TurretShooterMode.MANUAL));
    }
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.MANUAL)),
        AutoBuilder.resetOdom(cycle.getStartingHolonomicPose().get()),
        AutoBuilder.followPath(cycle));
  }

  public Command RIGHT_MID_DOUBLE() {
    PathPlannerPath cycle;
    try {
      cycle = PathPlannerPath.fromChoreoTrajectory("RIGHT_MID_DOUBLE");
    } catch (Exception e) {
      System.out.println("Failed to load path: " + e.getMessage());
      return Commands.none();
    }

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.MANUAL)),
        Commands.runOnce(() -> RobotState.getInstance().setStrictPoseEstimation(true)),
        AutoBuilder.resetOdom(cycle.getStartingHolonomicPose().get()),
        Commands.runOnce(intake::deploy),
        Commands.runOnce(
            () -> superstructure.getShooter().setGoals(7.76, Units.degreesToRadians(23.6))),
        Commands.runOnce(
            () ->
                superstructure
                    .getTurret()
                    .setTargetFieldRelativeAngle(Rotation2d.fromDegrees(-101), 0)),
        AutoBuilder.followPath(cycle),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(false)),
        AutoBuilder.followPath(cycle),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)));
  }

  public Command RIGHT_MID_OUTPOST_CLIMB() {
    PathPlannerPath cycle;
    try {
      cycle = PathPlannerPath.fromChoreoTrajectory("RIGHT_MID_OUTPOST");
    } catch (Exception e) {
      System.out.println("Failed to load path: " + e.getMessage());
      return Commands.print("Auto failed");
    }
    return Commands.sequence(
        Constants.currentMode == Constants.Mode.SIM
            ? Commands.runOnce(() -> superstructure.setFuelSimInventoryCount(24))
            : Commands.none(),
        AutoBuilder.resetOdom(cycle.getStartingHolonomicPose().get()),
        Commands.runOnce(() -> RobotState.getInstance().setStrictPoseEstimation(true)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.AIM)),
        Commands.runOnce(intake::deploy),
        AutoBuilder.followPath(cycle),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)),
        new DriveToPose(
                drive,
                () ->
                    new Pose2d(
                        FieldConstants.Outpost.centerPoint
                            .get()
                            .minus(new Translation2d(Units.inchesToMeters(12 + 27.5 / 2), 0)),
                        Rotation2d.fromDegrees(180)))
            .withTimeout(8),
        new DriveToPose(
            drive,
            () ->
                new Pose2d(
                    FieldConstants.Tower.rightUpright
                        .get()
                        .minus(
                            new Translation2d(
                                -Units.inchesToMeters(14),
                                Units.inchesToMeters(-5.875 - 27.5 / 2))),
                    Rotation2d.fromDegrees(-90))));
  }
}
