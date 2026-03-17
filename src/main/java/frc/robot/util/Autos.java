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

  public Command RIGHT_MID_OUTPOST() {
    return generateSequence(true);
  }

  public Command LEFT_MID_DEPOT() {
    return generateSequence(false);
  }

  private Command generateSequence(boolean lr) {
    PathPlannerPath pathOne;
    PathPlannerPath pathTwo;
    try {
      pathOne = PathPlannerPath.fromChoreoTrajectory(lr ? "RIGHT_ONE" : "LEFT_ONE");
      pathTwo = PathPlannerPath.fromChoreoTrajectory(lr ? "RIGHT_TWO" : "LEFT_TWO");
    } catch (Exception e) {
      System.out.println("Failed to load path: " + e.getMessage());
      return Commands.print("Auto failed");
    }
    return Commands.sequence(
        AutoBuilder.resetOdom(pathOne.getStartingHolonomicPose().get()),
        Commands.runOnce(() -> RobotState.getInstance().setStrictPoseEstimation(true)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.AIM)),
        Commands.runOnce(intake::deploy),
        Commands.waitUntil(intake::isAtGoal),
        AutoBuilder.followPath(pathOne),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
        Commands.waitSeconds(5),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(false)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.AIM)),
        AutoBuilder.followPath(pathTwo),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)),
        new DriveToPose(
            drive,
            () ->
                !lr
                    ? new Pose2d(
                        FieldConstants.Depot.depotCenter
                            .get()
                            .toTranslation2d()
                            .minus(new Translation2d(Units.inchesToMeters(27.5 / 2 - 9), 0)),
                        Rotation2d.fromDegrees(180))
                    : new Pose2d(
                        FieldConstants.Outpost.centerPoint
                            .get()
                            .minus(new Translation2d(Units.inchesToMeters(12 + 27.5 / 2), 0)),
                        Rotation2d.fromDegrees(180))),
        Constants.currentMode == Constants.Mode.SIM
            ? Commands.runOnce(() -> superstructure.setFuelSimInventoryCount(24))
            : Commands.none());
  }
}
