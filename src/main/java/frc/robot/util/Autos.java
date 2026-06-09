package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public Command RIGHT_MID_DOUBLE() {
    return generateSequenceDouble(true);
  }

  public Command RIGHT_MID_OUTPOST() {
    return generateSequenceSingle(true);
  }

  public Command LEFT_MID_DOUBLE() {
    return generateSequenceDouble(false);
  }

  public Command LEFT_MID_DEPOT() {
    return generateSequenceSingle(false);
  }
  /*
   public Command MIDDLE_OUTPOST() {
     return Commands.sequence(
         AutoBuilder.resetOdom(new Pose2d(3.5, 4, Rotation2d.kZero)),
         Commands.runOnce(
             () ->
                 RobotState.getInstance()
                     .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)),
         Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
         Commands.runOnce(intake::deploy),
         new DriveToPose(
             drive,
             () ->
                 new Pose2d(
                     FieldConstants.Outpost.centerPoint
                         .get()
                         .plus(allianceRelativeXOffset(Units.inchesToMeters(12 + 27.5 / 2))),
                     depotOrOutpostHeading())));
   }

  */

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  private static Rotation2d depotOrOutpostHeading() {
    return isRedAlliance() ? Rotation2d.kPi : Rotation2d.kZero;
  }

  private static Translation2d allianceRelativeXOffset(double offsetMeters) {
    return new Translation2d(isRedAlliance() ? -offsetMeters : offsetMeters, 0.0);
  }

  private static Translation2d allianceRelativeBackupOffset(double offsetMeters) {
    return allianceRelativeXOffset(-offsetMeters);
  }

  private Command generateSequenceSingle(boolean lr) {
    PathPlannerPath pathOne;
    try {
      pathOne = PathPlannerPath.fromChoreoTrajectory(lr ? "RIGHT_OUTPOST" : "LEFT_DEPOT");
    } catch (Exception e) {
      System.out.println("Failed to load path: " + e.getMessage());
      return Commands.print("Auto failed");
    }
    return Commands.sequence(
        AutoBuilder.resetOdom(pathOne.getStartingHolonomicPose().get()),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)),
        Commands.runOnce(intake::deploy),
        Commands.waitUntil(intake::isAtGoal),
        AutoBuilder.followPath(pathOne),
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
                            .plus(allianceRelativeXOffset(Units.inchesToMeters(27.5 / 2 - 9)))
                            .plus(allianceRelativeBackupOffset(Units.inchesToMeters(5.0))),
                        depotOrOutpostHeading())
                    : new Pose2d(
                        FieldConstants.Outpost.centerPoint
                            .get()
                            .plus(allianceRelativeXOffset(Units.inchesToMeters(12 + 27.5 / 2))),
                        depotOrOutpostHeading())),
        Constants.currentMode == Constants.Mode.SIM
            ? Commands.runOnce(() -> superstructure.setFuelSimInventoryCount(24))
            : Commands.none());
  }

  private Command generateSequenceDouble(boolean lr) {
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
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)),
        Commands.runOnce(intake::deploy),
        Commands.waitUntil(intake::isAtGoal),
        AutoBuilder.followPath(pathOne),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(false)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)),
        AutoBuilder.followPath(pathTwo),
        Commands.runOnce(() -> RobotState.getInstance().setAutoEmpty(true)),
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .setTurretShooterRequestedMode(RobotState.TurretShooterMode.SOTM)));
  }
}
