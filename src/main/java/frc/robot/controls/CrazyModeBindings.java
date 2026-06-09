package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.FieldConstants;

public final class CrazyModeBindings {
  private CrazyModeBindings() {}

  public static void configure(
      CommandXboxController driver,
      Drive drive,
      Intake intake,
      Superstructure superstructure,
      Command orchestraCommand) {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver
        .leftTrigger(0.5)
        .whileTrue(
            Commands.startEnd(
                intake::deploy,
                () -> {
                  intake.getRoller().stop();
                },
                intake,
                intake.getRoller()));

    driver.rightBumper().onTrue(Commands.runOnce(intake::stow, intake));

    driver
        .rightTrigger(0.5)
        .whileTrue(
            Commands.startEnd(
                    () -> RobotState.getInstance().setAutoEmpty(true),
                    () -> RobotState.getInstance().setAutoEmpty(false))
                .ignoringDisable(true));

    driver
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> {
                  Translation2d robotToHub =
                      FieldConstants.Hub.topCenterPoint
                          .get()
                          .toTranslation2d()
                          .minus(drive.getPose().getTranslation());
                  return new Rotation2d(robotToHub.getX(), robotToHub.getY())
                      .rotateBy(Rotation2d.kPi);
                }));

    driver
        .b()
        .and(driver.start().negate())
        .whileTrue(
            Commands.startEnd(
                () -> intake.getRoller().setTemporaryVelocityOverride(Roller.ejectVelocity.get()),
                intake.getRoller()::clearTemporaryVelocityOverride,
                intake.getRoller()));

    driver
        .start()
        .and(driver.b())
        .whileTrue(
            Commands.startEnd(
                () -> superstructure.getIndexer().runVolts(-2.0),
                superstructure.getIndexer()::stop));

    bindOrchestraCommand(driver.start().and(driver.back()), orchestraCommand);
  }

  static void bindOrchestraCommand(Trigger songButtons, Command orchestraCommand) {
    songButtons.whileTrue(orchestraCommand);
  }
}
