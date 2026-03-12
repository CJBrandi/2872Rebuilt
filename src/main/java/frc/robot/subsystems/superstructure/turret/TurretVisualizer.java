package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class TurretVisualizer {
  private static final double LINE_LENGTH = 2.0;

  private TurretVisualizer() {}

  /**
   * Logs visualization data for the turret direction and aim line.
   *
   * @param turretAngleRad The current turret angle in radians (robot-relative)
   */
  public static void update(double turretAngleRad) {
    Pose2d robotPose = RobotState.getInstance().getRobotPose();
    Pose3d turretRobotRelativePose = getTurretRobotRelativePose(turretAngleRad);
    Pose3d turretFieldPose =
        new Pose3d(robotPose)
            .plus(
                new Transform3d(
                    turretRobotRelativePose.getTranslation(),
                    turretRobotRelativePose.getRotation()));

    // Turret direction line
    Pose3d turretEnd =
        turretFieldPose.plus(
            new Transform3d(new Translation3d(LINE_LENGTH, 0.0, 0.0), new Rotation3d()));
    Logger.recordOutput("Turret/Visualization/Direction", turretFieldPose, turretEnd);
    Logger.recordOutput(
        "Turret/Visualization/ComponentPoses", new Pose3d[] {turretRobotRelativePose});
  }

  private static Pose3d getTurretRobotRelativePose(double turretAngleRad) {
    return new Pose3d()
        .plus(new Transform3d(VisionConstants.robotCenterToTurretAxisAtZero, new Rotation3d()))
        .plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, turretAngleRad)));
  }
}
