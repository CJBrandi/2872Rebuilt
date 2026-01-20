package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class TurretVisualizer {
  private static final double TURRET_HEIGHT = 0.5;
  private static final double LINE_LENGTH = 2.0;

  private TurretVisualizer() {}

  /**
   * Logs visualization data for the turret direction and aim line.
   *
   * @param turretAngleRad The current turret angle in radians (robot-relative)
   */
  public static void update(double turretAngleRad) {
    Pose2d robotPose = RobotState.getInstance().getRobotPose();
    double robotAngleRad = robotPose.getRotation().getRadians();
    double turretFieldAngle = turretAngleRad + robotAngleRad;

    Pose3d robotPose3d =
        new Pose3d(robotPose.getX(), robotPose.getY(), TURRET_HEIGHT, new Rotation3d());

    // Turret direction line
    Pose3d turretEnd =
        new Pose3d(
            robotPose.getX() + LINE_LENGTH * Math.cos(turretFieldAngle),
            robotPose.getY() + LINE_LENGTH * Math.sin(turretFieldAngle),
            TURRET_HEIGHT,
            new Rotation3d(0, 0, turretFieldAngle));
    Logger.recordOutput("Turret/Visualization/Direction", robotPose3d, turretEnd);
  }
}
