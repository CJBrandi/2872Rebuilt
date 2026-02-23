package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class ShotVectorCompensatorTest {
  private static final double EPSILON = 1e-9;

  @Test
  void compensatesUsingFieldRelativeLinearVelocity() {
    Translation3d requiredFieldVelocity = new Translation3d(10.0, 0.0, 5.0);
    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    Rotation2d robotHeading = Rotation2d.fromDegrees(90.0);

    var shot =
        ShotVectorCompensator.compensateForRobotMotion(
            requiredFieldVelocity, robotRelativeSpeeds, robotHeading, new Translation2d());

    assertEquals(10.0, shot.shooterRelativeVelocity().getX(), EPSILON);
    assertEquals(-1.0, shot.shooterRelativeVelocity().getY(), EPSILON);
    assertEquals(5.0, shot.shooterRelativeVelocity().getZ(), EPSILON);
    assertEquals(
        Math.hypot(10.0, -1.0),
        Math.hypot(shot.shooterRelativeVelocity().getX(), shot.shooterRelativeVelocity().getY()),
        EPSILON);
  }

  @Test
  void compensatesForTangentialVelocityFromRotation() {
    Translation3d requiredFieldVelocity = new Translation3d(8.0, 0.0, 4.0);
    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(0.0, 0.0, 2.0);
    Translation2d robotToShooter = new Translation2d(0.5, 0.0);

    var shot =
        ShotVectorCompensator.compensateForRobotMotion(
            requiredFieldVelocity, robotRelativeSpeeds, Rotation2d.kZero, robotToShooter);

    assertEquals(8.0, shot.shooterRelativeVelocity().getX(), EPSILON);
    assertEquals(-1.0, shot.shooterRelativeVelocity().getY(), EPSILON);
    assertEquals(4.0, shot.shooterRelativeVelocity().getZ(), EPSILON);
  }

  @Test
  void convertsVectorToYawPitchAndSpeed() {
    Translation3d shooterVector = new Translation3d(3.0, 4.0, 5.0);

    var shot = ShotVectorCompensator.fromShooterRelativeVector(shooterVector);

    assertEquals(Math.sqrt(50.0), shot.exitVelocity(), EPSILON);
    assertEquals(Math.atan2(5.0, 5.0), shot.pitch(), EPSILON);
    assertEquals(Math.atan2(4.0, 3.0), shot.yaw().getRadians(), EPSILON);
  }
}
