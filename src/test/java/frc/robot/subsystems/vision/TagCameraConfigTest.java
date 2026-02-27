package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class TagCameraConfigTest {
  private static final double EPSILON = 1e-9;

  @Test
  void turretCameraPositionOrbitsAroundAxis() {
    TagCameraConfig atZero =
        VisionConstants.camera0Config.withTurretAngleAtTimestamp(timestamp -> Rotation2d.kZero);
    TagCameraConfig atNinety =
        VisionConstants.camera0Config.withTurretAngleAtTimestamp(
            timestamp -> Rotation2d.fromDegrees(90.0));

    var axis0 = atZero.robotToTurretAxis(0.0).orElseThrow().getTranslation();
    var camera0 = atZero.robotToCamera(0.0).getTranslation();
    var axis90 = atNinety.robotToTurretAxis(0.0).orElseThrow().getTranslation();
    var camera90 = atNinety.robotToCamera(0.0).getTranslation();

    double expectedAxisX = Units.inchesToMeters(0.25);
    double expectedRadius = Units.inchesToMeters(8.532);

    assertEquals(expectedAxisX, axis0.getX(), EPSILON);
    assertEquals(expectedAxisX, axis90.getX(), EPSILON);
    assertEquals(0.0, axis0.getY(), EPSILON);
    assertEquals(0.0, axis90.getY(), EPSILON);

    assertEquals(expectedAxisX + expectedRadius, camera0.getX(), EPSILON);
    assertEquals(0.0, camera0.getY(), EPSILON);

    assertEquals(expectedAxisX, camera90.getX(), EPSILON);
    assertEquals(expectedRadius, camera90.getY(), EPSILON);

    assertEquals(expectedRadius, camera0.getDistance(axis0), EPSILON);
    assertEquals(expectedRadius, camera90.getDistance(axis90), EPSILON);
  }
}
