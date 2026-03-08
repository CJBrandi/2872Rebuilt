package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
  @Test
  void mirroredRedAlliancePosesSelectSameShotMode() {
    assertMirroredPoseSelectsSameShotMode(
        new Pose2d(4.0, 3.0, Rotation2d.kZero), ShotCalculator.ShotMode.HUB);
    assertMirroredPoseSelectsSameShotMode(
        new Pose2d(5.4, 5.1, Rotation2d.kZero), ShotCalculator.ShotMode.LOB_LEFT);
    assertMirroredPoseSelectsSameShotMode(
        new Pose2d(5.4, 3.1, Rotation2d.kZero), ShotCalculator.ShotMode.LOB_RIGHT);
  }

  private static void assertMirroredPoseSelectsSameShotMode(
      Pose2d blueAlliancePose, ShotCalculator.ShotMode expectedMode) {
    Pose2d mirroredRedAlliancePose =
        new Pose2d(
            FieldConstants.fieldLength - blueAlliancePose.getX(),
            FieldConstants.fieldWidth - blueAlliancePose.getY(),
            blueAlliancePose.getRotation().rotateBy(Rotation2d.kPi));

    assertEquals(expectedMode, ShotCalculator.selectShotMode(blueAlliancePose, false));
    assertEquals(expectedMode, ShotCalculator.selectShotMode(mirroredRedAlliancePose, true));
  }
}
