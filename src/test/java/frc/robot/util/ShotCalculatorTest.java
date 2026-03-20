package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
  private static final double EPSILON = 1e-9;

  @Test
  void selectAllianceLobTargetUsesHubToCornerMidpointsForBlueAlliance() {
    Translation3d hub = FieldConstants.Hub.topCenterPoint.getBlue();
    Translation2d expectedLeft =
        midpoint(hub.toTranslation2d(), new Translation2d(0.0, FieldConstants.fieldWidth));
    Translation2d expectedRight = midpoint(hub.toTranslation2d(), new Translation2d(0.0, 0.0));

    assertAll(
        () ->
            assertTranslationEquals(
                expectedLeft, ShotCalculator.selectAllianceLobTarget(true, false)),
        () ->
            assertTranslationEquals(
                expectedRight, ShotCalculator.selectAllianceLobTarget(false, false)));
  }

  @Test
  void selectAllianceLobTargetFlipsToOppositeAllianceCorners() {
    Translation2d blueLeft = ShotCalculator.selectAllianceLobTarget(true, false);
    Translation2d blueRight = ShotCalculator.selectAllianceLobTarget(false, false);

    Translation2d expectedRedLeft =
        new Translation2d(
            FieldConstants.fieldLength - blueLeft.getX(),
            FieldConstants.fieldWidth - blueLeft.getY());
    Translation2d expectedRedRight =
        new Translation2d(
            FieldConstants.fieldLength - blueRight.getX(),
            FieldConstants.fieldWidth - blueRight.getY());

    assertAll(
        () ->
            assertTranslationEquals(
                expectedRedLeft, ShotCalculator.selectAllianceLobTarget(true, true)),
        () ->
            assertTranslationEquals(
                expectedRedRight, ShotCalculator.selectAllianceLobTarget(false, true)));
  }

  @Test
  void applyLobScalingHalvesPitchAndExitVelocity() {
    ShotCalculator.ShootingParameters rawParameters =
        new ShotCalculator.ShootingParameters(Rotation2d.fromDegrees(15.0), 1.2, 0.8, 0.4, 18.0);

    ShotCalculator.ShootingParameters scaledParameters =
        ShotCalculator.applyLobScaling(rawParameters);

    assertAll(
        () ->
            assertEquals(
                rawParameters.turretAngle().getRadians(),
                scaledParameters.turretAngle().getRadians(),
                EPSILON),
        () ->
            assertEquals(
                rawParameters.turretVelocity(), scaledParameters.turretVelocity(), EPSILON),
        () ->
            assertEquals(rawParameters.pitchAngle() * 0.5, scaledParameters.pitchAngle(), EPSILON),
        () ->
            assertEquals(
                rawParameters.pitchVelocity() * 0.5, scaledParameters.pitchVelocity(), EPSILON),
        () ->
            assertEquals(
                rawParameters.exitVelocity() * 0.5, scaledParameters.exitVelocity(), EPSILON));
  }

  private static Translation2d midpoint(Translation2d first, Translation2d second) {
    return new Translation2d(
        (first.getX() + second.getX()) / 2.0, (first.getY() + second.getY()) / 2.0);
  }

  private static void assertTranslationEquals(Translation2d expected, Translation2d actual) {
    assertAll(
        () -> assertEquals(expected.getX(), actual.getX(), EPSILON),
        () -> assertEquals(expected.getY(), actual.getY(), EPSILON));
  }
}
