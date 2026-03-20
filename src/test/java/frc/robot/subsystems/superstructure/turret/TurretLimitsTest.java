package frc.robot.subsystems.superstructure.turret;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class TurretLimitsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void limitsAreSortedBeforeUse() {
    assertAll(
        () -> assertTrue(TurretLimits.MIN_ANGLE_DEG < TurretLimits.MAX_ANGLE_DEG),
        () -> assertTrue(TurretLimits.MIN_ANGLE_RAD < TurretLimits.MAX_ANGLE_RAD));
  }

  @Test
  void absoluteAngleSelectionTakesDirectPathWithinLimits() {
    TurretLimits.Selection insideRange =
        TurretLimits.selectAbsoluteAngleRadians(
            Rotation2d.fromDegrees(-150.0).getRadians(),
            Rotation2d.fromDegrees(-180.0).getRadians());

    assertAll(
        () -> assertEquals(-150.0, insideRange.selectedDeg(), EPSILON),
        () -> assertEquals(-150.0, insideRange.candidateDeg(), EPSILON),
        () -> assertFalse(insideRange.clamped()),
        () -> assertEquals(-180.0, insideRange.referenceDeg(), EPSILON));
  }

  @Test
  void fieldRelativeSelectionUnwrapsToClosestLegalEquivalent() {
    TurretLimits.Selection wrappedTarget =
        TurretLimits.selectFieldRelativeAngleRadians(
            Rotation2d.fromDegrees(210.0).getRadians(),
            Rotation2d.fromDegrees(-180.0).getRadians());

    assertAll(
        () -> assertEquals(-150.0, wrappedTarget.selectedDeg(), EPSILON),
        () -> assertEquals(-150.0, wrappedTarget.candidateDeg(), EPSILON),
        () -> assertFalse(wrappedTarget.clamped()));
  }

  @Test
  void selectionPrefersTheEquivalentClosestToTheCurrentTurretAngle() {
    TurretLimits.Selection nearNegativeWrap =
        TurretLimits.selectFieldRelativeAngleRadians(
            Rotation2d.fromDegrees(10.0).getRadians(), Rotation2d.fromDegrees(-340.0).getRadians());
    TurretLimits.Selection nearPositiveWrap =
        TurretLimits.selectFieldRelativeAngleRadians(
            Rotation2d.fromDegrees(10.0).getRadians(), Rotation2d.fromDegrees(0.0).getRadians());

    assertAll(
        () -> assertEquals(-350.0, nearNegativeWrap.selectedDeg(), EPSILON),
        () -> assertEquals(10.0, nearPositiveWrap.selectedDeg(), EPSILON),
        () -> assertFalse(nearNegativeWrap.clamped()),
        () -> assertFalse(nearPositiveWrap.clamped()));
  }

  @Test
  void clampAndVelocityChecksHonorTheConfiguredWindow() {
    assertAll(
        () ->
            assertEquals(TurretLimits.MIN_ANGLE_DEG, TurretLimits.clampDegrees(-1_000.0), EPSILON),
        () -> assertEquals(TurretLimits.MAX_ANGLE_DEG, TurretLimits.clampDegrees(1_000.0), EPSILON),
        () -> assertTrue(TurretLimits.commandWouldPushPastLimit(TurretLimits.MAX_ANGLE_RAD, 0.1)),
        () -> assertTrue(TurretLimits.commandWouldPushPastLimit(TurretLimits.MIN_ANGLE_RAD, -0.1)),
        () -> assertFalse(TurretLimits.commandWouldPushPastLimit(TurretLimits.MAX_ANGLE_RAD, -0.1)),
        () -> assertFalse(TurretLimits.commandWouldPushPastLimit(TurretLimits.MIN_ANGLE_RAD, 0.1)));
  }
}
