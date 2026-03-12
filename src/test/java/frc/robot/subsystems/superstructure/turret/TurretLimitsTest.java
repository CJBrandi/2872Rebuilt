package frc.robot.subsystems.superstructure.turret;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class TurretLimitsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void selectAngleSupportsAllowedNegativeAngles() {
    double startReferenceRad = Rotation2d.fromDegrees(-180.0).getRadians();

    TurretLimits.Selection angleMinus300 =
        TurretLimits.selectAngleRadians(
            Rotation2d.fromDegrees(-300.0).getRadians(), startReferenceRad);
    TurretLimits.Selection angleMinus200 =
        TurretLimits.selectAngleRadians(
            Rotation2d.fromDegrees(-200.0).getRadians(), startReferenceRad);
    TurretLimits.Selection angleMinus100 =
        TurretLimits.selectAngleRadians(
            Rotation2d.fromDegrees(-100.0).getRadians(), startReferenceRad);

    assertAll(
        () -> assertEquals(-300.0, angleMinus300.selectedDeg(), EPSILON),
        () -> assertEquals(-200.0, angleMinus200.selectedDeg(), EPSILON),
        () -> assertEquals(-100.0, angleMinus100.selectedDeg(), EPSILON),
        () -> assertFalse(angleMinus300.usedLimitFallback()),
        () -> assertFalse(angleMinus200.usedLimitFallback()),
        () -> assertFalse(angleMinus100.usedLimitFallback()));
  }

  @Test
  void selectAngleChoosesClosestWrappedEquivalentFromReference() {
    TurretLimits.Selection nearNegativeReference =
        TurretLimits.selectAngleRadians(
            Rotation2d.fromDegrees(25.0).getRadians(), Rotation2d.fromDegrees(-180.0).getRadians());
    TurretLimits.Selection nearPositiveReference =
        TurretLimits.selectAngleRadians(
            Rotation2d.fromDegrees(25.0).getRadians(), Rotation2d.fromDegrees(10.0).getRadians());

    assertAll(
        () -> assertEquals(-335.0, nearNegativeReference.selectedDeg(), EPSILON),
        () -> assertEquals(25.0, nearPositiveReference.selectedDeg(), EPSILON),
        () -> assertFalse(nearNegativeReference.usedLimitFallback()),
        () -> assertFalse(nearPositiveReference.usedLimitFallback()));
  }
}
