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
  void absoluteAngleSelectionClampsWithoutWrapping() {
    TurretLimits.Selection insideRange =
        TurretLimits.selectAbsoluteAngleRadians(Rotation2d.fromDegrees(-300.0).getRadians());
    TurretLimits.Selection upperClamp =
        TurretLimits.selectAbsoluteAngleRadians(Rotation2d.fromDegrees(-100.0).getRadians());
    TurretLimits.Selection lowerClamp =
        TurretLimits.selectAbsoluteAngleRadians(Rotation2d.fromDegrees(-600.0).getRadians());
    TurretLimits.Selection lowerBoundary =
        TurretLimits.selectAbsoluteAngleRadians(Rotation2d.fromDegrees(-540.0).getRadians());

    assertAll(
        () -> assertEquals(-300.0, insideRange.selectedDeg(), EPSILON),
        () -> assertEquals(-180.0, upperClamp.selectedDeg(), EPSILON),
        () -> assertEquals(-540.0, lowerClamp.selectedDeg(), EPSILON),
        () -> assertEquals(-540.0, lowerBoundary.selectedDeg(), EPSILON),
        () -> assertTrue(upperClamp.clamped()),
        () -> assertTrue(lowerClamp.clamped()),
        () -> assertFalse(insideRange.clamped()),
        () -> assertFalse(lowerBoundary.clamped()));
  }

  @Test
  void fieldRelativeSelectionMapsIntoNegativeOnlyRevolution() {
    TurretLimits.Selection zeroDegrees =
        TurretLimits.selectFieldRelativeAngleRadians(Rotation2d.fromDegrees(0.0).getRadians());
    TurretLimits.Selection plusTwentyFive =
        TurretLimits.selectFieldRelativeAngleRadians(Rotation2d.fromDegrees(25.0).getRadians());
    TurretLimits.Selection minusOneHundred =
        TurretLimits.selectFieldRelativeAngleRadians(Rotation2d.fromDegrees(-100.0).getRadians());
    TurretLimits.Selection minusFourHundredFifty =
        TurretLimits.selectFieldRelativeAngleRadians(Rotation2d.fromDegrees(-450.0).getRadians());

    assertAll(
        () -> assertEquals(-360.0, zeroDegrees.selectedDeg(), EPSILON),
        () -> assertEquals(-335.0, plusTwentyFive.selectedDeg(), EPSILON),
        () -> assertEquals(-460.0, minusOneHundred.selectedDeg(), EPSILON),
        () -> assertEquals(-450.0, minusFourHundredFifty.selectedDeg(), EPSILON),
        () -> assertFalse(zeroDegrees.clamped()),
        () -> assertFalse(plusTwentyFive.clamped()),
        () -> assertFalse(minusOneHundred.clamped()),
        () -> assertFalse(minusFourHundredFifty.clamped()));
  }

  @Test
  void fieldRelativeBackAngleUsesUpperHardStop() {
    TurretLimits.Selection backAngle =
        TurretLimits.selectFieldRelativeAngleRadians(Rotation2d.fromDegrees(180.0).getRadians());

    assertAll(
        () -> assertEquals(-180.0, backAngle.selectedDeg(), EPSILON),
        () -> assertFalse(backAngle.clamped()));
  }
}
