package frc.robot.subsystems.superstructure.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class TurretLimitsTest {
  @Test
  void keepsAnglesAlreadyInsideRange() {
    double selected =
        Units.radiansToDegrees(
            TurretLimits.findBestAngleWithinLimitsRadians(
                Units.degreesToRadians(-90.0), Units.degreesToRadians(-100.0)));

    assertEquals(-90.0, selected, 1e-9);
  }

  @Test
  void wrapsEquivalentPositiveRequestIntoNegativeRange() {
    double selected =
        Units.radiansToDegrees(
            TurretLimits.findBestAngleWithinLimitsRadians(
                Units.degreesToRadians(160.0), Units.degreesToRadians(-180.0)));

    assertEquals(-200.0, selected, 1e-9);
  }

  @Test
  void clampsUnreachableRequestsToNearestLimit() {
    double selected =
        Units.radiansToDegrees(
            TurretLimits.findBestAngleWithinLimitsRadians(
                Units.degreesToRadians(20.0), Units.degreesToRadians(-20.0)));

    assertEquals(0.0, selected, 1e-9);
  }

  @Test
  void clampsAnglesToConfiguredRange() {
    assertEquals(-200.0, TurretLimits.clampDegrees(-250.0), 1e-9);
    assertEquals(0.0, TurretLimits.clampDegrees(25.0), 1e-9);
  }

  @Test
  void blocksOnlyOutwardCommandsAtHardStops() {
    assertTrue(TurretLimits.commandWouldPushPastLimit(Units.degreesToRadians(0.0), 1.0));
    assertTrue(TurretLimits.commandWouldPushPastLimit(Units.degreesToRadians(-200.0), -1.0));
    assertFalse(TurretLimits.commandWouldPushPastLimit(Units.degreesToRadians(-200.0), 1.0));
    assertFalse(TurretLimits.commandWouldPushPastLimit(Units.degreesToRadians(-100.0), -1.0));
  }
}
