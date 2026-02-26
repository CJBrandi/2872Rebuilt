package frc.robot.subsystems.fuelpickup;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldConstants;
import org.junit.jupiter.api.Test;

class FuelPickupZonePolicyTest {
  private static final double EPSILON = 1e-9;

  @Test
  void blueOwnZoneRangeWhenShiftActive() {
    var range = FuelPickupZonePolicy.getAllowedXRange(Alliance.Blue, true);
    assertEquals(0.0, range.minX(), EPSILON);
    assertEquals(FieldConstants.LinesVertical.neutralZoneNear, range.maxX(), EPSILON);
  }

  @Test
  void blueMiddlePlusOpponentRangeWhenShiftInactive() {
    var range = FuelPickupZonePolicy.getAllowedXRange(Alliance.Blue, false);
    assertEquals(FieldConstants.LinesVertical.neutralZoneNear, range.minX(), EPSILON);
    assertEquals(FieldConstants.fieldLength, range.maxX(), EPSILON);
  }

  @Test
  void redOwnZoneRangeWhenShiftActive() {
    var range = FuelPickupZonePolicy.getAllowedXRange(Alliance.Red, true);
    assertEquals(FieldConstants.LinesVertical.neutralZoneFar, range.minX(), EPSILON);
    assertEquals(FieldConstants.fieldLength, range.maxX(), EPSILON);
  }

  @Test
  void redMiddlePlusOpponentRangeWhenShiftInactive() {
    var range = FuelPickupZonePolicy.getAllowedXRange(Alliance.Red, false);
    assertEquals(0.0, range.minX(), EPSILON);
    assertEquals(FieldConstants.LinesVertical.neutralZoneFar, range.maxX(), EPSILON);
  }

  @Test
  void clusterAllowedChecksRangeBoundaries() {
    var range = FuelPickupZonePolicy.getAllowedXRange(Alliance.Blue, false);
    assertTrue(FuelPickupZonePolicy.isClusterAllowedX(range.minX(), range));
    assertTrue(FuelPickupZonePolicy.isClusterAllowedX(range.maxX(), range));
    assertFalse(FuelPickupZonePolicy.isClusterAllowedX(range.minX() - 1e-3, range));
    assertFalse(FuelPickupZonePolicy.isClusterAllowedX(range.maxX() + 1e-3, range));
  }
}
