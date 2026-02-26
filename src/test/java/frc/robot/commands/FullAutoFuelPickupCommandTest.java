package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.fuelpickup.FuelPickupZonePolicy.AllowedXRange;
import org.junit.jupiter.api.Test;

class FullAutoFuelPickupCommandTest {
  private static final double EPSILON = 1e-9;

  @Test
  void targetInAllowedZoneUsesChaseModeAndCruiseVector() {
    var mode = FullAutoFuelPickupCommand.selectMode(false, true);
    var velocity =
        FullAutoFuelPickupCommand.computeCruiseVelocity(
            new Translation2d(1.0, 1.0), new Translation2d(3.0, 1.0), 2.0);

    assertEquals(FullAutoFuelPickupCommand.AutoMode.CHASE, mode);
    assertEquals(2.0, velocity.getX(), EPSILON);
    assertEquals(0.0, velocity.getY(), EPSILON);
  }

  @Test
  void noAllowedTargetTransitionsToSearchMode() {
    var mode = FullAutoFuelPickupCommand.selectMode(false, false);
    var velocity =
        FullAutoFuelPickupCommand.computeCruiseVelocity(
            new Translation2d(2.0, 2.0), new Translation2d(2.0, 2.0), 2.0);

    assertEquals(FullAutoFuelPickupCommand.AutoMode.SEARCH, mode);
    assertEquals(0.0, velocity.getNorm(), EPSILON);
  }

  @Test
  void robotOutsideAllowedRangeTransitionsToRecoverMode() {
    AllowedXRange range = new AllowedXRange(5.0, 10.0);
    var mode = FullAutoFuelPickupCommand.selectMode(true, true);
    double leftTarget = FullAutoFuelPickupCommand.computeRecoveryTargetX(3.0, range, 0.05);
    double rightTarget = FullAutoFuelPickupCommand.computeRecoveryTargetX(12.0, range, 0.05);

    assertEquals(FullAutoFuelPickupCommand.AutoMode.RECOVER, mode);
    assertEquals(5.05, leftTarget, EPSILON);
    assertEquals(9.95, rightTarget, EPSILON);
  }

  @Test
  void headingTargetIsOppositeTravelDirection() {
    Rotation2d heading =
        FullAutoFuelPickupCommand.headingOppositeTravelDirection(
            new Translation2d(1.0, 0.0), Rotation2d.kZero);
    assertEquals(Math.PI, heading.getRadians(), EPSILON);
  }

  @Test
  void zeroTravelUsesFallbackHeading() {
    Rotation2d fallback = Rotation2d.fromDegrees(27.0);
    Rotation2d heading =
        FullAutoFuelPickupCommand.headingOppositeTravelDirection(Translation2d.kZero, fallback);
    assertEquals(fallback.getRadians(), heading.getRadians(), EPSILON);
  }
}
