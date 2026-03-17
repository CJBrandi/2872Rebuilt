package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.FieldConstants;
import org.junit.jupiter.api.Test;

class SuperstructureTest {
  private static final double EPSILON = 1e-9;
  private static final double INTAKE_EXTENSION_METERS = Units.inchesToMeters(12.0);
  private static final double TRENCH_SAFETY_ZONE_METERS = Units.inchesToMeters(6.0);

  @Test
  void rearIntakeTipTranslationMovesToTheBackOfTheRobot() {
    Translation2d robotTranslation = new Translation2d(5.0, 3.0);
    double rearTipDistance = Constants.RobotDimensions.length / 2.0 + INTAKE_EXTENSION_METERS;

    Translation2d facingPositiveX =
        Superstructure.getRearIntakeTipTranslation(
            robotTranslation,
            Rotation2d.kZero,
            Constants.RobotDimensions.length,
            INTAKE_EXTENSION_METERS);
    Translation2d facingNegativeX =
        Superstructure.getRearIntakeTipTranslation(
            robotTranslation,
            Rotation2d.kPi,
            Constants.RobotDimensions.length,
            INTAKE_EXTENSION_METERS);

    assertAll(
        () ->
            assertEquals(
                robotTranslation.getX() - rearTipDistance, facingPositiveX.getX(), EPSILON),
        () -> assertEquals(robotTranslation.getY(), facingPositiveX.getY(), EPSILON),
        () ->
            assertEquals(
                robotTranslation.getX() + rearTipDistance, facingNegativeX.getX(), EPSILON),
        () -> assertEquals(robotTranslation.getY(), facingNegativeX.getY(), EPSILON));
  }

  @Test
  void rearMountedIntakeTriggersEarlyWhenVelocityAndLookaheadWouldCarryItIntoTrench() {
    double trenchX = FieldConstants.LinesVertical.hubCenter;
    double trenchY =
        (FieldConstants.LinesHorizontal.rightTrenchOpenStart
                + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
            / 2.0;
    double rearTipDistance = Constants.RobotDimensions.length / 2.0 + INTAKE_EXTENSION_METERS;
    Translation2d currentTranslation =
        new Translation2d(trenchX + rearTipDistance + Units.inchesToMeters(10.0), trenchY);
    ChassisSpeeds backingTowardTrench = new ChassisSpeeds(-Units.inchesToMeters(24.0), 0.0, 0.0);

    assertTrue(
        Superstructure.shouldDeployRearIntakeForTrench(
            currentTranslation,
            Rotation2d.kZero,
            backingTowardTrench,
            0.5,
            Constants.RobotDimensions.length,
            INTAKE_EXTENSION_METERS,
            TRENCH_SAFETY_ZONE_METERS));
  }

  @Test
  void rearMountedIntakeDoesNotTriggerWithoutEnoughLookaheadAtCurrentVelocity() {
    double trenchX = FieldConstants.LinesVertical.hubCenter;
    double trenchY =
        (FieldConstants.LinesHorizontal.rightTrenchOpenStart
                + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
            / 2.0;
    double rearTipDistance = Constants.RobotDimensions.length / 2.0 + INTAKE_EXTENSION_METERS;
    Translation2d currentTranslation =
        new Translation2d(trenchX + rearTipDistance + Units.inchesToMeters(10.0), trenchY);
    ChassisSpeeds backingTowardTrench = new ChassisSpeeds(-Units.inchesToMeters(24.0), 0.0, 0.0);

    assertFalse(
        Superstructure.shouldDeployRearIntakeForTrench(
            currentTranslation,
            Rotation2d.kZero,
            backingTowardTrench,
            0.1,
            Constants.RobotDimensions.length,
            INTAKE_EXTENSION_METERS,
            TRENCH_SAFETY_ZONE_METERS));
  }

  @Test
  void rearMountedIntakeTriggersWhenProjectedTipEntersBufferWithoutCrossingPlane() {
    double trenchX = FieldConstants.LinesVertical.hubCenter;
    double trenchY =
        (FieldConstants.LinesHorizontal.rightTrenchOpenStart
                + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
            / 2.0;
    double rearTipDistance = Constants.RobotDimensions.length / 2.0 + INTAKE_EXTENSION_METERS;
    Translation2d currentTranslation =
        new Translation2d(trenchX + rearTipDistance + Units.inchesToMeters(10.0), trenchY);
    Translation2d projectedTranslation =
        new Translation2d(trenchX + rearTipDistance + Units.inchesToMeters(5.0), trenchY);

    assertTrue(
        Superstructure.shouldDeployRearIntakeForTrench(
            currentTranslation,
            projectedTranslation,
            Rotation2d.kZero,
            Rotation2d.kZero,
            Constants.RobotDimensions.length,
            INTAKE_EXTENSION_METERS,
            TRENCH_SAFETY_ZONE_METERS));
  }

  @Test
  void rearMountedIntakeDoesNotTriggerJustBecauseTheRobotCenterCrossesFirstWithinLookahead() {
    double trenchX = FieldConstants.LinesVertical.hubCenter;
    double trenchY =
        (FieldConstants.LinesHorizontal.rightTrenchOpenStart
                + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
            / 2.0;
    Translation2d currentTranslation =
        new Translation2d(trenchX - Units.inchesToMeters(2.0), trenchY);
    ChassisSpeeds drivingForwardIntoTrench =
        new ChassisSpeeds(Units.inchesToMeters(24.0), 0.0, 0.0);

    assertFalse(
        Superstructure.shouldDeployRearIntakeForTrench(
            currentTranslation,
            Rotation2d.kZero,
            drivingForwardIntoTrench,
            0.2,
            Constants.RobotDimensions.length,
            INTAKE_EXTENSION_METERS,
            TRENCH_SAFETY_ZONE_METERS));
  }

  @Test
  void trenchDeployStateOnlyArmsFromStowedButStaysLatchedWhileEnvelopeIsActive() {
    assertAll(
        () -> assertFalse(Superstructure.resolveTrenchIntakeDeployState(false, true, false, true)),
        () -> assertTrue(Superstructure.resolveTrenchIntakeDeployState(true, true, false, true)),
        () -> assertFalse(Superstructure.resolveTrenchIntakeDeployState(true, true, false, false)),
        () -> assertFalse(Superstructure.resolveTrenchIntakeDeployState(true, false, true, true)));
  }
}
