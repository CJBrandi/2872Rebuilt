package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.util.FieldConstants;
import org.junit.jupiter.api.Test;

class SuperstructureTrenchStowTest {
  private static final double HALF_ROBOT_LENGTH = Constants.RobotDimensions.length / 2.0;

  @Test
  void parkedNextToTrenchWithoutCrossingDoesNotStow() {
    Translation2d currentTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.hubCenter - HALF_ROBOT_LENGTH - 0.01,
            rightTrenchOpeningMidY());

    assertFalse(
        Superstructure.shouldStowForTrench(
            currentTranslation, currentTranslation, Constants.RobotDimensions.length));
  }

  @Test
  void crossingRightTrenchOpeningStows() {
    Translation2d currentTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.hubCenter - HALF_ROBOT_LENGTH - 0.05,
            rightTrenchOpeningMidY());
    Translation2d projectedTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.hubCenter + HALF_ROBOT_LENGTH + 0.05,
            rightTrenchOpeningMidY());

    assertTrue(
        Superstructure.shouldStowForTrench(
            currentTranslation, projectedTranslation, Constants.RobotDimensions.length));
  }

  @Test
  void crossingLeftTrenchOpeningStows() {
    Translation2d currentTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.oppHubCenter - HALF_ROBOT_LENGTH - 0.05,
            leftTrenchOpeningMidY());
    Translation2d projectedTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.oppHubCenter + HALF_ROBOT_LENGTH + 0.05,
            leftTrenchOpeningMidY());

    assertTrue(
        Superstructure.shouldStowForTrench(
            currentTranslation, projectedTranslation, Constants.RobotDimensions.length));
  }

  @Test
  void crossingOutsideTrenchOpeningDoesNotStow() {
    Translation2d currentTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.hubCenter - HALF_ROBOT_LENGTH - 0.05,
            FieldConstants.LinesHorizontal.center);
    Translation2d projectedTranslation =
        new Translation2d(
            FieldConstants.LinesVertical.hubCenter + HALF_ROBOT_LENGTH + 0.05,
            FieldConstants.LinesHorizontal.center);

    assertFalse(
        Superstructure.shouldStowForTrench(
            currentTranslation, projectedTranslation, Constants.RobotDimensions.length));
  }

  @Test
  void alreadyUnderTrenchOpeningStows() {
    Translation2d currentTranslation =
        new Translation2d(FieldConstants.LinesVertical.hubCenter, rightTrenchOpeningMidY());

    assertTrue(
        Superstructure.shouldStowForTrench(
            currentTranslation, currentTranslation, Constants.RobotDimensions.length));
  }

  private static double rightTrenchOpeningMidY() {
    return average(
        FieldConstants.LinesHorizontal.rightTrenchOpenStart,
        FieldConstants.LinesHorizontal.rightTrenchOpenEnd);
  }

  private static double leftTrenchOpeningMidY() {
    return average(
        FieldConstants.LinesHorizontal.leftTrenchOpenStart,
        FieldConstants.LinesHorizontal.leftTrenchOpenEnd);
  }

  private static double average(double a, double b) {
    return (a + b) / 2.0;
  }
}
