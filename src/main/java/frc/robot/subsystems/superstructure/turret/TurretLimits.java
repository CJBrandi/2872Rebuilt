package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

final class TurretLimits {
  private static final double FULL_ROTATION_DEG = 360.0;
  private static final double EPSILON = 1e-9;

  static final double MIN_ANGLE_DEG =
      Math.min(
          Constants.SuperstructureConstants.TurretConstants.minAngleDeg,
          Constants.SuperstructureConstants.TurretConstants.maxAngleDeg);
  static final double MAX_ANGLE_DEG =
      Math.max(
          Constants.SuperstructureConstants.TurretConstants.minAngleDeg,
          Constants.SuperstructureConstants.TurretConstants.maxAngleDeg);
  static final double MIN_ANGLE_RAD = Units.degreesToRadians(MIN_ANGLE_DEG);
  static final double MAX_ANGLE_RAD = Units.degreesToRadians(MAX_ANGLE_DEG);

  private TurretLimits() {}

  static Selection selectAbsoluteAngleRadians(double requestedAngleRad, double referenceAngleRad) {
    return selectClosestEquivalentAngleDegrees(
        Units.radiansToDegrees(requestedAngleRad), Units.radiansToDegrees(referenceAngleRad));
  }

  static Selection selectFieldRelativeAngleRadians(
      double requestedAngleRad, double referenceAngleRad) {
    return selectClosestEquivalentAngleDegrees(
        Units.radiansToDegrees(requestedAngleRad), Units.radiansToDegrees(referenceAngleRad));
  }

  static double clampDegrees(double angleDeg) {
    return MathUtil.clamp(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
  }

  static double clampRadians(double angleRad) {
    return MathUtil.clamp(angleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
  }

  static boolean commandWouldPushPastLimit(double angleRad, double velocityRadPerSec) {
    return (angleRad >= MAX_ANGLE_RAD && velocityRadPerSec > 0.0)
        || (angleRad <= MIN_ANGLE_RAD && velocityRadPerSec < 0.0);
  }

  private static Selection selectClosestEquivalentAngleDegrees(
      double requestedDeg, double referenceDeg) {
    long nearestTurnOffset = Math.round((referenceDeg - requestedDeg) / FULL_ROTATION_DEG);
    double candidateDeg = requestedDeg + nearestTurnOffset * FULL_ROTATION_DEG;

    long minTurnOffset =
        (long) Math.ceil((MIN_ANGLE_DEG - requestedDeg - EPSILON) / FULL_ROTATION_DEG);
    long maxTurnOffset =
        (long) Math.floor((MAX_ANGLE_DEG - requestedDeg + EPSILON) / FULL_ROTATION_DEG);

    if (minTurnOffset <= maxTurnOffset) {
      long boundedTurnOffset = Math.max(minTurnOffset, Math.min(maxTurnOffset, nearestTurnOffset));
      candidateDeg = requestedDeg + boundedTurnOffset * FULL_ROTATION_DEG;
    }

    double selectedDeg = clampDegrees(candidateDeg);
    return new Selection(
        requestedDeg,
        referenceDeg,
        candidateDeg,
        selectedDeg,
        Math.abs(selectedDeg - candidateDeg) > EPSILON);
  }

  record Selection(
      double requestedDeg,
      double referenceDeg,
      double candidateDeg,
      double selectedDeg,
      boolean clamped) {
    double selectedRadians() {
      return Units.degreesToRadians(selectedDeg);
    }
  }
}
