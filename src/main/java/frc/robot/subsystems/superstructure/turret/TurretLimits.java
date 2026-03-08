package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

final class TurretLimits {
  static final double MIN_ANGLE_DEG = Constants.SuperstructureConstants.TurretConstants.minAngleDeg;
  static final double MAX_ANGLE_DEG = Constants.SuperstructureConstants.TurretConstants.maxAngleDeg;
  static final double MIN_ANGLE_RAD = Units.degreesToRadians(MIN_ANGLE_DEG);
  static final double MAX_ANGLE_RAD = Units.degreesToRadians(MAX_ANGLE_DEG);

  private TurretLimits() {}

  static Selection selectAngleRadians(double requestedAngleRad, double referenceAngleRad) {
    double requestedDeg = Units.radiansToDegrees(requestedAngleRad);
    double principalDeg = MathUtil.inputModulus(requestedDeg, -180.0, 180.0);
    double referenceDeg = clampDegrees(Units.radiansToDegrees(referenceAngleRad));

    double selectedDeg = Double.NaN;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (int wrap = -2; wrap <= 2; wrap++) {
      double candidateDeg = principalDeg + 360.0 * wrap;
      if (candidateDeg < MIN_ANGLE_DEG || candidateDeg > MAX_ANGLE_DEG) {
        continue;
      }

      double distance = Math.abs(candidateDeg - referenceDeg);
      if (distance < bestDistance) {
        bestDistance = distance;
        selectedDeg = candidateDeg;
      }
    }

    boolean usedLimitFallback = false;
    if (Double.isNaN(selectedDeg)) {
      selectedDeg = nearestLimitDegrees(principalDeg, referenceDeg);
      usedLimitFallback = true;
    }

    return new Selection(requestedDeg, principalDeg, referenceDeg, selectedDeg, usedLimitFallback);
  }

  static double findBestAngleWithinLimitsRadians(
      double requestedAngleRad, double referenceAngleRad) {
    return selectAngleRadians(requestedAngleRad, referenceAngleRad).selectedRadians();
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

  private static double nearestLimitDegrees(double principalDeg, double referenceDeg) {
    double minDistance = angularDistanceDegrees(principalDeg, MIN_ANGLE_DEG);
    double maxDistance = angularDistanceDegrees(principalDeg, MAX_ANGLE_DEG);

    if (Math.abs(minDistance - maxDistance) < 1e-9) {
      double minReferenceDistance = Math.abs(MIN_ANGLE_DEG - referenceDeg);
      double maxReferenceDistance = Math.abs(MAX_ANGLE_DEG - referenceDeg);
      return minReferenceDistance <= maxReferenceDistance ? MIN_ANGLE_DEG : MAX_ANGLE_DEG;
    }

    return minDistance < maxDistance ? MIN_ANGLE_DEG : MAX_ANGLE_DEG;
  }

  private static double angularDistanceDegrees(double firstDeg, double secondDeg) {
    return Math.abs(MathUtil.inputModulus(firstDeg - secondDeg, -180.0, 180.0));
  }

  record Selection(
      double requestedDeg,
      double principalDeg,
      double referenceDeg,
      double selectedDeg,
      boolean usedLimitFallback) {
    double selectedRadians() {
      return Units.degreesToRadians(selectedDeg);
    }
  }
}
