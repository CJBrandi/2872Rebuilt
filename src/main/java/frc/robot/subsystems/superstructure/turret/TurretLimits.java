package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

final class TurretLimits {
  private static final double FULL_ROTATION_DEG = 360.0;
  private static final double EPSILON = 1e-9;

  static final double MIN_ANGLE_DEG = Constants.SuperstructureConstants.TurretConstants.minAngleDeg;
  static final double MAX_ANGLE_DEG = Constants.SuperstructureConstants.TurretConstants.maxAngleDeg;
  static final double MIN_ANGLE_RAD = Units.degreesToRadians(MIN_ANGLE_DEG);
  static final double MAX_ANGLE_RAD = Units.degreesToRadians(MAX_ANGLE_DEG);

  private TurretLimits() {}

  static Selection selectAbsoluteAngleRadians(double requestedAngleRad) {
    double requestedDeg = Units.radiansToDegrees(requestedAngleRad);
    double selectedDeg = clampDegrees(requestedDeg);

    return new Selection(
        requestedDeg, requestedDeg, selectedDeg, Math.abs(selectedDeg - requestedDeg) > EPSILON);
  }

  static Selection selectFieldRelativeAngleRadians(double requestedAngleRad) {
    double requestedDeg = Units.radiansToDegrees(requestedAngleRad);
    double candidateDeg = MathUtil.inputModulus(requestedDeg, -180.0, 180.0);

    while (candidateDeg > MAX_ANGLE_DEG + EPSILON) {
      candidateDeg -= FULL_ROTATION_DEG;
    }
    while (candidateDeg < MIN_ANGLE_DEG - EPSILON) {
      candidateDeg += FULL_ROTATION_DEG;
    }

    double selectedDeg = clampDegrees(candidateDeg);
    return new Selection(
        requestedDeg, candidateDeg, selectedDeg, Math.abs(selectedDeg - candidateDeg) > EPSILON);
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

  record Selection(double requestedDeg, double candidateDeg, double selectedDeg, boolean clamped) {
    double selectedRadians() {
      return Units.degreesToRadians(selectedDeg);
    }
  }
}
