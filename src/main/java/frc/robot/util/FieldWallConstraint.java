package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/** Limits commanded chassis motion so the robot footprint stays inside the field walls. */
public final class FieldWallConstraint {
  private static final double LOOP_PERIOD_SECS = Constants.loopPeriodSecs;
  private static final double OMEGA_EPSILON_RAD_PER_SEC = 1e-6;
  private static final int SEARCH_ITERATIONS = 24;
  private static final double[] MOTION_SAMPLE_ALPHAS = {0.25, 0.5, 0.75, 1.0};

  private static final Translation2d[] FOOTPRINT_CORNERS =
      new Translation2d[] {
        new Translation2d(
            Constants.FieldWallProtection.frontExtentMeters,
            Constants.FieldWallProtection.sideExtentMeters),
        new Translation2d(
            Constants.FieldWallProtection.frontExtentMeters,
            -Constants.FieldWallProtection.sideExtentMeters),
        new Translation2d(
            -Constants.FieldWallProtection.rearExtentMeters,
            Constants.FieldWallProtection.sideExtentMeters),
        new Translation2d(
            -Constants.FieldWallProtection.rearExtentMeters,
            -Constants.FieldWallProtection.sideExtentMeters)
      };

  private FieldWallConstraint() {}

  public static ChassisSpeeds constrain(
      Pose2d currentPose, ChassisSpeeds desiredRobotRelativeSpeeds) {
    Pose2d referencePose = clampPoseToField(currentPose);
    Rotation2d currentHeading = currentPose.getRotation();
    ChassisSpeeds desiredFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            desiredRobotRelativeSpeeds.vxMetersPerSecond,
            desiredRobotRelativeSpeeds.vyMetersPerSecond,
            desiredRobotRelativeSpeeds.omegaRadiansPerSecond,
            currentHeading);

    double fieldVx = desiredFieldRelativeSpeeds.vxMetersPerSecond;
    double fieldVy = desiredFieldRelativeSpeeds.vyMetersPerSecond;
    double omega = desiredRobotRelativeSpeeds.omegaRadiansPerSecond;

    omega = clampOmega(referencePose, fieldVx, fieldVy, omega);
    CenterBounds bounds =
        getAllowedCenterBounds(
            currentHeading.plus(Rotation2d.fromRadians(omega * LOOP_PERIOD_SECS)));
    fieldVx =
        MathUtil.clamp(
            fieldVx,
            (bounds.minX() - referencePose.getX()) / LOOP_PERIOD_SECS,
            (bounds.maxX() - referencePose.getX()) / LOOP_PERIOD_SECS);
    fieldVy =
        MathUtil.clamp(
            fieldVy,
            (bounds.minY() - referencePose.getY()) / LOOP_PERIOD_SECS,
            (bounds.maxY() - referencePose.getY()) / LOOP_PERIOD_SECS);

    omega = clampOmega(referencePose, fieldVx, fieldVy, omega);
    bounds =
        getAllowedCenterBounds(
            currentHeading.plus(Rotation2d.fromRadians(omega * LOOP_PERIOD_SECS)));
    fieldVx =
        MathUtil.clamp(
            fieldVx,
            (bounds.minX() - referencePose.getX()) / LOOP_PERIOD_SECS,
            (bounds.maxX() - referencePose.getX()) / LOOP_PERIOD_SECS);
    fieldVy =
        MathUtil.clamp(
            fieldVy,
            (bounds.minY() - referencePose.getY()) / LOOP_PERIOD_SECS,
            (bounds.maxY() - referencePose.getY()) / LOOP_PERIOD_SECS);

    if (!isMotionSafe(referencePose, fieldVx, fieldVy, omega)) {
      double safeScale = findLargestSafeScale(referencePose, fieldVx, fieldVy, omega);
      fieldVx *= safeScale;
      fieldVy *= safeScale;
      omega *= safeScale;
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldVx, fieldVy, omega, currentHeading);
  }

  static CenterBounds getAllowedCenterBounds(Rotation2d heading) {
    double minOffsetX = Double.POSITIVE_INFINITY;
    double maxOffsetX = Double.NEGATIVE_INFINITY;
    double minOffsetY = Double.POSITIVE_INFINITY;
    double maxOffsetY = Double.NEGATIVE_INFINITY;

    for (Translation2d corner : FOOTPRINT_CORNERS) {
      Translation2d rotatedCorner = corner.rotateBy(heading);
      minOffsetX = Math.min(minOffsetX, rotatedCorner.getX());
      maxOffsetX = Math.max(maxOffsetX, rotatedCorner.getX());
      minOffsetY = Math.min(minOffsetY, rotatedCorner.getY());
      maxOffsetY = Math.max(maxOffsetY, rotatedCorner.getY());
    }

    return new CenterBounds(
        -minOffsetX,
        FieldConstants.fieldLength - maxOffsetX,
        -minOffsetY,
        FieldConstants.fieldWidth - maxOffsetY);
  }

  static boolean isPoseWithinField(Pose2d pose) {
    CenterBounds bounds = getAllowedCenterBounds(pose.getRotation());
    return pose.getX() >= bounds.minX()
        && pose.getX() <= bounds.maxX()
        && pose.getY() >= bounds.minY()
        && pose.getY() <= bounds.maxY();
  }

  static Pose2d clampPoseToField(Pose2d pose) {
    CenterBounds bounds = getAllowedCenterBounds(pose.getRotation());
    return new Pose2d(
        MathUtil.clamp(pose.getX(), bounds.minX(), bounds.maxX()),
        MathUtil.clamp(pose.getY(), bounds.minY(), bounds.maxY()),
        pose.getRotation());
  }

  private static double clampOmega(
      Pose2d currentPose, double fieldVx, double fieldVy, double desiredOmega) {
    if (Math.abs(desiredOmega) <= OMEGA_EPSILON_RAD_PER_SEC
        || isMotionSafe(currentPose, fieldVx, fieldVy, desiredOmega)) {
      return desiredOmega;
    }

    if (!isMotionSafe(currentPose, fieldVx, fieldVy, 0.0)) {
      return 0.0;
    }

    double low = 0.0;
    double high = Math.abs(desiredOmega);
    for (int i = 0; i < SEARCH_ITERATIONS; i++) {
      double mid = (low + high) / 2.0;
      double candidateOmega = Math.copySign(mid, desiredOmega);
      if (isMotionSafe(currentPose, fieldVx, fieldVy, candidateOmega)) {
        low = mid;
      } else {
        high = mid;
      }
    }

    return Math.copySign(low, desiredOmega);
  }

  private static double findLargestSafeScale(
      Pose2d currentPose, double fieldVx, double fieldVy, double omega) {
    if (!isMotionSafe(currentPose, 0.0, 0.0, 0.0)) {
      return 0.0;
    }

    double low = 0.0;
    double high = 1.0;
    for (int i = 0; i < SEARCH_ITERATIONS; i++) {
      double mid = (low + high) / 2.0;
      if (isMotionSafe(currentPose, fieldVx * mid, fieldVy * mid, omega * mid)) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return low;
  }

  static boolean isMotionSafe(
      Pose2d currentPose, double fieldVx, double fieldVy, double omegaRadiansPerSecond) {
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldVx, fieldVy, omegaRadiansPerSecond, currentPose.getRotation());

    for (double alpha : MOTION_SAMPLE_ALPHAS) {
      Pose2d sampledPose =
          currentPose.exp(
              new Twist2d(
                  robotRelativeSpeeds.vxMetersPerSecond * LOOP_PERIOD_SECS * alpha,
                  robotRelativeSpeeds.vyMetersPerSecond * LOOP_PERIOD_SECS * alpha,
                  robotRelativeSpeeds.omegaRadiansPerSecond * LOOP_PERIOD_SECS * alpha));
      if (!isPoseWithinField(sampledPose)) {
        return false;
      }
    }

    return true;
  }

  record CenterBounds(double minX, double maxX, double minY, double maxY) {}
}
