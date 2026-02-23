package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Utility for converting lookup-shot vectors into motion-compensated shooter commands. */
public final class ShotVectorCompensator {
  private ShotVectorCompensator() {}

  /** Motion-compensated shooter command. */
  public record CompensatedShot(
      Translation3d shooterRelativeVelocity,
      Translation3d requiredFieldVelocity,
      Translation3d shooterFieldVelocity,
      Rotation2d yaw,
      double pitch,
      double exitVelocity) {}

  /** Converts a shooter-relative velocity vector directly into yaw/pitch/speed commands. */
  public static CompensatedShot fromShooterRelativeVector(Translation3d shooterRelativeVelocity) {
    return buildShot(
        shooterRelativeVelocity, shooterRelativeVelocity, new Translation3d(0.0, 0.0, 0.0));
  }

  /**
   * Builds a shooter-frame launch vector from scalar speed and pitch.
   *
   * <p>Coordinates are shooter-frame with +x forward, +y left, +z up.
   */
  public static Translation3d lookupVectorFromSpeedPitch(
      double speedMetersPerSecond, double pitchRad) {
    double horizontal = speedMetersPerSecond * Math.cos(pitchRad);
    double vertical = speedMetersPerSecond * Math.sin(pitchRad);
    return new Translation3d(horizontal, 0.0, vertical);
  }

  /** Rotates a shooter-frame lookup vector to a field-relative required velocity vector. */
  public static Translation3d orientLookupVectorToField(
      Translation3d lookupVector, Rotation2d fieldYawToTarget) {
    Translation2d horizontal =
        new Translation2d(lookupVector.getX(), lookupVector.getY()).rotateBy(fieldYawToTarget);
    return new Translation3d(horizontal.getX(), horizontal.getY(), lookupVector.getZ());
  }

  /**
   * Computes field-relative velocity of the shooter exit point.
   *
   * <p>Input chassis speeds are robot-relative.
   */
  public static Translation3d calculateShooterFieldVelocity(
      ChassisSpeeds robotRelativeSpeeds, Rotation2d robotHeading, Translation2d robotToShooter) {
    Translation2d linearFieldVelocity = robotRelativeToField(robotRelativeSpeeds, robotHeading);
    Translation2d shooterOffsetField = robotToShooter.rotateBy(robotHeading);

    // omega x r for planar motion where omega points along +z.
    double tangentialX = -robotRelativeSpeeds.omegaRadiansPerSecond * shooterOffsetField.getY();
    double tangentialY = robotRelativeSpeeds.omegaRadiansPerSecond * shooterOffsetField.getX();

    return new Translation3d(
        linearFieldVelocity.getX() + tangentialX, linearFieldVelocity.getY() + tangentialY, 0.0);
  }

  /**
   * Computes the shooter command required to achieve a field-relative launch velocity.
   *
   * <p>The commanded shooter-relative velocity is:
   *
   * <p>v_shooter = v_required_field - v_shooter_field
   */
  public static CompensatedShot compensateForRobotMotion(
      Translation3d requiredFieldVelocity,
      ChassisSpeeds robotRelativeSpeeds,
      Rotation2d robotHeading,
      Translation2d robotToShooter) {
    Translation3d shooterFieldVelocity =
        calculateShooterFieldVelocity(robotRelativeSpeeds, robotHeading, robotToShooter);
    Translation3d shooterRelativeVelocity = requiredFieldVelocity.minus(shooterFieldVelocity);
    return buildShot(shooterRelativeVelocity, requiredFieldVelocity, shooterFieldVelocity);
  }

  /** Converts robot-relative chassis linear velocity to field-relative translation velocity. */
  public static Translation2d robotRelativeToField(
      ChassisSpeeds robotRelativeSpeeds, Rotation2d robotHeading) {
    return new Translation2d(
            robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        .rotateBy(robotHeading);
  }

  private static CompensatedShot buildShot(
      Translation3d shooterRelativeVelocity,
      Translation3d requiredFieldVelocity,
      Translation3d shooterFieldVelocity) {
    double horizontalSpeed =
        Math.hypot(shooterRelativeVelocity.getX(), shooterRelativeVelocity.getY());
    double exitVelocity = shooterRelativeVelocity.getNorm();
    Rotation2d yaw =
        horizontalSpeed > 1e-9
            ? new Rotation2d(shooterRelativeVelocity.getX(), shooterRelativeVelocity.getY())
            : Rotation2d.kZero;
    double pitch = Math.atan2(shooterRelativeVelocity.getZ(), horizontalSpeed);

    return new CompensatedShot(
        shooterRelativeVelocity,
        requiredFieldVelocity,
        shooterFieldVelocity,
        yaw,
        pitch,
        exitVelocity);
  }
}
