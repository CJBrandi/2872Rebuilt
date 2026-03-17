package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class FieldWallConstraintTest {
  private static final double EPSILON = 1e-9;

  @Test
  void zeroHeadingUsesRearIntakeAndFrontSideBumpersForWallClearance() {
    FieldWallConstraint.CenterBounds bounds =
        FieldWallConstraint.getAllowedCenterBounds(Rotation2d.kZero);

    assertAll(
        () -> assertEquals(Constants.FieldWallProtection.rearExtentMeters, bounds.minX(), EPSILON),
        () ->
            assertEquals(
                FieldConstants.fieldLength - Constants.FieldWallProtection.frontExtentMeters,
                bounds.maxX(),
                EPSILON),
        () -> assertEquals(Constants.FieldWallProtection.sideExtentMeters, bounds.minY(), EPSILON),
        () ->
            assertEquals(
                FieldConstants.fieldWidth - Constants.FieldWallProtection.sideExtentMeters,
                bounds.maxY(),
                EPSILON));
  }

  @Test
  void intakeSideCannotDriveIntoWallWhileStillSlidingAlongIt() {
    Pose2d pose =
        new Pose2d(
            Constants.FieldWallProtection.rearExtentMeters,
            FieldConstants.fieldWidth / 2.0,
            Rotation2d.kZero);
    ChassisSpeeds constrained =
        FieldWallConstraint.constrain(pose, new ChassisSpeeds(-1.0, 0.75, 0.0));

    assertAll(
        () -> assertEquals(0.0, constrained.vxMetersPerSecond, EPSILON),
        () -> assertEquals(0.75, constrained.vyMetersPerSecond, EPSILON),
        () -> assertEquals(0.0, constrained.omegaRadiansPerSecond, EPSILON));
  }

  @Test
  void slightlyPastWallStillAllowsSlidingAndDrivingBackOut() {
    Pose2d pose =
        new Pose2d(
            Constants.FieldWallProtection.rearExtentMeters - 0.01,
            FieldConstants.fieldWidth / 2.0,
            Rotation2d.kZero);
    ChassisSpeeds constrained =
        FieldWallConstraint.constrain(pose, new ChassisSpeeds(-1.0, 0.75, 0.0));
    ChassisSpeeds escape = FieldWallConstraint.constrain(pose, new ChassisSpeeds(1.0, 0.0, 0.0));

    assertAll(
        () -> assertEquals(0.0, constrained.vxMetersPerSecond, EPSILON),
        () -> assertEquals(0.75, constrained.vyMetersPerSecond, EPSILON),
        () -> assertEquals(1.0, escape.vxMetersPerSecond, EPSILON));
  }

  @Test
  void frontBumperCannotDriveIntoWallWhenRobotFacesThatWall() {
    Pose2d pose =
        new Pose2d(
            Constants.FieldWallProtection.frontExtentMeters,
            FieldConstants.fieldWidth / 2.0,
            Rotation2d.kPi);
    ChassisSpeeds constrained =
        FieldWallConstraint.constrain(pose, new ChassisSpeeds(1.0, 0.0, 0.0));

    assertEquals(0.0, constrained.vxMetersPerSecond, EPSILON);
  }

  @Test
  void rotationIsReducedWhenCornerWouldSwingIntoWall() {
    Pose2d pose =
        new Pose2d(
            Constants.FieldWallProtection.sideExtentMeters + 0.005,
            FieldConstants.fieldWidth / 2.0,
            Rotation2d.fromDegrees(90.0));
    ChassisSpeeds desired = new ChassisSpeeds(0.0, 0.0, -4.0);
    ChassisSpeeds desiredFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            desired.vxMetersPerSecond,
            desired.vyMetersPerSecond,
            desired.omegaRadiansPerSecond,
            pose.getRotation());
    ChassisSpeeds constrained = FieldWallConstraint.constrain(pose, desired);
    ChassisSpeeds constrainedFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            constrained.vxMetersPerSecond,
            constrained.vyMetersPerSecond,
            constrained.omegaRadiansPerSecond,
            pose.getRotation());

    assertAll(
        () ->
            assertFalse(
                FieldWallConstraint.isMotionSafe(
                    pose,
                    desiredFieldRelative.vxMetersPerSecond,
                    desiredFieldRelative.vyMetersPerSecond,
                    desiredFieldRelative.omegaRadiansPerSecond)),
        () ->
            assertTrue(
                FieldWallConstraint.isMotionSafe(
                    pose,
                    constrainedFieldRelative.vxMetersPerSecond,
                    constrainedFieldRelative.vyMetersPerSecond,
                    constrainedFieldRelative.omegaRadiansPerSecond)),
        () ->
            assertTrue(
                Math.abs(constrained.omegaRadiansPerSecond)
                    < Math.abs(desired.omegaRadiansPerSecond)));
  }
}
