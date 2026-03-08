// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {

  /** Check if field coordinates should be flipped based on alliance */
  private static boolean shouldFlip() {
    return !Constants.disableHAL
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  /** Wrapper class for Translation2d that provides flipping functionality */
  public static class FlippableTranslation2d {
    private final Translation2d value;

    public FlippableTranslation2d(double x, double y) {
      this.value = new Translation2d(x, y);
    }

    public FlippableTranslation2d(Translation2d value) {
      this.value = value;
    }

    /** Returns alliance-aware coordinates (flipped for red alliance) */
    public Translation2d get() {
      return shouldFlip()
          ? new Translation2d(fieldLength - value.getX(), fieldWidth - value.getY())
          : value;
    }

    /** Returns blue alliance coordinates (raw, unflipped) */
    public Translation2d getBlue() {
      return value;
    }

    /** Returns red alliance coordinates (always flipped) */
    public Translation2d getRed() {
      return new Translation2d(fieldLength - value.getX(), fieldWidth - value.getY());
    }
  }

  /** Wrapper class for Translation3d that provides flipping functionality */
  public static class FlippableTranslation3d {
    private final Translation3d value;

    public FlippableTranslation3d(double x, double y, double z) {
      this.value = new Translation3d(x, y, z);
    }

    public FlippableTranslation3d(Translation3d value) {
      this.value = value;
    }

    /** Returns alliance-aware coordinates (flipped for red alliance) */
    public Translation3d get() {
      return shouldFlip()
          ? new Translation3d(fieldLength - value.getX(), fieldWidth - value.getY(), value.getZ())
          : value;
    }

    /** Returns blue alliance coordinates (raw, unflipped) */
    public Translation3d getBlue() {
      return value;
    }

    /** Returns red alliance coordinates (always flipped) */
    public Translation3d getRed() {
      return new Translation3d(fieldLength - value.getX(), fieldWidth - value.getY(), value.getZ());
    }
  }

  /** Wrapper class for Pose2d that provides flipping functionality */
  public static class FlippablePose2d {
    private final Pose2d value;

    public FlippablePose2d(double x, double y, Rotation2d rotation) {
      this.value = new Pose2d(x, y, rotation);
    }

    public FlippablePose2d(Translation2d translation, Rotation2d rotation) {
      this.value = new Pose2d(translation, rotation);
    }

    public FlippablePose2d(Pose2d value) {
      this.value = value;
    }

    /** Returns alliance-aware pose (flipped for red alliance) */
    public Pose2d get() {
      return shouldFlip()
          ? new Pose2d(
              fieldLength - value.getX(),
              fieldWidth - value.getY(),
              value.getRotation().rotateBy(Rotation2d.kPi))
          : value;
    }

    /** Returns blue alliance pose (raw, unflipped) */
    public Pose2d getBlue() {
      return value;
    }

    /** Returns red alliance pose (always flipped) */
    public Pose2d getRed() {
      return new Pose2d(
          fieldLength - value.getX(),
          fieldWidth - value.getY(),
          value.getRotation().rotateBy(Rotation2d.kPi));
    }
  }

  public static final FieldType fieldType = FieldType.WELDED;

  // AprilTag related constants
  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  // Field dimensions
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
    public static final double oppAllianceZone =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double center = fieldWidth / 2.0;

    // Right of hub
    public static final double rightBumpStart = Hub.nearRightCorner.getBlue().getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left of hub
    public static final double leftBumpEnd = Hub.nearLeftCorner.getBlue().getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final FlippableTranslation3d topCenterPoint =
        new FlippableTranslation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final FlippableTranslation3d innerCenterPoint =
        new FlippableTranslation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);

    public static final FlippableTranslation2d nearLeftCorner =
        new FlippableTranslation2d(
            topCenterPoint.getBlue().getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final FlippableTranslation2d nearRightCorner =
        new FlippableTranslation2d(
            topCenterPoint.getBlue().getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final FlippableTranslation2d farLeftCorner =
        new FlippableTranslation2d(
            topCenterPoint.getBlue().getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final FlippableTranslation2d farRightCorner =
        new FlippableTranslation2d(
            topCenterPoint.getBlue().getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final FlippableTranslation3d oppTopCenterPoint =
        new FlippableTranslation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final FlippableTranslation2d oppNearLeftCorner =
        new FlippableTranslation2d(
            oppTopCenterPoint.getBlue().getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final FlippableTranslation2d oppNearRightCorner =
        new FlippableTranslation2d(
            oppTopCenterPoint.getBlue().getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final FlippableTranslation2d oppFarLeftCorner =
        new FlippableTranslation2d(
            oppTopCenterPoint.getBlue().getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final FlippableTranslation2d oppFarRightCorner =
        new FlippableTranslation2d(
            oppTopCenterPoint.getBlue().getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Hub faces
    public static final FlippablePose2d nearFace =
        new FlippablePose2d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d());
    public static final FlippablePose2d farFace =
        new FlippablePose2d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d());
    public static final FlippablePose2d rightFace =
        new FlippablePose2d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d());
    public static final FlippablePose2d leftFace =
        new FlippablePose2d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d());
  }

  /** Lob-shot targets (defined in blue-alliance field coordinates). */
  public static class Lob {
    public static final FlippableTranslation3d LOB_RIGHT =
        new FlippableTranslation3d(2.0, 1.0, 0.0);
    public static final FlippableTranslation3d LOB_LEFT = new FlippableTranslation3d(2.0, 7.0, 0.0);
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final FlippableTranslation2d nearLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d nearRightCorner = Hub.nearLeftCorner;
    public static final FlippableTranslation2d farLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final FlippableTranslation2d oppNearLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final FlippableTranslation2d oppFarLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final FlippableTranslation2d nearLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d nearRightCorner = Hub.nearLeftCorner;
    public static final FlippableTranslation2d farLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final FlippableTranslation2d oppNearLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final FlippableTranslation2d oppFarLeftCorner =
        new FlippableTranslation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final FlippableTranslation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final FlippableTranslation3d openingTopLeft =
        new FlippableTranslation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
    public static final FlippableTranslation3d openingTopRight =
        new FlippableTranslation3d(
            LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

    // Relevant reference points on opposing side
    public static final FlippableTranslation3d oppOpeningTopLeft =
        new FlippableTranslation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
    public static final FlippableTranslation3d oppOpeningTopRight =
        new FlippableTranslation3d(
            LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
  }

  public static class RightTrench {

    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final FlippableTranslation3d openingTopLeft =
        new FlippableTranslation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final FlippableTranslation3d openingTopRight =
        new FlippableTranslation3d(LinesVertical.hubCenter, 0, openingHeight);

    // Relevant reference points on opposing side
    public static final FlippableTranslation3d oppOpeningTopLeft =
        new FlippableTranslation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final FlippableTranslation3d oppOpeningTopRight =
        new FlippableTranslation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final FlippableTranslation2d centerPoint =
        new FlippableTranslation2d(
            frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
    public static final FlippableTranslation2d leftUpright =
        new FlippableTranslation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final FlippableTranslation2d rightUpright =
        new FlippableTranslation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));

    // Relevant reference points on opposing side
    public static final FlippableTranslation2d oppCenterPoint =
        new FlippableTranslation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
    public static final FlippableTranslation2d oppLeftUpright =
        new FlippableTranslation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final FlippableTranslation2d oppRightUpright =
        new FlippableTranslation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final FlippableTranslation3d depotCenter =
        new FlippableTranslation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
    public static final FlippableTranslation3d leftCorner =
        new FlippableTranslation3d(
            depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
    public static final FlippableTranslation3d rightCorner =
        new FlippableTranslation3d(
            depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
  }

  public static class Outpost {
    // Dimensions
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final FlippableTranslation2d centerPoint =
        new FlippableTranslation2d(
            0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
  }

  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              Path p =
                  Constants.disableHAL
                      ? Path.of(
                          "src",
                          "main",
                          "deploy",
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json")
                      : Path.of(
                          Filesystem.getDeployDirectory().getPath(),
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}
