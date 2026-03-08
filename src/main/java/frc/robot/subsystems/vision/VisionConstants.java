// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Tag camera configurations
  public static final TagCameraConfig camera0Config =
      TagCameraConfig.turretMounted(
          "camera_0",
          new TagCameraConfig.TurretMountSettings(
              new Translation3d(Units.inchesToMeters(0.25), 0.0, 0.419),
              new Translation3d(Units.inchesToMeters(8.532), 0.0, 0.0),
              new Rotation3d(0.0, -0.2261799, 0.0)),
          1.0);

  public static final TagCameraConfig camera1Config =
      TagCameraConfig.fixed(
          "camera_1",
          new Transform3d(
              Units.inchesToMeters(-13.426), // 13.426
              Units.inchesToMeters(-5.957), // 5.957
              Units.inchesToMeters(16.421), // 16.421
              new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(45))),
          1.0);

  public static final TagCameraConfig[] tagCameraConfigs =
      new TagCameraConfig[] {camera0Config, camera1Config};

  public static Transform3d robotToDetectionCamera =
      new Transform3d(
          Units.inchesToMeters(-25.382), // 9.906
          Units.inchesToMeters(9.906), // 25.382
          Units.inchesToMeters(10.004),
          new Rotation3d(0.0, Math.toRadians(10), Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  /** Returns the configured standard-deviation scale factor for a camera index. */
  public static double getCameraStdDevFactor(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= tagCameraConfigs.length) {
      return 1.0;
    }
    return tagCameraConfigs[cameraIndex].stdDevFactor();
  }

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
