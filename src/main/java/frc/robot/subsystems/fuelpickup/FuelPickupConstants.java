package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class FuelPickupConstants {
  private FuelPickupConstants() {}

  public static final int frameWidthPx = 640;
  public static final int frameHeightPx = 480;
  public static final double horizontalFovDeg = 63.3;
  public static final double verticalFovDeg = 49.7;
  public static final double maxDetectionRangeMeters = 8.0;
  public static final double pixelNoiseStdDevPx = 1.4;
  public static final double confidenceNoiseStdDev = 0.08;
  public static final double detectionDropRateNear = 0.02;
  public static final double detectionDropRateFar = 0.40;
  public static final double nearRangeMeters = 1.5;
  public static final double farRangeMeters = 8.0;

  public static final double fieldLengthMeters = 16.51;
  public static final double fieldWidthMeters = 8.04;

  public static final Transform3d robotToLimelight =
      new Transform3d(
          Units.inchesToMeters(9.906),
          Units.inchesToMeters(25.382),
          Units.inchesToMeters(10.004),
          new Rotation3d(0.0, Math.toRadians(25.0), Math.PI));

  public static final double dbscanEpsilonMeters = 0.80;
  public static final int dbscanMinPoints = 2;
  public static final double clusterAssociationDistanceMeters = 1.2;
  public static final int clusterMaxMissedFrames = 8;
  public static final double clusterCentroidEmaAlpha = 0.45;
  public static final double zoneBoundaryOneMeters = fieldLengthMeters / 3.0;
  public static final double zoneBoundaryTwoMeters = 2.0 * fieldLengthMeters / 3.0;

  public static final double countWeight = 2.5;
  public static final double distanceWeight = 1.0;

  public static final int maxLoggedClusters = 8;
  public static final int browserStreamPort = 1187;
  public static final String streamName = "FuelDetectionSim";
}
