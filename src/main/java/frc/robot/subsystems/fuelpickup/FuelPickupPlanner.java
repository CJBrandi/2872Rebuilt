package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fuelpickup.FuelDbscanClusterer.ClusterResult;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSim.FuelState;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * End-to-end simulation pipeline for fuel pickup perception.
 *
 * <p>Pipeline: Simulated detections -> field projection -> DBSCAN clusters -> cluster scoring.
 */
public class FuelPickupPlanner extends SubsystemBase {
  private static final Scalar BACKGROUND_COLOR = new Scalar(28, 22, 18);
  private static final Scalar HUD_TEXT_COLOR = new Scalar(235, 235, 235);
  private static final Scalar HUD_SUBTEXT_COLOR = new Scalar(200, 200, 200);
  private static final Scalar RAW_DETECTION_COLOR = new Scalar(95, 95, 95);
  private static final Scalar NOISE_COLOR = new Scalar(135, 135, 135);
  private static final Scalar BEST_COLOR = new Scalar(0, 215, 255);
  private static final Scalar[] CLUSTER_COLORS = {
    new Scalar(66, 133, 244),
    new Scalar(52, 168, 83),
    new Scalar(219, 68, 55),
    new Scalar(244, 180, 0),
    new Scalar(17, 198, 232),
    new Scalar(124, 77, 255),
    new Scalar(255, 112, 67),
    new Scalar(141, 110, 99)
  };

  private record SimDetection(double xPx, double yPx, double radiusPx, double confidence) {}

  private record ProjectedFieldPoint(
      int pointId, Translation3d position, double xPx, double yPx, double radiusPx) {}

  private record DetectionSimulationStats(
      int totalFuelCount,
      int projectedInFrameCount,
      int projectionRejectedCount,
      int rangeRejectedCount,
      int dropRejectedCount,
      int noisyOutOfFrameRejectedCount,
      double minProjectedDepthMeters,
      double maxProjectedDepthMeters) {}

  private record ClusterScore(
      int clusterId, Translation3d centroid, int count, double distanceMeters, double score) {}

  private final Supplier<Pose2d> robotPoseSupplier;
  private final FuelSim fuelSim = FuelSim.getInstance();
  private final FuelCameraModel cameraModel =
      new FuelCameraModel(
          FuelPickupConstants.frameWidthPx,
          FuelPickupConstants.frameHeightPx,
          FuelPickupConstants.horizontalFovDeg,
          FuelPickupConstants.verticalFovDeg);
  private final FuelDbscanClusterer clusterer =
      new FuelDbscanClusterer(
          FuelPickupConstants.dbscanEpsilonMeters, FuelPickupConstants.dbscanMinPoints);
  private final FuelClusterStabilizer clusterStabilizer =
      new FuelClusterStabilizer(
          FuelPickupConstants.clusterAssociationDistanceMeters,
          FuelPickupConstants.clusterMaxMissedFrames,
          FuelPickupConstants.clusterCentroidEmaAlpha);
  private final Random random = new Random(2026);
  private DetectionSimulationStats lastDetectionStats =
      new DetectionSimulationStats(0, 0, 0, 0, 0, 0, Double.NaN, Double.NaN);

  private final CvSource videoSource;
  private final MjpegServer browserServer;
  private final Mat frameMat =
      new Mat(FuelPickupConstants.frameHeightPx, FuelPickupConstants.frameWidthPx, CvType.CV_8UC3);

  public FuelPickupPlanner(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.videoSource =
        CameraServer.putVideo(
            FuelPickupConstants.streamName,
            FuelPickupConstants.frameWidthPx,
            FuelPickupConstants.frameHeightPx);
    this.browserServer =
        new MjpegServer(
            FuelPickupConstants.streamName + "_browser", FuelPickupConstants.browserStreamPort);
    this.browserServer.setSource(videoSource);

    Logger.recordOutput(
        "FuelPickup/Video/BrowserUrl",
        "http://localhost:" + FuelPickupConstants.browserStreamPort + "/stream.mjpg");
    Logger.recordOutput("FuelPickup/Video/CameraName", FuelPickupConstants.streamName);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();
    Pose3d cameraPose = new Pose3d(robotPose).plus(FuelPickupConstants.robotToLimelight);
    FuelState[] fuelStates = fuelSim.getFuelStates();
    Logger.recordOutput("FuelPickup/CameraPose", cameraPose);

    List<SimDetection> detections = simulateDetections(fuelStates, cameraPose);
    logDetections(detections);

    List<ProjectedFieldPoint> projectedFieldPoints = estimateFieldPoints(detections, cameraPose);
    logFieldEstimates(projectedFieldPoints);

    List<Translation3d> estimatedPositions =
        projectedFieldPoints.stream().map(ProjectedFieldPoint::position).toList();
    ClusterResult clusterResult = clusterer.cluster(estimatedPositions);
    Map<Integer, List<ProjectedFieldPoint>> rawClusterMap =
        buildClusterMap(projectedFieldPoints, clusterResult.labels());
    Map<Integer, Translation3d> rawClusterCentroids = buildClusterCentroids(rawClusterMap);
    Map<Integer, Integer> rawToStableClusterIds = clusterStabilizer.update(rawClusterCentroids);
    Map<Integer, List<ProjectedFieldPoint>> stableClusterMap =
        remapClusterMap(rawClusterMap, rawToStableClusterIds);
    int[] stablePointClusterLabels =
        remapTrackClusterLabels(clusterResult.labels(), rawToStableClusterIds);

    List<ClusterScore> clusterScores = scoreClusters(stableClusterMap, robotPose);
    int bestClusterId = selectBestCluster(clusterScores);

    logClusters(
        clusterResult.clusterCount(),
        stableClusterMap.size(),
        projectedFieldPoints,
        clusterResult.labels(),
        stablePointClusterLabels,
        stableClusterMap,
        clusterScores,
        bestClusterId);
    renderFrame(detections, projectedFieldPoints, stablePointClusterLabels, clusterScores);
  }

  private List<SimDetection> simulateDetections(FuelState[] fuelStates, Pose3d cameraPose) {
    List<SimDetection> detections = new ArrayList<>();
    int projectionRejectedCount = 0;
    int projectedInFrameCount = 0;
    int rangeRejectedCount = 0;
    int dropRejectedCount = 0;
    int noisyOutOfFrameRejectedCount = 0;
    double minProjectedDepthMeters = Double.POSITIVE_INFINITY;
    double maxProjectedDepthMeters = Double.NEGATIVE_INFINITY;

    for (FuelState fuelState : fuelStates) {
      FuelCameraModel.PixelObservation projected =
          cameraModel
              .project(fuelState.position(), cameraPose, FuelSim.getFuelRadiusMeters())
              .orElse(null);
      if (projected == null) {
        projectionRejectedCount++;
        continue;
      }
      projectedInFrameCount++;
      minProjectedDepthMeters = Math.min(minProjectedDepthMeters, projected.depthMeters());
      maxProjectedDepthMeters = Math.max(maxProjectedDepthMeters, projected.depthMeters());

      if (projected.depthMeters() > FuelPickupConstants.maxDetectionRangeMeters) {
        rangeRejectedCount++;
        continue;
      }

      double dropRate = interpolationByRange(projected.depthMeters());
      if (random.nextDouble() < dropRate) {
        dropRejectedCount++;
        continue;
      }

      double xPx = projected.xPx() + random.nextGaussian() * FuelPickupConstants.pixelNoiseStdDevPx;
      double yPx = projected.yPx() + random.nextGaussian() * FuelPickupConstants.pixelNoiseStdDevPx;
      if (xPx < 0.0
          || xPx >= FuelPickupConstants.frameWidthPx
          || yPx < 0.0
          || yPx >= FuelPickupConstants.frameHeightPx) {
        noisyOutOfFrameRejectedCount++;
        continue;
      }
      double radiusPx =
          Math.max(1.0, projected.radiusPx() + 0.2 * random.nextGaussian() * projected.radiusPx());

      double rangeT =
          clamp(
              (projected.depthMeters() - FuelPickupConstants.nearRangeMeters)
                  / (FuelPickupConstants.farRangeMeters - FuelPickupConstants.nearRangeMeters),
              0.0,
              1.0);
      double confidence =
          clamp(
              1.0
                  - 0.6 * rangeT
                  + random.nextGaussian() * FuelPickupConstants.confidenceNoiseStdDev,
              0.05,
              0.99);
      detections.add(new SimDetection(xPx, yPx, radiusPx, confidence));
    }

    detections.sort(Comparator.comparingDouble(SimDetection::confidence).reversed());
    lastDetectionStats =
        new DetectionSimulationStats(
            fuelStates.length,
            projectedInFrameCount,
            projectionRejectedCount,
            rangeRejectedCount,
            dropRejectedCount,
            noisyOutOfFrameRejectedCount,
            projectedInFrameCount > 0 ? minProjectedDepthMeters : Double.NaN,
            projectedInFrameCount > 0 ? maxProjectedDepthMeters : Double.NaN);
    return detections;
  }

  private double interpolationByRange(double depthMeters) {
    double t =
        clamp(
            (depthMeters - FuelPickupConstants.nearRangeMeters)
                / (FuelPickupConstants.farRangeMeters - FuelPickupConstants.nearRangeMeters),
            0.0,
            1.0);
    return FuelPickupConstants.detectionDropRateNear
        + t
            * (FuelPickupConstants.detectionDropRateFar
                - FuelPickupConstants.detectionDropRateNear);
  }

  private List<ProjectedFieldPoint> estimateFieldPoints(
      List<SimDetection> detections, Pose3d cameraPose) {
    List<ProjectedFieldPoint> points = new ArrayList<>();
    for (int i = 0; i < detections.size(); i++) {
      SimDetection detection = detections.get(i);
      Translation3d fieldPoint =
          cameraModel
              .backProjectToPlane(
                  detection.xPx(), detection.yPx(), cameraPose, FuelSim.getFuelRadiusMeters())
              .orElse(null);
      if (fieldPoint == null) {
        continue;
      }
      if (fieldPoint.getX() < 0.0
          || fieldPoint.getX() > FuelPickupConstants.fieldLengthMeters
          || fieldPoint.getY() < 0.0
          || fieldPoint.getY() > FuelPickupConstants.fieldWidthMeters) {
        continue;
      }
      points.add(
          new ProjectedFieldPoint(
              i, fieldPoint, detection.xPx(), detection.yPx(), detection.radiusPx()));
    }
    return points;
  }

  private Map<Integer, List<ProjectedFieldPoint>> buildClusterMap(
      List<ProjectedFieldPoint> projectedFieldPoints, int[] labels) {
    Map<Integer, List<ProjectedFieldPoint>> clusters = new HashMap<>();
    for (int i = 0; i < projectedFieldPoints.size(); i++) {
      int label = labels[i];
      if (label < 0) {
        continue;
      }
      clusters
          .computeIfAbsent(label, ignored -> new ArrayList<>())
          .add(projectedFieldPoints.get(i));
    }
    return clusters;
  }

  private Map<Integer, Translation3d> buildClusterCentroids(
      Map<Integer, List<ProjectedFieldPoint>> clusterMap) {
    Map<Integer, Translation3d> centroids = new HashMap<>();
    for (Map.Entry<Integer, List<ProjectedFieldPoint>> entry : clusterMap.entrySet()) {
      centroids.put(entry.getKey(), average(entry.getValue()));
    }
    return centroids;
  }

  private Map<Integer, List<ProjectedFieldPoint>> remapClusterMap(
      Map<Integer, List<ProjectedFieldPoint>> rawClusterMap,
      Map<Integer, Integer> rawToStableClusterIds) {
    Map<Integer, List<ProjectedFieldPoint>> stableClusters = new HashMap<>();
    for (Map.Entry<Integer, List<ProjectedFieldPoint>> entry : rawClusterMap.entrySet()) {
      Integer stableId = rawToStableClusterIds.get(entry.getKey());
      if (stableId == null) {
        continue;
      }
      stableClusters
          .computeIfAbsent(stableId, ignored -> new ArrayList<>())
          .addAll(entry.getValue());
    }
    return stableClusters;
  }

  private int[] remapTrackClusterLabels(
      int[] rawLabels, Map<Integer, Integer> rawToStableClusterIds) {
    int[] stableLabels = rawLabels.clone();
    for (int i = 0; i < stableLabels.length; i++) {
      if (stableLabels[i] < 0) {
        continue;
      }
      stableLabels[i] =
          rawToStableClusterIds.getOrDefault(stableLabels[i], FuelDbscanClusterer.NOISE);
    }
    return stableLabels;
  }

  private List<ClusterScore> scoreClusters(
      Map<Integer, List<ProjectedFieldPoint>> clusterMap, Pose2d robotPose) {
    List<ClusterScore> scores = new ArrayList<>();
    Translation2d robotTranslation = robotPose.getTranslation();
    for (Map.Entry<Integer, List<ProjectedFieldPoint>> entry : clusterMap.entrySet()) {
      int clusterId = entry.getKey();
      List<ProjectedFieldPoint> points = entry.getValue();
      int count = points.size();
      Translation3d centroid = average(points);
      double distance = robotTranslation.getDistance(centroid.toTranslation2d());
      double score =
          FuelPickupConstants.countWeight * count - FuelPickupConstants.distanceWeight * distance;
      scores.add(new ClusterScore(clusterId, centroid, count, distance, score));
    }
    scores.sort(Comparator.comparingDouble(ClusterScore::score).reversed());
    return scores;
  }

  private int selectBestCluster(List<ClusterScore> scores) {
    if (scores.isEmpty()) {
      return -1;
    }
    return scores.get(0).clusterId();
  }

  private Translation3d average(List<ProjectedFieldPoint> points) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (ProjectedFieldPoint point : points) {
      x += point.position().getX();
      y += point.position().getY();
      z += point.position().getZ();
    }
    double invCount = 1.0 / points.size();
    return new Translation3d(x * invCount, y * invCount, z * invCount);
  }

  private void renderFrame(
      List<SimDetection> detections,
      List<ProjectedFieldPoint> projectedFieldPoints,
      int[] clusterLabels,
      List<ClusterScore> clusterScores) {
    frameMat.setTo(BACKGROUND_COLOR);

    for (SimDetection detection : detections) {
      Imgproc.circle(
          frameMat,
          new Point(detection.xPx(), detection.yPx()),
          (int) Math.round(detection.radiusPx()),
          RAW_DETECTION_COLOR,
          1);
    }

    Map<Integer, Integer> clusterByPointId = new HashMap<>();
    for (int i = 0; i < projectedFieldPoints.size(); i++) {
      clusterByPointId.put(projectedFieldPoints.get(i).pointId(), clusterLabels[i]);
    }

    for (ProjectedFieldPoint point : projectedFieldPoints) {
      int clusterId = clusterByPointId.getOrDefault(point.pointId(), FuelDbscanClusterer.NOISE);
      Scalar color =
          clusterId >= 0 ? CLUSTER_COLORS[clusterId % CLUSTER_COLORS.length] : NOISE_COLOR;
      Point center = new Point(point.xPx(), point.yPx());

      Imgproc.circle(frameMat, center, (int) Math.round(point.radiusPx()), color, 2);
      Imgproc.putText(
          frameMat,
          "P" + point.pointId(),
          new Point(point.xPx() + point.radiusPx() + 4.0, point.yPx() - 4.0),
          Imgproc.FONT_HERSHEY_SIMPLEX,
          0.45,
          color,
          1);
    }

    Imgproc.putText(
        frameMat,
        "Fuel Pickup Vision Sim",
        new Point(12, 20),
        Imgproc.FONT_HERSHEY_SIMPLEX,
        0.55,
        HUD_TEXT_COLOR,
        2);
    Imgproc.putText(
        frameMat,
        "Browser: http://localhost:" + FuelPickupConstants.browserStreamPort + "/stream.mjpg",
        new Point(12, 40),
        Imgproc.FONT_HERSHEY_SIMPLEX,
        0.43,
        HUD_SUBTEXT_COLOR,
        1);
    Imgproc.putText(
        frameMat,
        "Detections: " + detections.size() + "  Points: " + projectedFieldPoints.size(),
        new Point(12, 58),
        Imgproc.FONT_HERSHEY_SIMPLEX,
        0.43,
        HUD_SUBTEXT_COLOR,
        1);
    Imgproc.putText(
        frameMat,
        String.format(
            Locale.US,
            "Fuel: %d proj:%d rej:{proj=%d range=%d drop=%d noise=%d}",
            lastDetectionStats.totalFuelCount(),
            lastDetectionStats.projectedInFrameCount(),
            lastDetectionStats.projectionRejectedCount(),
            lastDetectionStats.rangeRejectedCount(),
            lastDetectionStats.dropRejectedCount(),
            lastDetectionStats.noisyOutOfFrameRejectedCount()),
        new Point(12, 76),
        Imgproc.FONT_HERSHEY_SIMPLEX,
        0.34,
        HUD_SUBTEXT_COLOR,
        1);
    Imgproc.putText(
        frameMat,
        String.format(
            Locale.US,
            "Depth(m): [%.2f, %.2f] maxRange=%.2f",
            lastDetectionStats.minProjectedDepthMeters(),
            lastDetectionStats.maxProjectedDepthMeters(),
            FuelPickupConstants.maxDetectionRangeMeters),
        new Point(12, 90),
        Imgproc.FONT_HERSHEY_SIMPLEX,
        0.34,
        HUD_SUBTEXT_COLOR,
        1);

    if (clusterScores.isEmpty()) {
      Imgproc.putText(
          frameMat,
          "No clusters",
          new Point(12, 108),
          Imgproc.FONT_HERSHEY_SIMPLEX,
          0.5,
          HUD_SUBTEXT_COLOR,
          1);
    } else {
      int y = 108;
      for (int i = 0; i < clusterScores.size(); i++) {
        ClusterScore score = clusterScores.get(i);
        boolean best = i == 0;
        Scalar lineColor =
            best ? BEST_COLOR : CLUSTER_COLORS[score.clusterId() % CLUSTER_COLORS.length];
        String line =
            String.format(
                Locale.US,
                "%sC%d n=%d d=%.2f s=%.2f",
                best ? ">" : " ",
                score.clusterId(),
                score.count(),
                score.distanceMeters(),
                score.score());
        Imgproc.putText(
            frameMat, line, new Point(12, y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.45, lineColor, 1);
        y += 18;
      }
    }

    videoSource.putFrame(frameMat);
  }

  private void logDetections(List<SimDetection> detections) {
    Logger.recordOutput("FuelPickup/Detections/Count", detections.size());
    Logger.recordOutput(
        "FuelPickup/Detections/SimInputFuelCount", lastDetectionStats.totalFuelCount());
    Logger.recordOutput(
        "FuelPickup/Detections/ProjectedInFrameCount", lastDetectionStats.projectedInFrameCount());
    Logger.recordOutput(
        "FuelPickup/Detections/ProjectionRejectedCount",
        lastDetectionStats.projectionRejectedCount());
    Logger.recordOutput(
        "FuelPickup/Detections/RangeRejectedCount", lastDetectionStats.rangeRejectedCount());
    Logger.recordOutput(
        "FuelPickup/Detections/DropRejectedCount", lastDetectionStats.dropRejectedCount());
    Logger.recordOutput(
        "FuelPickup/Detections/NoisyOutOfFrameRejectedCount",
        lastDetectionStats.noisyOutOfFrameRejectedCount());
    Logger.recordOutput(
        "FuelPickup/Detections/ProjectedDepthRangeMeters",
        new double[] {
          lastDetectionStats.minProjectedDepthMeters(), lastDetectionStats.maxProjectedDepthMeters()
        });
    Logger.recordOutput(
        "FuelPickup/Detections/CentersPx",
        detections.stream()
            .map((detection) -> new Translation2d(detection.xPx(), detection.yPx()))
            .toArray(Translation2d[]::new));
    Logger.recordOutput(
        "FuelPickup/Detections/Confidence",
        detections.stream().mapToDouble(SimDetection::confidence).toArray());
    Logger.recordOutput(
        "FuelPickup/Detections/RadiusPx",
        detections.stream().mapToDouble(SimDetection::radiusPx).toArray());
  }

  private void logFieldEstimates(List<ProjectedFieldPoint> projectedFieldPoints) {
    Logger.recordOutput(
        "FuelPickup/FieldEstimates/PointIds",
        projectedFieldPoints.stream().mapToInt(ProjectedFieldPoint::pointId).toArray());
    Logger.recordOutput(
        "FuelPickup/FieldEstimates/EstimatedPoses",
        projectedFieldPoints.stream()
            .map(ProjectedFieldPoint::position)
            .toArray(Translation3d[]::new));
  }

  private void logClusters(
      int rawClusterCount,
      int stableClusterCount,
      List<ProjectedFieldPoint> projectedFieldPoints,
      int[] rawTrackClusterLabels,
      int[] stableTrackClusterLabels,
      Map<Integer, List<ProjectedFieldPoint>> stableClusterMap,
      List<ClusterScore> clusterScores,
      int bestClusterId) {
    Logger.recordOutput("FuelPickup/Clusters/RawCount", rawClusterCount);
    Logger.recordOutput("FuelPickup/Clusters/Count", stableClusterCount);
    Logger.recordOutput("FuelPickup/Clusters/ForcedZoneMode", false);
    Logger.recordOutput("FuelPickup/Clusters/CoordinateFrame", "field");
    Logger.recordOutput("FuelPickup/Clusters/RawTrackClusterIds", rawTrackClusterLabels);
    Logger.recordOutput("FuelPickup/Clusters/TrackClusterIds", stableTrackClusterLabels);
    Logger.recordOutput(
        "FuelPickup/Clusters/FieldPointsUsed",
        projectedFieldPoints.stream()
            .map(ProjectedFieldPoint::position)
            .toArray(Translation3d[]::new));
    Logger.recordOutput(
        "FuelPickup/Clusters/PointIds",
        projectedFieldPoints.stream().mapToInt(ProjectedFieldPoint::pointId).toArray());
    Logger.recordOutput(
        "FuelPickup/Clusters/Centroids",
        clusterScores.stream().map((score) -> score.centroid()).toArray(Translation3d[]::new));
    Logger.recordOutput(
        "FuelPickup/Selection/ClusterIds",
        clusterScores.stream().mapToInt(ClusterScore::clusterId).toArray());
    Logger.recordOutput(
        "FuelPickup/Selection/ClusterCounts",
        clusterScores.stream().mapToInt(ClusterScore::count).toArray());
    Logger.recordOutput(
        "FuelPickup/Selection/ClusterDistancesMeters",
        clusterScores.stream().mapToDouble(ClusterScore::distanceMeters).toArray());
    Logger.recordOutput(
        "FuelPickup/Selection/ClusterScores",
        clusterScores.stream().mapToDouble(ClusterScore::score).toArray());
    Logger.recordOutput("FuelPickup/Selection/BestClusterId", bestClusterId);

    Translation3d bestClusterCentroid =
        clusterScores.isEmpty() ? new Translation3d() : clusterScores.get(0).centroid();
    Logger.recordOutput("FuelPickup/Selection/BestClusterCentroid", bestClusterCentroid);

    for (int i = 0; i < FuelPickupConstants.maxLoggedClusters; i++) {
      List<ProjectedFieldPoint> points = stableClusterMap.getOrDefault(i, List.of());
      Logger.recordOutput(
          "FuelPickup/Clusters/Cluster" + i,
          points.stream().map(ProjectedFieldPoint::position).toArray(Translation3d[]::new));
    }
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
