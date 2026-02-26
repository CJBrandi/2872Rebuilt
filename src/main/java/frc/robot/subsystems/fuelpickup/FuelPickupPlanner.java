package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fuelpickup.FuelBallTracker.Detection;
import frc.robot.subsystems.fuelpickup.FuelBallTracker.TrackEstimate;
import frc.robot.subsystems.fuelpickup.FuelBallTracker.TrackerOutput;
import frc.robot.subsystems.fuelpickup.FuelDbscanClusterer.ClusterResult;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSim.FuelState;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.Set;
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
 * <p>Pipeline: Simulated detections -> tracking -> field projection -> DBSCAN clusters -> cluster
 * scoring.
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

  private record TrackedFieldPoint(int trackId, Translation3d position, double xPx, double yPx) {}

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
  private final FuelBallTracker tracker =
      new FuelBallTracker(
          FuelPickupConstants.trackerHighConfidence,
          FuelPickupConstants.trackerLowConfidence,
          FuelPickupConstants.trackerMatchDistancePx,
          FuelPickupConstants.trackerSecondPassDistancePx,
          FuelPickupConstants.trackerSpawnSuppressionDistancePx,
          FuelPickupConstants.trackerMinConfirmHits,
          FuelPickupConstants.trackerMaxMissedFrames);
  private final FuelDbscanClusterer clusterer =
      new FuelDbscanClusterer(
          FuelPickupConstants.dbscanEpsilonMeters, FuelPickupConstants.dbscanMinPoints);
  private final FuelClusterStabilizer clusterStabilizer =
      new FuelClusterStabilizer(
          FuelPickupConstants.clusterAssociationDistanceMeters,
          FuelPickupConstants.clusterMaxMissedFrames,
          FuelPickupConstants.clusterCentroidEmaAlpha);
  private final Map<Integer, Translation3d> filteredTrackFieldPoints = new HashMap<>();
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

    List<Detection> detections = simulateDetections(fuelStates, cameraPose);
    logDetections(detections);

    TrackerOutput trackerOutput = tracker.update(detections, Timer.getFPGATimestamp());
    logTracking(trackerOutput);

    List<TrackedFieldPoint> trackedFieldPoints =
        estimateFieldPoints(trackerOutput.confirmedTracks(), cameraPose);
    logFieldEstimates(trackedFieldPoints);

    List<Translation3d> estimatedPositions =
        trackedFieldPoints.stream().map(TrackedFieldPoint::position).toList();
    ClusterResult clusterResult = clusterer.cluster(estimatedPositions);
    Map<Integer, List<TrackedFieldPoint>> rawClusterMap =
        buildClusterMap(trackedFieldPoints, clusterResult.labels());
    Map<Integer, Translation3d> rawClusterCentroids = buildClusterCentroids(rawClusterMap);
    Map<Integer, Integer> rawToStableClusterIds = clusterStabilizer.update(rawClusterCentroids);
    Map<Integer, List<TrackedFieldPoint>> stableClusterMap =
        remapClusterMap(rawClusterMap, rawToStableClusterIds);
    int[] stableTrackClusterLabels =
        remapTrackClusterLabels(clusterResult.labels(), rawToStableClusterIds);

    List<ClusterScore> clusterScores = scoreClusters(stableClusterMap, robotPose);
    int bestClusterId = selectBestCluster(clusterScores);

    logClusters(
        clusterResult.clusterCount(),
        stableClusterMap.size(),
        trackedFieldPoints,
        clusterResult.labels(),
        stableTrackClusterLabels,
        stableClusterMap,
        clusterScores,
        bestClusterId);
    renderFrame(
        detections,
        trackerOutput.allTracks(),
        trackedFieldPoints,
        stableTrackClusterLabels,
        clusterScores);
  }

  private List<Detection> simulateDetections(FuelState[] fuelStates, Pose3d cameraPose) {
    List<Detection> detections = new ArrayList<>();
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
      detections.add(new Detection(xPx, yPx, radiusPx, confidence));
    }

    detections.sort(Comparator.comparingDouble(Detection::confidence).reversed());
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

  private List<TrackedFieldPoint> estimateFieldPoints(
      List<TrackEstimate> confirmedTracks, Pose3d cameraPose) {
    List<TrackedFieldPoint> points = new ArrayList<>();
    Set<Integer> activeTrackIds = new HashSet<>();
    for (TrackEstimate track : confirmedTracks) {
      Translation3d fieldPoint =
          cameraModel
              .backProjectToPlane(
                  track.xPx(), track.yPx(), cameraPose, FuelSim.getFuelRadiusMeters())
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
      activeTrackIds.add(track.id());
      Translation3d filteredPoint = filterFieldPoint(track.id(), fieldPoint);
      points.add(new TrackedFieldPoint(track.id(), filteredPoint, track.xPx(), track.yPx()));
    }
    filteredTrackFieldPoints.keySet().removeIf((trackId) -> !activeTrackIds.contains(trackId));
    return points;
  }

  private Map<Integer, List<TrackedFieldPoint>> buildClusterMap(
      List<TrackedFieldPoint> trackedFieldPoints, int[] labels) {
    Map<Integer, List<TrackedFieldPoint>> clusters = new HashMap<>();
    for (int i = 0; i < trackedFieldPoints.size(); i++) {
      int label = labels[i];
      if (label < 0) {
        continue;
      }
      clusters.computeIfAbsent(label, ignored -> new ArrayList<>()).add(trackedFieldPoints.get(i));
    }
    return clusters;
  }

  private Map<Integer, Translation3d> buildClusterCentroids(
      Map<Integer, List<TrackedFieldPoint>> clusterMap) {
    Map<Integer, Translation3d> centroids = new HashMap<>();
    for (Map.Entry<Integer, List<TrackedFieldPoint>> entry : clusterMap.entrySet()) {
      centroids.put(entry.getKey(), average(entry.getValue()));
    }
    return centroids;
  }

  private Map<Integer, List<TrackedFieldPoint>> remapClusterMap(
      Map<Integer, List<TrackedFieldPoint>> rawClusterMap,
      Map<Integer, Integer> rawToStableClusterIds) {
    Map<Integer, List<TrackedFieldPoint>> stableClusters = new HashMap<>();
    for (Map.Entry<Integer, List<TrackedFieldPoint>> entry : rawClusterMap.entrySet()) {
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
      Map<Integer, List<TrackedFieldPoint>> clusterMap, Pose2d robotPose) {
    List<ClusterScore> scores = new ArrayList<>();
    Translation2d robotTranslation = robotPose.getTranslation();
    for (Map.Entry<Integer, List<TrackedFieldPoint>> entry : clusterMap.entrySet()) {
      int clusterId = entry.getKey();
      List<TrackedFieldPoint> points = entry.getValue();
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

  private Translation3d average(List<TrackedFieldPoint> points) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (TrackedFieldPoint point : points) {
      x += point.position().getX();
      y += point.position().getY();
      z += point.position().getZ();
    }
    double invCount = 1.0 / points.size();
    return new Translation3d(x * invCount, y * invCount, z * invCount);
  }

  private void renderFrame(
      List<Detection> detections,
      List<TrackEstimate> tracks,
      List<TrackedFieldPoint> trackedFieldPoints,
      int[] clusterLabels,
      List<ClusterScore> clusterScores) {
    frameMat.setTo(BACKGROUND_COLOR);

    for (Detection detection : detections) {
      Imgproc.circle(
          frameMat,
          new Point(detection.xPx(), detection.yPx()),
          (int) Math.round(detection.radiusPx()),
          RAW_DETECTION_COLOR,
          1);
    }

    Map<Integer, Integer> clusterByTrackId = new HashMap<>();
    for (int i = 0; i < trackedFieldPoints.size(); i++) {
      clusterByTrackId.put(trackedFieldPoints.get(i).trackId(), clusterLabels[i]);
    }

    for (TrackEstimate track : tracks) {
      int clusterId = clusterByTrackId.getOrDefault(track.id(), FuelDbscanClusterer.NOISE);
      Scalar color =
          clusterId >= 0 ? CLUSTER_COLORS[clusterId % CLUSTER_COLORS.length] : NOISE_COLOR;
      Point center = new Point(track.xPx(), track.yPx());

      Imgproc.circle(frameMat, center, (int) Math.round(track.radiusPx()), color, 2);
      Imgproc.putText(
          frameMat,
          "T" + track.id(),
          new Point(track.xPx() + track.radiusPx() + 4.0, track.yPx() - 4.0),
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
        "Detections: " + detections.size() + "  Tracks: " + tracks.size(),
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

  private void logDetections(List<Detection> detections) {
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
        detections.stream().mapToDouble(Detection::confidence).toArray());
    Logger.recordOutput(
        "FuelPickup/Detections/RadiusPx",
        detections.stream().mapToDouble(Detection::radiusPx).toArray());
  }

  private void logTracking(TrackerOutput trackerOutput) {
    Logger.recordOutput(
        "FuelPickup/Tracking/AllTrackIds",
        trackerOutput.allTracks().stream().mapToInt(TrackEstimate::id).toArray());
    Logger.recordOutput(
        "FuelPickup/Tracking/ConfirmedTrackIds",
        trackerOutput.confirmedTracks().stream().mapToInt(TrackEstimate::id).toArray());
    Logger.recordOutput(
        "FuelPickup/Tracking/AllTrackCentersPx",
        trackerOutput.allTracks().stream()
            .map((track) -> new Translation2d(track.xPx(), track.yPx()))
            .toArray(Translation2d[]::new));
  }

  private void logFieldEstimates(List<TrackedFieldPoint> trackedFieldPoints) {
    Logger.recordOutput(
        "FuelPickup/FieldEstimates/Tracks",
        trackedFieldPoints.stream().mapToInt(TrackedFieldPoint::trackId).toArray());
    Logger.recordOutput(
        "FuelPickup/FieldEstimates/EstimatedPoses",
        trackedFieldPoints.stream().map(TrackedFieldPoint::position).toArray(Translation3d[]::new));
  }

  private void logClusters(
      int rawClusterCount,
      int stableClusterCount,
      List<TrackedFieldPoint> trackedFieldPoints,
      int[] rawTrackClusterLabels,
      int[] stableTrackClusterLabels,
      Map<Integer, List<TrackedFieldPoint>> stableClusterMap,
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
        trackedFieldPoints.stream().map(TrackedFieldPoint::position).toArray(Translation3d[]::new));
    Logger.recordOutput(
        "FuelPickup/Clusters/TrackIds",
        trackedFieldPoints.stream().mapToInt(TrackedFieldPoint::trackId).toArray());
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
      List<TrackedFieldPoint> points = stableClusterMap.getOrDefault(i, List.of());
      Logger.recordOutput(
          "FuelPickup/Clusters/Cluster" + i,
          points.stream().map(TrackedFieldPoint::position).toArray(Translation3d[]::new));
    }
  }

  private Translation3d filterFieldPoint(int trackId, Translation3d fieldPoint) {
    Translation3d previous = filteredTrackFieldPoints.get(trackId);
    if (previous == null) {
      filteredTrackFieldPoints.put(trackId, fieldPoint);
      return fieldPoint;
    }
    double alpha = clamp(FuelPickupConstants.fieldEstimateEmaAlpha, 0.0, 1.0);
    Translation3d filtered = previous.times(1.0 - alpha).plus(fieldPoint.times(alpha));
    filteredTrackFieldPoints.put(trackId, filtered);
    return filtered;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
