package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.fuelpickup.FuelBallTracker;
import frc.robot.subsystems.fuelpickup.FuelBallTracker.TrackEstimate;
import frc.robot.subsystems.fuelpickup.FuelBallTracker.TrackerOutput;
import frc.robot.subsystems.fuelpickup.FuelCameraModel;
import frc.robot.subsystems.fuelpickup.FuelClusterStabilizer;
import frc.robot.subsystems.fuelpickup.FuelDbscanClusterer;
import frc.robot.subsystems.fuelpickup.FuelDbscanClusterer.ClusterResult;
import frc.robot.subsystems.fuelpickup.FuelPickupConstants;
import frc.robot.util.FuelSim;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Fuel detection pipeline subsystem.
 *
 * <p>Pipeline: Pixel detections (IO) -> tracking -> field projection -> DBSCAN clustering ->
 * cluster scoring.
 */
public class Detection extends SubsystemBase {
  private record TrackedFieldPoint(int trackId, Translation3d position, double xPx, double yPx) {}

  private record ClusterScore(
      int clusterId, Translation3d centroid, int count, double distanceMeters, double score) {}

  private final Supplier<Pose2d> robotPoseSupplier;
  private final DetectionIO io;
  private final DetectionIOInputsAutoLogged inputs = new DetectionIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("Fuel detection camera is disconnected.", AlertType.kWarning);

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

  public Detection(Supplier<Pose2d> robotPoseSupplier, DetectionIO io) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Detection/IO", inputs);
    disconnectedAlert.set(!inputs.connected);

    Pose2d robotPose = robotPoseSupplier.get();
    Pose3d cameraPose = new Pose3d(robotPose).plus(VisionConstants.robotToDetectionCamera);
    Pose3d cameraRobotRelativePose = new Pose3d().plus(VisionConstants.robotToDetectionCamera);
    Translation3d cameraRobotRelativeTranslation = cameraRobotRelativePose.getTranslation();
    Logger.recordOutput("Detection/CameraPose", cameraPose);

    List<DetectionIO.PixelDetection> pixelDetections = List.of(inputs.detections);
    logDetections(pixelDetections);

    double timestampSecs =
        inputs.timestampSeconds > 0.0 ? inputs.timestampSeconds : Timer.getFPGATimestamp();
    List<FuelBallTracker.Detection> trackerDetections =
        pixelDetections.stream().map(this::toTrackerDetection).toList();

    TrackerOutput trackerOutput = tracker.update(trackerDetections, timestampSecs);
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

    publishBestClusterToRobotState(clusterScores);

    logClusters(
        clusterResult.clusterCount(),
        stableClusterMap.size(),
        trackedFieldPoints,
        clusterResult.labels(),
        stableTrackClusterLabels,
        stableClusterMap,
        clusterScores,
        bestClusterId);
    io.publishSimFrame(
        new DetectionIO.SimFrameData(
            pixelDetections.toArray(DetectionIO.PixelDetection[]::new),
            buildTrackOverlays(
                trackerOutput.allTracks(), trackedFieldPoints, stableTrackClusterLabels),
            buildClusterOverlays(clusterScores)));
  }

  private FuelBallTracker.Detection toTrackerDetection(DetectionIO.PixelDetection detection) {
    double inferredRadius = Math.max(1.0, 0.5 * detection.bboxWidthPx());
    return new FuelBallTracker.Detection(
        detection.xPx(), detection.yPx(), inferredRadius, detection.confidence());
  }

  private List<TrackedFieldPoint> estimateFieldPoints(
      List<TrackEstimate> confirmedTracks, Pose3d cameraPose) {
    List<TrackedFieldPoint> points = new ArrayList<>();
    Set<Integer> activeTrackIds = new HashSet<>();
    for (TrackEstimate track : confirmedTracks) {
      Translation3d groundPoint =
          cameraModel.backProjectToPlane(track.xPx(), track.yPx(), cameraPose, 0.0).orElse(null);
      if (groundPoint == null) {
        continue;
      }

      Translation3d fieldPoint =
          new Translation3d(groundPoint.getX(), groundPoint.getY(), FuelSim.getFuelRadiusMeters());
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

  private void publishBestClusterToRobotState(List<ClusterScore> scores) {
    if (scores.isEmpty()) {
      RobotState.getInstance().clearBestFuelCluster();
      return;
    }
    RobotState.getInstance().setBestFuelCluster(scores.get(0).centroid().toTranslation2d());
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

  private DetectionIO.TrackOverlay[] buildTrackOverlays(
      List<TrackEstimate> tracks, List<TrackedFieldPoint> trackedFieldPoints, int[] clusterLabels) {
    Map<Integer, Integer> clusterByTrackId = new HashMap<>();
    for (int i = 0; i < trackedFieldPoints.size(); i++) {
      clusterByTrackId.put(trackedFieldPoints.get(i).trackId(), clusterLabels[i]);
    }

    return tracks.stream()
        .map(
            (track) ->
                new DetectionIO.TrackOverlay(
                    track.id(),
                    track.xPx(),
                    track.yPx(),
                    track.radiusPx(),
                    clusterByTrackId.getOrDefault(track.id(), FuelDbscanClusterer.NOISE)))
        .toArray(DetectionIO.TrackOverlay[]::new);
  }

  private DetectionIO.ClusterOverlay[] buildClusterOverlays(List<ClusterScore> clusterScores) {
    DetectionIO.ClusterOverlay[] overlays = new DetectionIO.ClusterOverlay[clusterScores.size()];
    for (int i = 0; i < clusterScores.size(); i++) {
      ClusterScore score = clusterScores.get(i);
      overlays[i] =
          new DetectionIO.ClusterOverlay(
              score.clusterId(), score.count(), score.distanceMeters(), score.score(), i == 0);
    }
    return overlays;
  }

  private void logDetections(List<DetectionIO.PixelDetection> detections) {
    Logger.recordOutput("Detection/Detections/Count", detections.size());
    Logger.recordOutput(
        "Detection/Detections/BottomPointsPx",
        detections.stream()
            .map((detection) -> new Translation2d(detection.xPx(), detection.yPx()))
            .toArray(Translation2d[]::new));
    Logger.recordOutput(
        "Detection/Detections/Confidence",
        detections.stream().mapToDouble(DetectionIO.PixelDetection::confidence).toArray());
    Logger.recordOutput(
        "Detection/Detections/BBoxWidthPx",
        detections.stream().mapToDouble(DetectionIO.PixelDetection::bboxWidthPx).toArray());
    Logger.recordOutput(
        "Detection/Detections/BBoxHeightPx",
        detections.stream().mapToDouble(DetectionIO.PixelDetection::bboxHeightPx).toArray());
  }

  private void logTracking(TrackerOutput trackerOutput) {
    Logger.recordOutput(
        "Detection/Tracking/AllTrackIds",
        trackerOutput.allTracks().stream().mapToInt(TrackEstimate::id).toArray());
    Logger.recordOutput(
        "Detection/Tracking/ConfirmedTrackIds",
        trackerOutput.confirmedTracks().stream().mapToInt(TrackEstimate::id).toArray());
    Logger.recordOutput(
        "Detection/Tracking/AllTrackCentersPx",
        trackerOutput.allTracks().stream()
            .map((track) -> new Translation2d(track.xPx(), track.yPx()))
            .toArray(Translation2d[]::new));
  }

  private void logFieldEstimates(List<TrackedFieldPoint> trackedFieldPoints) {
    Logger.recordOutput(
        "Detection/FieldEstimates/Tracks",
        trackedFieldPoints.stream().mapToInt(TrackedFieldPoint::trackId).toArray());
    Logger.recordOutput(
        "Detection/FieldEstimates/EstimatedPoses",
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
    Logger.recordOutput("Detection/Clusters/RawCount", rawClusterCount);
    Logger.recordOutput("Detection/Clusters/Count", stableClusterCount);
    Logger.recordOutput("Detection/Clusters/ForcedZoneMode", false);
    Logger.recordOutput("Detection/Clusters/CoordinateFrame", "field");
    Logger.recordOutput("Detection/Clusters/RawTrackClusterIds", rawTrackClusterLabels);
    Logger.recordOutput("Detection/Clusters/TrackClusterIds", stableTrackClusterLabels);
    Logger.recordOutput(
        "Detection/Clusters/FieldPointsUsed",
        trackedFieldPoints.stream().map(TrackedFieldPoint::position).toArray(Translation3d[]::new));
    Logger.recordOutput(
        "Detection/Clusters/TrackIds",
        trackedFieldPoints.stream().mapToInt(TrackedFieldPoint::trackId).toArray());
    Logger.recordOutput(
        "Detection/Clusters/Centroids",
        clusterScores.stream().map((score) -> score.centroid()).toArray(Translation3d[]::new));
    Logger.recordOutput(
        "Detection/Selection/ClusterIds",
        clusterScores.stream().mapToInt(ClusterScore::clusterId).toArray());
    Logger.recordOutput(
        "Detection/Selection/ClusterCounts",
        clusterScores.stream().mapToInt(ClusterScore::count).toArray());
    Logger.recordOutput(
        "Detection/Selection/ClusterDistancesMeters",
        clusterScores.stream().mapToDouble(ClusterScore::distanceMeters).toArray());
    Logger.recordOutput(
        "Detection/Selection/ClusterScores",
        clusterScores.stream().mapToDouble(ClusterScore::score).toArray());
    Logger.recordOutput("Detection/Selection/BestClusterId", bestClusterId);

    Translation3d bestClusterCentroid =
        clusterScores.isEmpty() ? new Translation3d() : clusterScores.get(0).centroid();
    Logger.recordOutput("Detection/Selection/BestClusterCentroid", bestClusterCentroid);

    for (int i = 0; i < FuelPickupConstants.maxLoggedClusters; i++) {
      List<TrackedFieldPoint> points = stableClusterMap.getOrDefault(i, List.of());
      Logger.recordOutput(
          "Detection/Clusters/Cluster" + i,
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
