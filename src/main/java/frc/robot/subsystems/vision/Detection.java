package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.fuelpickup.FuelCameraModel;
import frc.robot.subsystems.fuelpickup.FuelClusterStabilizer;
import frc.robot.subsystems.fuelpickup.FuelDbscanClusterer;
import frc.robot.subsystems.fuelpickup.FuelDbscanClusterer.ClusterResult;
import frc.robot.subsystems.fuelpickup.FuelPickupConstants;
import frc.robot.util.FuelSim;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Fuel detection pipeline subsystem.
 *
 * <p>Pipeline:
 *
 * <p>1) Preferred path (if available): field-relative clusters from IO -> cluster scoring.
 *
 * <p>2) Fallback path: pixel detections (IO) -> field projection -> DBSCAN clustering -> cluster
 * scoring.
 */
public class Detection extends SubsystemBase {
  private record ProjectedFieldPoint(
      int pointId, Translation3d position, double xPx, double yPx, double radiusPx) {}

  private record ClusterScore(
      int clusterId,
      Translation3d centroid,
      int count,
      double distanceMeters,
      double score,
      boolean preferred) {}

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
  private final FuelDbscanClusterer clusterer =
      new FuelDbscanClusterer(
          FuelPickupConstants.dbscanEpsilonMeters, FuelPickupConstants.dbscanMinPoints);
  private final FuelClusterStabilizer clusterStabilizer =
      new FuelClusterStabilizer(
          FuelPickupConstants.clusterAssociationDistanceMeters,
          FuelPickupConstants.clusterMaxMissedFrames,
          FuelPickupConstants.clusterCentroidEmaAlpha);

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
    Logger.recordOutput("Detection/CameraPose", cameraPose);

    List<DetectionIO.PixelDetection> pixelDetections = List.of(inputs.detections);
    logDetections(pixelDetections);
    List<DetectionIO.FieldClusterDetection> fieldClusters = List.of(inputs.fieldClusters);
    Logger.recordOutput("Detection/Clusters/ExternalCount", fieldClusters.size());

    if (!fieldClusters.isEmpty()) {
      Logger.recordOutput("Detection/Clusters/Source", "limelight_preclustered");
      processPreclusteredInputs(fieldClusters, pixelDetections, robotPose);
      return;
    }
    Logger.recordOutput("Detection/Clusters/Source", "rio_clustered");

    List<ProjectedFieldPoint> projectedFieldPoints =
        estimateFieldPoints(pixelDetections, cameraPose);
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

    publishBestClusterToRobotState(clusterScores);

    logClusters(
        clusterResult.clusterCount(),
        stableClusterMap.size(),
        projectedFieldPoints,
        clusterResult.labels(),
        stablePointClusterLabels,
        stableClusterMap,
        clusterScores,
        bestClusterId);
    io.publishSimFrame(
        new DetectionIO.SimFrameData(
            pixelDetections.toArray(DetectionIO.PixelDetection[]::new),
            buildTrackOverlays(projectedFieldPoints, stablePointClusterLabels),
            buildClusterOverlays(clusterScores)));
  }

  private void processPreclusteredInputs(
      List<DetectionIO.FieldClusterDetection> fieldClusters,
      List<DetectionIO.PixelDetection> pixelDetections,
      Pose2d robotPose) {
    List<ProjectedFieldPoint> projectedFieldPoints = new ArrayList<>(fieldClusters.size());
    int[] clusterLabels = new int[fieldClusters.size()];
    Map<Integer, List<ProjectedFieldPoint>> clusterMap = new HashMap<>();

    for (int i = 0; i < fieldClusters.size(); i++) {
      DetectionIO.FieldClusterDetection cluster = fieldClusters.get(i);
      Translation3d centroid =
          new Translation3d(cluster.xMeters(), cluster.yMeters(), FuelSim.getFuelRadiusMeters());
      ProjectedFieldPoint point = new ProjectedFieldPoint(i, centroid, 0.0, 0.0, 1.0);
      projectedFieldPoints.add(point);
      clusterLabels[i] = cluster.clusterId();
      clusterMap.computeIfAbsent(cluster.clusterId(), ignored -> new ArrayList<>()).add(point);
    }

    logFieldEstimates(projectedFieldPoints);
    List<ClusterScore> clusterScores = scorePreclusteredClusters(fieldClusters, robotPose);
    int bestClusterId = selectBestCluster(clusterScores);
    publishBestClusterToRobotState(clusterScores);
    logClusters(
        fieldClusters.size(),
        fieldClusters.size(),
        projectedFieldPoints,
        clusterLabels,
        clusterLabels,
        clusterMap,
        clusterScores,
        bestClusterId);

    io.publishSimFrame(
        new DetectionIO.SimFrameData(
            pixelDetections.toArray(DetectionIO.PixelDetection[]::new),
            new DetectionIO.TrackOverlay[0],
            buildClusterOverlays(clusterScores)));
  }

  private List<ProjectedFieldPoint> estimateFieldPoints(
      List<DetectionIO.PixelDetection> pixelDetections, Pose3d cameraPose) {
    List<ProjectedFieldPoint> points = new ArrayList<>();
    for (int i = 0; i < pixelDetections.size(); i++) {
      DetectionIO.PixelDetection detection = pixelDetections.get(i);
      Translation3d groundPoint =
          cameraModel
              .backProjectToPlane(detection.xPx(), detection.yPx(), cameraPose, 0.0)
              .orElse(null);
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

      points.add(
          new ProjectedFieldPoint(
              i,
              fieldPoint,
              detection.xPx(),
              detection.yPx(),
              Math.max(1.0, 0.5 * detection.bboxWidthPx())));
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
      scores.add(new ClusterScore(clusterId, centroid, count, distance, score, false));
    }
    scores.sort(Comparator.comparingDouble(ClusterScore::score).reversed());
    return scores;
  }

  private List<ClusterScore> scorePreclusteredClusters(
      List<DetectionIO.FieldClusterDetection> fieldClusters, Pose2d robotPose) {
    List<ClusterScore> scores = new ArrayList<>();
    Translation2d robotTranslation = robotPose.getTranslation();
    for (DetectionIO.FieldClusterDetection cluster : fieldClusters) {
      Translation3d centroid =
          new Translation3d(cluster.xMeters(), cluster.yMeters(), FuelSim.getFuelRadiusMeters());
      double distance = robotTranslation.getDistance(centroid.toTranslation2d());
      int count = Math.max(0, (int) Math.round(cluster.count()));
      double score = cluster.score();
      if (!Double.isFinite(score)) {
        score =
            FuelPickupConstants.countWeight * count - FuelPickupConstants.distanceWeight * distance;
      }
      scores.add(
          new ClusterScore(cluster.clusterId(), centroid, count, distance, score, cluster.best()));
    }
    scores.sort(
        Comparator.comparing(ClusterScore::preferred)
            .reversed()
            .thenComparing(Comparator.comparingDouble(ClusterScore::score).reversed()));
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

  private DetectionIO.TrackOverlay[] buildTrackOverlays(
      List<ProjectedFieldPoint> projectedFieldPoints, int[] clusterLabels) {
    Map<Integer, Integer> clusterByPointId = new HashMap<>();
    for (int i = 0; i < projectedFieldPoints.size(); i++) {
      clusterByPointId.put(projectedFieldPoints.get(i).pointId(), clusterLabels[i]);
    }

    return projectedFieldPoints.stream()
        .map(
            (point) ->
                new DetectionIO.TrackOverlay(
                    point.pointId(),
                    point.xPx(),
                    point.yPx(),
                    point.radiusPx(),
                    clusterByPointId.getOrDefault(point.pointId(), FuelDbscanClusterer.NOISE)))
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

  private void logFieldEstimates(List<ProjectedFieldPoint> projectedFieldPoints) {
    Logger.recordOutput(
        "Detection/FieldEstimates/PointIds",
        projectedFieldPoints.stream().mapToInt(ProjectedFieldPoint::pointId).toArray());
    Logger.recordOutput(
        "Detection/FieldEstimates/EstimatedPoses",
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
    Logger.recordOutput("Detection/Clusters/RawCount", rawClusterCount);
    Logger.recordOutput("Detection/Clusters/Count", stableClusterCount);
    Logger.recordOutput("Detection/Clusters/ForcedZoneMode", false);
    Logger.recordOutput("Detection/Clusters/CoordinateFrame", "field");
    Logger.recordOutput("Detection/Clusters/RawTrackClusterIds", rawTrackClusterLabels);
    Logger.recordOutput("Detection/Clusters/TrackClusterIds", stableTrackClusterLabels);
    Logger.recordOutput(
        "Detection/Clusters/FieldPointsUsed",
        projectedFieldPoints.stream()
            .map(ProjectedFieldPoint::position)
            .toArray(Translation3d[]::new));
    Logger.recordOutput(
        "Detection/Clusters/PointIds",
        projectedFieldPoints.stream().mapToInt(ProjectedFieldPoint::pointId).toArray());
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
      List<ProjectedFieldPoint> points = stableClusterMap.getOrDefault(i, List.of());
      Logger.recordOutput(
          "Detection/Clusters/Cluster" + i,
          points.stream().map(ProjectedFieldPoint::position).toArray(Translation3d[]::new));
    }
  }
}
