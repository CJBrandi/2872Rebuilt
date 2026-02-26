package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.fuelpickup.FuelCameraModel;
import frc.robot.subsystems.fuelpickup.FuelPickupConstants;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSim.FuelState;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/** Detection IO implementation for simulation. Produces pixel detections from FuelSim state. */
public class DetectionIOSim implements DetectionIO {
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

  private final Supplier<Pose2d> robotPoseSupplier;
  private final FuelSim fuelSim = FuelSim.getInstance();
  private final FuelCameraModel cameraModel =
      new FuelCameraModel(
          FuelPickupConstants.frameWidthPx,
          FuelPickupConstants.frameHeightPx,
          FuelPickupConstants.horizontalFovDeg,
          FuelPickupConstants.verticalFovDeg);
  private final Random random = new Random(2026);
  private final CvSource videoSource;
  private final MjpegServer browserServer;
  private final Mat frameMat =
      new Mat(FuelPickupConstants.frameHeightPx, FuelPickupConstants.frameWidthPx, CvType.CV_8UC3);

  public DetectionIOSim(Supplier<Pose2d> robotPoseSupplier) {
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
        "Detection/Video/BrowserUrl",
        "http://localhost:" + FuelPickupConstants.browserStreamPort + "/stream.mjpg");
    Logger.recordOutput("Detection/Video/CameraName", FuelPickupConstants.streamName);
  }

  @Override
  public void updateInputs(DetectionIOInputs inputs) {
    inputs.connected = true;
    inputs.timestampSeconds = Timer.getFPGATimestamp();

    Pose3d cameraPose =
        new Pose3d(robotPoseSupplier.get()).plus(VisionConstants.robotToDetectionCamera);
    FuelState[] fuelStates = fuelSim.getFuelStates();

    List<PixelDetection> detections = new ArrayList<>();
    for (FuelState fuelState : fuelStates) {
      FuelCameraModel.PixelObservation projected =
          cameraModel
              .project(fuelState.position(), cameraPose, FuelSim.getFuelRadiusMeters())
              .orElse(null);
      if (projected == null
          || projected.depthMeters() > FuelPickupConstants.maxDetectionRangeMeters) {
        continue;
      }

      double dropRate = interpolationByRange(projected.depthMeters());
      if (random.nextDouble() < dropRate) {
        continue;
      }

      double bboxWidthPx = Math.max(4.0, 2.0 * projected.radiusPx());
      double bboxHeightPx = bboxWidthPx;

      double xPx = projected.xPx() + random.nextGaussian() * FuelPickupConstants.pixelNoiseStdDevPx;
      double yPx =
          projected.yPx()
              + projected.radiusPx()
              + random.nextGaussian() * FuelPickupConstants.pixelNoiseStdDevPx;
      if (xPx < 0.0
          || xPx >= FuelPickupConstants.frameWidthPx
          || yPx < 0.0
          || yPx >= FuelPickupConstants.frameHeightPx) {
        continue;
      }

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

      detections.add(new PixelDetection(xPx, yPx, bboxWidthPx, bboxHeightPx, confidence));
    }

    detections.sort(Comparator.comparingDouble(PixelDetection::confidence).reversed());
    inputs.detections = detections.toArray(PixelDetection[]::new);
  }

  @Override
  public void publishSimFrame(SimFrameData frameData) {
    frameMat.setTo(BACKGROUND_COLOR);

    for (PixelDetection detection : frameData.detections()) {
      int width = (int) Math.round(Math.max(2.0, detection.bboxWidthPx()));
      int height = (int) Math.round(Math.max(2.0, detection.bboxHeightPx()));
      int left = (int) Math.round(detection.xPx() - width * 0.5);
      int top = (int) Math.round(detection.yPx() - height);
      Rect box = new Rect(left, top, width, height);
      Imgproc.rectangle(frameMat, box, RAW_DETECTION_COLOR, 1);
      Imgproc.circle(
          frameMat, new Point(detection.xPx(), detection.yPx()), 2, RAW_DETECTION_COLOR, 1);
    }

    for (TrackOverlay track : frameData.tracks()) {
      int clusterId = track.clusterId();
      Scalar color =
          clusterId >= 0 ? CLUSTER_COLORS[clusterId % CLUSTER_COLORS.length] : NOISE_COLOR;
      Point center = new Point(track.xPx(), track.yPx());

      Imgproc.circle(frameMat, center, (int) Math.round(track.radiusPx()), color, 2);
      Imgproc.putText(
          frameMat,
          "T" + track.trackId(),
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
        "Detections: " + frameData.detections().length + "  Tracks: " + frameData.tracks().length,
        new Point(12, 58),
        Imgproc.FONT_HERSHEY_SIMPLEX,
        0.43,
        HUD_SUBTEXT_COLOR,
        1);

    if (frameData.clusters().length == 0) {
      Imgproc.putText(
          frameMat,
          "No clusters",
          new Point(12, 78),
          Imgproc.FONT_HERSHEY_SIMPLEX,
          0.5,
          HUD_SUBTEXT_COLOR,
          1);
    } else {
      int y = 78;
      for (ClusterOverlay cluster : frameData.clusters()) {
        Scalar lineColor =
            cluster.best()
                ? BEST_COLOR
                : CLUSTER_COLORS[cluster.clusterId() % CLUSTER_COLORS.length];
        String line =
            String.format(
                Locale.US,
                "%sC%d n=%d d=%.2f s=%.2f",
                cluster.best() ? ">" : " ",
                cluster.clusterId(),
                cluster.count(),
                cluster.distanceMeters(),
                cluster.score());
        Imgproc.putText(
            frameMat, line, new Point(12, y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.45, lineColor, 1);
        y += 18;
      }
    }

    videoSource.putFrame(frameMat);
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

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
