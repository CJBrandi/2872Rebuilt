package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.fuelpickup.FuelPickupConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.target.pipeline.NeuralDetector;
import limelight.results.RawDetection;

/** Detection IO implementation for a real Limelight using YALL. */
public class DetectionIOLimeLight implements DetectionIO {
  private final String limelightName;
  private final String targetClassName;
  private final Limelight limelight;

  public DetectionIOLimeLight(String limelightName) {
    this(limelightName, "");
  }

  /**
   * @param limelightName NetworkTables name for the Limelight
   * @param targetClassName Class to filter on (empty to accept all classes)
   */
  public DetectionIOLimeLight(String limelightName, String targetClassName) {
    this.limelightName = limelightName;
    this.targetClassName = targetClassName == null ? "" : targetClassName.trim();
    this.limelight = new Limelight(limelightName);

    limelight
        .getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(new Pose3d().plus(VisionConstants.robotToDetectionCamera))
        .save();
  }

  @Override
  public void updateInputs(DetectionIOInputs inputs) {
    inputs.timestampSeconds = Timer.getFPGATimestamp();
    inputs.connected = Limelight.isAvailable(limelightName);

    Map<Integer, String> classNameById = new HashMap<>();
    Map<Integer, Double> confidenceById = new HashMap<>();

    LimelightResults latestResults =
        limelight
            .getLatestResults()
            .map(
                (result) -> {
                  for (NeuralDetector detector : result.targets_Detector) {
                    int classId = (int) Math.round(detector.classID);
                    classNameById.put(classId, detector.className);
                    confidenceById.put(classId, detector.confidence);
                  }
                  return result;
                })
            .orElse(null);

    List<PixelDetection> detections = new ArrayList<>();
    RawDetection[] rawDetections = limelight.getData().getRawDetections();
    for (RawDetection detection : rawDetections) {
      String className = classNameById.getOrDefault(detection.classId, "");
      if (!isClassAllowed(className)) {
        continue;
      }

      double minX =
          Math.min(
              Math.min(detection.corner0_X, detection.corner1_X),
              Math.min(detection.corner2_X, detection.corner3_X));
      double maxX =
          Math.max(
              Math.max(detection.corner0_X, detection.corner1_X),
              Math.max(detection.corner2_X, detection.corner3_X));
      double minY =
          Math.min(
              Math.min(detection.corner0_Y, detection.corner1_Y),
              Math.min(detection.corner2_Y, detection.corner3_Y));
      double maxY =
          Math.max(
              Math.max(detection.corner0_Y, detection.corner1_Y),
              Math.max(detection.corner2_Y, detection.corner3_Y));

      double bboxWidth = Math.max(2.0, maxX - minX);
      double bboxHeight = Math.max(2.0, maxY - minY);
      double xPx = (minX + maxX) * 0.5;
      double yPx = maxY;
      if (xPx < 0.0
          || xPx >= FuelPickupConstants.frameWidthPx
          || yPx < 0.0
          || yPx >= FuelPickupConstants.frameHeightPx) {
        continue;
      }

      double confidence =
          clamp(confidenceById.getOrDefault(detection.classId, detection.ta), 0.05, 1.0);
      detections.add(new PixelDetection(xPx, yPx, bboxWidth, bboxHeight, confidence));
    }

    // Fallback path: use detector center points if raw detections are not provided.
    if (detections.isEmpty() && latestResults != null) {
      for (NeuralDetector detector : latestResults.targets_Detector) {
        if (!isClassAllowed(detector.className)) {
          continue;
        }

        // tx_pixels/ty_pixels are center-relative, so shift to image coordinates.
        double centerX = detector.tx_pixels + FuelPickupConstants.frameWidthPx * 0.5;
        double centerY = detector.ty_pixels + FuelPickupConstants.frameHeightPx * 0.5;

        // Estimate box dimensions from target area when no corner data is available.
        double areaPixels =
            Math.max(0.0, detector.ta)
                * FuelPickupConstants.frameWidthPx
                * FuelPickupConstants.frameHeightPx;
        double approxSizePx = Math.max(8.0, Math.sqrt(areaPixels));

        double xPx = centerX;
        double yPx = centerY + approxSizePx * 0.5;
        if (xPx < 0.0
            || xPx >= FuelPickupConstants.frameWidthPx
            || yPx < 0.0
            || yPx >= FuelPickupConstants.frameHeightPx) {
          continue;
        }

        detections.add(
            new PixelDetection(
                xPx, yPx, approxSizePx, approxSizePx, clamp(detector.confidence, 0.05, 1.0)));
      }
    }

    inputs.detections = detections.toArray(PixelDetection[]::new);
  }

  private boolean isClassAllowed(String className) {
    if (targetClassName.isBlank()) {
      return true;
    }
    if (className == null || className.isBlank()) {
      return true;
    }
    return className.trim().toLowerCase(Locale.US).equals(targetClassName.toLowerCase(Locale.US));
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
