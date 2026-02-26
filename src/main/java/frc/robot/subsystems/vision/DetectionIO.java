package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface DetectionIO {
  @AutoLog
  public static class DetectionIOInputs {
    public boolean connected = false;
    public PixelDetection[] detections = new PixelDetection[0];
    public double timestampSeconds = 0.0;
  }

  /**
   * Pixel-space detection point sampled at the bottom-middle of the bounding box.
   *
   * <p>This is the image point that best corresponds to the ground contact for field projection.
   */
  public static record PixelDetection(
      double xPx, double yPx, double bboxWidthPx, double bboxHeightPx, double confidence) {}

  /** Track overlay data for optional visualization output. */
  public static record TrackOverlay(
      int trackId, double xPx, double yPx, double radiusPx, int clusterId) {}

  /** Cluster summary data for optional visualization output. */
  public static record ClusterOverlay(
      int clusterId, int count, double distanceMeters, double score, boolean best) {}

  /** Payload for optional simulated camera visualization rendering. */
  public static record SimFrameData(
      PixelDetection[] detections, TrackOverlay[] tracks, ClusterOverlay[] clusters) {}

  public default void updateInputs(DetectionIOInputs inputs) {}

  public default void publishSimFrame(SimFrameData frameData) {}
}
