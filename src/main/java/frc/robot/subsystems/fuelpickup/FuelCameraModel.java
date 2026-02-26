package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/** Pinhole camera model for simulated fuel detections. */
public class FuelCameraModel {
  public record PixelObservation(double xPx, double yPx, double radiusPx, double depthMeters) {}

  private final int frameWidthPx;
  private final int frameHeightPx;
  private final double focalXPx;
  private final double focalYPx;
  private final double centerXPx;
  private final double centerYPx;

  public FuelCameraModel(
      int frameWidthPx, int frameHeightPx, double horizontalFovDeg, double verticalFovDeg) {
    this.frameWidthPx = frameWidthPx;
    this.frameHeightPx = frameHeightPx;
    this.centerXPx = frameWidthPx / 2.0;
    this.centerYPx = frameHeightPx / 2.0;
    this.focalXPx = centerXPx / Math.tan(Math.toRadians(horizontalFovDeg) / 2.0);
    this.focalYPx = centerYPx / Math.tan(Math.toRadians(verticalFovDeg) / 2.0);
  }

  public Optional<PixelObservation> project(
      Translation3d fieldPoint, Pose3d cameraPose, double objectRadiusMeters) {
    Pose3d pointRelativeToCamera = new Pose3d(fieldPoint, Rotation3d.kZero).relativeTo(cameraPose);
    double depthMeters = pointRelativeToCamera.getX();
    if (depthMeters <= 1e-4) {
      return Optional.empty();
    }

    double xPx = centerXPx - focalXPx * (pointRelativeToCamera.getY() / depthMeters);
    double yPx = centerYPx - focalYPx * (pointRelativeToCamera.getZ() / depthMeters);
    if (!isInsideFrame(xPx, yPx)) {
      return Optional.empty();
    }

    double radiusPx = Math.max(1.5, focalXPx * objectRadiusMeters / depthMeters);
    return Optional.of(new PixelObservation(xPx, yPx, radiusPx, depthMeters));
  }

  public Optional<Translation3d> backProjectToPlane(
      double xPx, double yPx, Pose3d cameraPose, double planeZMeters) {
    if (!isInsideFrame(xPx, yPx)) {
      return Optional.empty();
    }

    Translation3d rayCamera =
        new Translation3d(1.0, (centerXPx - xPx) / focalXPx, (centerYPx - yPx) / focalYPx);
    Translation3d rayField = rayCamera.rotateBy(cameraPose.getRotation());
    if (Math.abs(rayField.getZ()) < 1e-6) {
      return Optional.empty();
    }

    double t = (planeZMeters - cameraPose.getZ()) / rayField.getZ();
    if (t <= 0.0) {
      return Optional.empty();
    }

    return Optional.of(cameraPose.getTranslation().plus(rayField.times(t)));
  }

  private boolean isInsideFrame(double xPx, double yPx) {
    return xPx >= 0.0 && xPx < frameWidthPx && yPx >= 0.0 && yPx < frameHeightPx;
  }
}
