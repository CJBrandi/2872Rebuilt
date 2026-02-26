package frc.robot.subsystems.fuelpickup;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldConstants;

/** Zone policy for full-auto fuel pickup. */
public final class FuelPickupZonePolicy {
  private FuelPickupZonePolicy() {}

  public record AllowedXRange(double minX, double maxX) {
    public AllowedXRange {
      if (maxX < minX) {
        throw new IllegalArgumentException("maxX must be >= minX");
      }
    }
  }

  /**
   * Returns the legal X bounds for the current combination rule.
   *
   * <p>When shift is active: own alliance zone only. When inactive: middle + opponent zones.
   */
  public static AllowedXRange getAllowedXRange(Alliance alliance, boolean ownAllianceShiftActive) {
    double nearBoundary = FieldConstants.LinesVertical.neutralZoneNear;
    double farBoundary = FieldConstants.LinesVertical.neutralZoneFar;
    double fieldLength = FieldConstants.fieldLength;

    if (alliance == Alliance.Red) {
      return ownAllianceShiftActive
          ? new AllowedXRange(farBoundary, fieldLength)
          : new AllowedXRange(0.0, farBoundary);
    }

    return ownAllianceShiftActive
        ? new AllowedXRange(0.0, nearBoundary)
        : new AllowedXRange(nearBoundary, fieldLength);
  }

  public static boolean isClusterAllowedX(double clusterX, AllowedXRange allowedXRange) {
    return isClusterAllowedX(clusterX, allowedXRange.minX(), allowedXRange.maxX());
  }

  public static boolean isClusterAllowedX(double clusterX, double allowedXMin, double allowedXMax) {
    return clusterX >= allowedXMin && clusterX <= allowedXMax;
  }
}
