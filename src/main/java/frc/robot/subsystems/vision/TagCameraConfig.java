package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleFunction;

/** Per-camera settings used by tag vision IO implementations. */
public final class TagCameraConfig {
  private final String name;
  private final Transform3d fixedRobotToCamera;
  private final TurretMountSettings turretMountSettings;
  private final DoubleFunction<Rotation2d> turretAngleAtTimestamp;
  private final double stdDevFactor;

  private TagCameraConfig(
      String name,
      Transform3d fixedRobotToCamera,
      TurretMountSettings turretMountSettings,
      DoubleFunction<Rotation2d> turretAngleAtTimestamp,
      double stdDevFactor) {
    this.name = Objects.requireNonNull(name, "name");
    this.fixedRobotToCamera = fixedRobotToCamera;
    this.turretMountSettings = turretMountSettings;
    this.turretAngleAtTimestamp = turretAngleAtTimestamp;
    this.stdDevFactor = stdDevFactor;

    if ((fixedRobotToCamera == null) == (turretMountSettings == null)) {
      throw new IllegalArgumentException(
          "TagCameraConfig must be either fixed-mount or turret-mount");
    }
    if (turretMountSettings != null && turretAngleAtTimestamp == null) {
      throw new IllegalArgumentException("Turret-mount camera requires a turret angle supplier");
    }
  }

  public String name() {
    return name;
  }

  public double stdDevFactor() {
    return stdDevFactor;
  }

  public boolean isTurretMounted() {
    return turretMountSettings != null;
  }

  public Transform3d robotToCamera(double timestampSeconds) {
    if (fixedRobotToCamera != null) {
      return fixedRobotToCamera;
    }
    return turretMountSettings.robotToCamera(turretAngle(timestampSeconds));
  }

  public Optional<Transform3d> robotToTurretAxis(double timestampSeconds) {
    if (!isTurretMounted()) {
      return Optional.empty();
    }
    return Optional.of(turretMountSettings.robotToTurretAxis(turretAngle(timestampSeconds)));
  }

  private Rotation2d turretAngle(double timestampSeconds) {
    return Objects.requireNonNull(turretAngleAtTimestamp.apply(timestampSeconds), "turretAngle");
  }

  public TagCameraConfig withTurretAngleAtTimestamp(
      DoubleFunction<Rotation2d> turretAngleAtTimestamp) {
    if (turretMountSettings == null) {
      throw new IllegalStateException(
          "Cannot set turret angle supplier for fixed-mount camera \"" + name + "\"");
    }
    return new TagCameraConfig(
        name,
        null,
        turretMountSettings,
        Objects.requireNonNull(turretAngleAtTimestamp, "turretAngleAtTimestamp"),
        stdDevFactor);
  }

  public static TagCameraConfig fixed(String name, Transform3d robotToCamera) {
    return fixed(name, robotToCamera, 1.0);
  }

  public static TagCameraConfig fixed(String name, Transform3d robotToCamera, double stdDevFactor) {
    return new TagCameraConfig(
        name, Objects.requireNonNull(robotToCamera, "robotToCamera"), null, null, stdDevFactor);
  }

  public static TagCameraConfig turretMounted(String name, TurretMountSettings mountSettings) {
    return turretMounted(name, mountSettings, 1.0);
  }

  public static TagCameraConfig turretMounted(
      String name, TurretMountSettings mountSettings, double stdDevFactor) {
    return turretMounted(name, mountSettings, timestamp -> Rotation2d.kZero, stdDevFactor);
  }

  public static TagCameraConfig turretMounted(
      String name,
      TurretMountSettings mountSettings,
      DoubleFunction<Rotation2d> turretAngleAtTimestamp,
      double stdDevFactor) {
    return new TagCameraConfig(
        name,
        null,
        Objects.requireNonNull(mountSettings, "mountSettings"),
        Objects.requireNonNull(turretAngleAtTimestamp, "turretAngleAtTimestamp"),
        stdDevFactor);
  }

  public record TurretMountSettings(
      Translation3d robotCenterToTurretAxisAtZero,
      Translation3d turretAxisToCameraAtZero,
      Rotation3d cameraRotationAtZero) {
    public TurretMountSettings {
      Objects.requireNonNull(robotCenterToTurretAxisAtZero, "robotCenterToTurretAxisAtZero");
      Objects.requireNonNull(turretAxisToCameraAtZero, "turretAxisToCameraAtZero");
      Objects.requireNonNull(cameraRotationAtZero, "cameraRotationAtZero");
    }

    public Transform3d robotToCamera(Rotation2d turretAngle) {
      Transform3d robotToTurretAxis = robotToTurretAxis(turretAngle);
      Transform3d turretAxisToCamera =
          new Transform3d(turretAxisToCameraAtZero, cameraRotationAtZero);
      return robotToTurretAxis.plus(turretAxisToCamera);
    }

    public Transform3d robotToTurretAxis(Rotation2d turretAngle) {
      return new Transform3d(
          robotCenterToTurretAxisAtZero, new Rotation3d(0.0, 0.0, turretAngle.getRadians()));
    }

    public static TurretMountSettings fromReferenceModule(
        Translation3d robotCenterToReferenceModule,
        Translation3d referenceModuleToTurretAxisAtZero,
        Translation3d turretAxisToCameraAtZero,
        Rotation3d cameraRotationAtZero) {
      return new TurretMountSettings(
          Objects.requireNonNull(robotCenterToReferenceModule, "robotCenterToReferenceModule")
              .plus(
                  Objects.requireNonNull(
                      referenceModuleToTurretAxisAtZero, "referenceModuleToTurretAxisAtZero")),
          turretAxisToCameraAtZero,
          cameraRotationAtZero);
    }
  }
}
