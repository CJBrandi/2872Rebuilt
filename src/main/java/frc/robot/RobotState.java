package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.FieldConstants;
import java.util.NavigableMap;
import java.util.TreeMap;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  @Getter private static RobotState instance = new RobotState();

  private static final double TURRET_BUFFER_DURATION_SECS = 2.0;

  public enum TurretShooterMode {
    SOTM,
    AIM,
    MANUAL
  }

  @Getter @Setter private Pose2d robotPose = new Pose2d();
  @Getter @Setter private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  @Getter private Translation2d bestFuelCluster = new Translation2d();
  @Getter private boolean hasBestFuelCluster = false;
  @Setter @Getter boolean strictPoseEstimation = false;

  @Getter @Setter private boolean autoEmpty = false;
  @Getter @Setter private TurretShooterMode turretShooterRequestedMode = TurretShooterMode.SOTM;

  @Getter @Setter private TurretShooterMode turretShooterActiveMode = TurretShooterMode.SOTM;

  // Turret angle buffer for vision pose estimation
  private final NavigableMap<Double, Rotation2d> turretAngleBuffer = new TreeMap<>();
  @Getter private Rotation2d latestTurretAngle = new Rotation2d();

  public record TurretObservation(double timestamp, Rotation2d angle) {}

  public void addTurretObservation(TurretObservation observation) {
    latestTurretAngle = observation.angle();
    turretAngleBuffer.put(observation.timestamp(), observation.angle());

    // Remove old entries
    while (!turretAngleBuffer.isEmpty()
        && turretAngleBuffer.firstKey() < observation.timestamp() - TURRET_BUFFER_DURATION_SECS) {
      turretAngleBuffer.pollFirstEntry();
    }
  }

  public Rotation2d getTurretAngleAtTime(double timestamp) {
    if (turretAngleBuffer.isEmpty()) {
      return latestTurretAngle;
    }

    var floor = turretAngleBuffer.floorEntry(timestamp);
    var ceiling = turretAngleBuffer.ceilingEntry(timestamp);

    if (floor == null) return ceiling.getValue();
    if (ceiling == null) return floor.getValue();
    if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

    // Interpolate between floor and ceiling
    double t = (timestamp - floor.getKey()) / (ceiling.getKey() - floor.getKey());
    return floor.getValue().interpolate(ceiling.getValue(), t);
  }

  public void setBestFuelCluster(Translation2d cluster) {
    bestFuelCluster = cluster;
    hasBestFuelCluster = true;
  }

  public void clearBestFuelCluster() {
    bestFuelCluster = new Translation2d();
    hasBestFuelCluster = false;
  }

  public boolean isSotm() {
    return turretShooterRequestedMode == TurretShooterMode.SOTM;
  }

  public void setSotm(boolean sotm) {
    turretShooterRequestedMode = sotm ? TurretShooterMode.SOTM : TurretShooterMode.AIM;
  }

  public void periodic() {
    // Calculate horizontal distance from robot to HUB
    Translation3d hubCenter = FieldConstants.Hub.topCenterPoint.get();
    double horizontalDistance = robotPose.getTranslation().getDistance(hubCenter.toTranslation2d());
    Logger.recordOutput("RobotState/AutoEmpty", autoEmpty);
    Logger.recordOutput("RobotState/TurretShooterRequestedMode", turretShooterRequestedMode.name());
    Logger.recordOutput("RobotState/TurretShooterActiveMode", turretShooterActiveMode.name());
    Logger.recordOutput("RobotState/HubDistanceMeters", horizontalDistance);
    Logger.recordOutput("RobotState/FuelPickup/HasBestCluster", hasBestFuelCluster);
    Logger.recordOutput("RobotState/FuelPickup/BestCluster", bestFuelCluster);
  }
}
