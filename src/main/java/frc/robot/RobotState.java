package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.NavigableMap;
import java.util.TreeMap;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  @Getter private static RobotState instance = new RobotState();

  private static final double TURRET_BUFFER_DURATION_SECS = 2.0;

  @Getter @Setter private Pose2d robotPose = new Pose2d();
  @Getter @Setter private ChassisSpeeds robotVelocity = new ChassisSpeeds();

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
}
