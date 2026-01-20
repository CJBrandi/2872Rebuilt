package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.util.ShotCalculator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Shooter shooter;
  private final Turret turret;
  private final Indexer indexer;
  private final ShotCalculator shotCalculator;

  public Superstructure(Shooter shooter, Turret turret, Indexer indexer) {
    this.shooter = shooter;
    this.turret = turret;
    this.indexer = indexer;
    this.shotCalculator = ShotCalculator.getInstance();
  }

  @Override
  public void periodic() {
    // Clear cached shot parameters at start of each loop
    shotCalculator.clearShootingParameters();

    // Get shooting parameters from calculator
    var params = shotCalculator.getParameters();

    // Always aim - shooter and turret continuously track target
    shooter.setGoals(params.exitVelocity(), params.pitchAngle(), params.pitchVelocity());
    turret.setTargetFieldRelativeAngle(params.turretAngle(), params.turretVelocity());

    // Update subsystem periodics
    shooter.periodic();
    turret.periodic();
    indexer.periodic();

    // Logging
    Logger.recordOutput("Superstructure/ReadyToShoot", isReadyToShoot());
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean isReadyToShoot() {
    return shooter.isReady() && turret.isAtGoal();
  }

  public boolean isFlywheelReady() {
    return shooter.isFlywheelReady();
  }

  public boolean isHoodReady() {
    return shooter.isHoodReady();
  }

  public boolean isTurretReady() {
    return turret.isAtGoal();
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Turret getTurret() {
    return turret;
  }

  public Indexer getIndexer() {
    return indexer;
  }
}
