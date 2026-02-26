package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.util.FuelSim;
import frc.robot.util.ShotCalculator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Shooter shooter;
  private final Turret turret;
  private final Indexer indexer;
  private final ShotCalculator shotCalculator;
  private int fuelSimInventoryCount = 0;

  public Superstructure(Shooter shooter, Turret turret, Indexer indexer) {
    this.shooter = shooter;
    this.turret = turret;
    this.indexer = indexer;
    this.shotCalculator = ShotCalculator.getInstance();
  }

  @Override
  public void periodic() {
    // Get shooting parameters from calculator
    var params = shotCalculator.getParameters();

    if (shooter.isHoodHomed() && turret.isHomed()) {
      shooter.setGoals(params.exitVelocity(), params.pitchAngle(), params.pitchVelocity());
      turret.setTargetFieldRelativeAngle(params.turretAngle(), params.turretVelocity());
    }

    shooter.periodic();
    turret.periodic();
    indexer.periodic();

    Logger.recordOutput("Superstructure/ReadyToShoot", isReadyToShoot());
    Logger.recordOutput("Superstructure/FuelSimInventoryCount", fuelSimInventoryCount);
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean isReadyToShoot() {
    return shooter.isReady() && turret.isAtGoal() && shotCalculator.isShotStable();
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

  /** Increments simulated shooter inventory when a fuel enters the intake box. */
  public void addFuelSimIntaked() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }
    fuelSimInventoryCount++;
  }

  /** Launches a fuel in simulation. Call this when shooting a ball. Only has effect in SIM mode. */
  public void launchFuelSim() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }
    if (fuelSimInventoryCount <= 0) {
      return;
    }

    // Use raw lookup table values for simulation to verify the trajectory solver
    // This bypasses efficiency factor and hood angle convention issues
    var params = shotCalculator.getParameters();

    FuelSim.getInstance()
        .launchFuel(
            MetersPerSecond.of(params.exitVelocity()), // Raw lookup table velocity (no efficiency)
            Radians.of(params.pitchAngle()), // Raw lookup table pitch (from horizontal)
            Radians.of(turret.getFieldRelativeAngle().getRadians()),
            Meters.of(Constants.launchHeight));
    fuelSimInventoryCount--;
  }
}
