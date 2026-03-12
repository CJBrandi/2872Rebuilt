package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.util.FieldConstants;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotVectorCompensator;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled", 0.0);
  private static final LoggedTunableNumber TRENCH_LOOKAHEAD_SECS =
      new LoggedTunableNumber("Superstructure/TrenchLookaheadSecs", 0.5);
  private static final LoggedTunableNumber TRENCH_STATIC_ZONE_METERS =
      new LoggedTunableNumber("Superstructure/TrenchStaticZoneMeters", Units.inchesToMeters(6.0));
  private static final double TRENCH_CROSSING_EPSILON = 1e-9;
  private static final double AIM_PITCH_OFFSET_RAD = Units.degreesToRadians(0.5);

  @Getter private final Shooter shooter;
  @Getter private final Turret turret;
  @Getter private final Indexer indexer;
  private final ShotCalculator shotCalculator;
  private boolean trenchStowActive = false;
  private RobotState.TurretShooterMode activeTurretShooterMode = RobotState.TurretShooterMode.SOTM;
  @Getter @Setter private int fuelSimInventoryCount = 0;

  public Superstructure(Shooter shooter, Turret turret, Indexer indexer) {
    this.shooter = shooter;
    this.turret = turret;
    this.indexer = indexer;
    this.shotCalculator = ShotCalculator.getInstance();
  }

  @Override
  public void periodic() {
    var robotState = RobotState.getInstance();
    activeTurretShooterMode = resolveActiveMode(robotState);
    robotState.setTurretShooterActiveMode(activeTurretShooterMode);
    configureShotCalculator(activeTurretShooterMode);

    if (shooter.isHoodHomed()) {
      trenchStowActive = shouldStowHoodForTrench();
      shooter.setForceHoodMinimum(trenchStowActive);

      if (activeTurretShooterMode != RobotState.TurretShooterMode.MANUAL) {
        var params = shotCalculator.getParameters();
        shooter.setGoals(
            params.exitVelocity(),
            params.pitchAngle() + AIM_PITCH_OFFSET_RAD,
            params.pitchVelocity());
        turret.setTargetFieldRelativeAngle(params.turretAngle(), params.turretVelocity());
      }
    } else {
      trenchStowActive = false;
      shooter.setForceHoodMinimum(false);
    }

    shooter.periodic();
    turret.periodic();

    if (isReadyToShoot()) {
      indexer.runIntakeVelocity();
    } else {
      indexer.stop();
    }

    indexer.periodic();

    Logger.recordOutput(
        "Superstructure/RequestedTurretShooterMode",
        robotState.getTurretShooterRequestedMode().name());
    Logger.recordOutput("Superstructure/ActiveTurretShooterMode", activeTurretShooterMode.name());
    Logger.recordOutput("Superstructure/ManualEnabled", manualModeEnabled.get() > 0.5);
    Logger.recordOutput("Superstructure/TrenchStowActive", trenchStowActive);
    Logger.recordOutput("Superstructure/FuelSimInventoryCount", fuelSimInventoryCount);
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean isReadyToShoot() {
    boolean requiresStabilityGate = activeTurretShooterMode != RobotState.TurretShooterMode.MANUAL;

    return RobotState.getInstance().isAutoEmpty()
        && !trenchStowActive
        && shooter.isReady()
        && turret.isAtGoal()
        && (!requiresStabilityGate || shotCalculator.isShotStable());
  }

  private boolean shouldStowHoodForTrench() {
    var robotState = RobotState.getInstance();
    Translation2d currentTranslation = robotState.getRobotPose().getTranslation();
    Translation2d projectedTranslation =
        getProjectedTranslation(
            currentTranslation,
            robotState.getRobotVelocity(),
            robotState.getRobotPose().getRotation(),
            TRENCH_LOOKAHEAD_SECS.get());

    return shouldStowForTrench(
        currentTranslation,
        projectedTranslation,
        Constants.RobotDimensions.length,
        TRENCH_STATIC_ZONE_METERS.get());
  }

  static boolean shouldStowForTrench(
      Translation2d currentTranslation,
      Translation2d projectedTranslation,
      double robotLengthMeters,
      double trenchSafetyZoneMeters) {
    double halfRobotLengthMeters = robotLengthMeters / 2.0;
    double stowZoneMeters = halfRobotLengthMeters + Math.max(0.0, trenchSafetyZoneMeters);

    return isWithinTrenchStowZone(
            currentTranslation, FieldConstants.LinesVertical.hubCenter, stowZoneMeters)
        || isWithinTrenchStowZone(
            currentTranslation, FieldConstants.LinesVertical.oppHubCenter, stowZoneMeters)
        || crossesTrenchPlane(
            currentTranslation, projectedTranslation, FieldConstants.LinesVertical.hubCenter)
        || crossesTrenchPlane(
            currentTranslation, projectedTranslation, FieldConstants.LinesVertical.oppHubCenter);
  }

  private static Translation2d getProjectedTranslation(
      Translation2d currentTranslation,
      ChassisSpeeds robotRelativeSpeeds,
      Rotation2d robotHeading,
      double lookaheadSecs) {
    Translation2d fieldRelativeVelocity =
        ShotVectorCompensator.robotRelativeToField(robotRelativeSpeeds, robotHeading);
    return currentTranslation.plus(fieldRelativeVelocity.times(Math.max(lookaheadSecs, 0.0)));
  }

  private static boolean isWithinTrenchStowZone(
      Translation2d translation, double trenchX, double stowZoneMeters) {
    return Math.abs(translation.getX() - trenchX) <= stowZoneMeters
        && isWithinTrenchOpeningY(translation.getY());
  }

  private static boolean crossesTrenchPlane(
      Translation2d currentTranslation, Translation2d projectedTranslation, double trenchX) {
    double deltaX = projectedTranslation.getX() - currentTranslation.getX();
    if (Math.abs(deltaX) <= TRENCH_CROSSING_EPSILON) {
      return false;
    }

    double crossingProgress = (trenchX - currentTranslation.getX()) / deltaX;
    if (crossingProgress < 0.0 || crossingProgress > 1.0) {
      return false;
    }

    double crossingY =
        currentTranslation.getY()
            + (projectedTranslation.getY() - currentTranslation.getY()) * crossingProgress;
    return isWithinTrenchOpeningY(crossingY);
  }

  private static boolean isWithinTrenchOpeningY(double yMeters) {
    return isWithinInclusiveRange(
            yMeters,
            FieldConstants.LinesHorizontal.rightTrenchOpenEnd,
            FieldConstants.LinesHorizontal.rightTrenchOpenStart)
        || isWithinInclusiveRange(
            yMeters,
            FieldConstants.LinesHorizontal.leftTrenchOpenEnd,
            FieldConstants.LinesHorizontal.leftTrenchOpenStart);
  }

  private static boolean isWithinInclusiveRange(double value, double boundA, double boundB) {
    return value >= Math.min(boundA, boundB) && value <= Math.max(boundA, boundB);
  }

  private RobotState.TurretShooterMode resolveActiveMode(RobotState robotState) {
    if (manualModeEnabled.get() > 0.5) {
      return RobotState.TurretShooterMode.MANUAL;
    }
    return robotState.getTurretShooterRequestedMode();
  }

  private void configureShotCalculator(RobotState.TurretShooterMode mode) {
    switch (mode) {
      case SOTM -> {
        shotCalculator.setShootOnMoveEnabled(true);
        shotCalculator.setTargetingMode(ShotCalculator.TargetingMode.AUTO);
      }
      case AIM -> {
        shotCalculator.setShootOnMoveEnabled(false);
        shotCalculator.setTargetingMode(ShotCalculator.TargetingMode.ALLIANCE_HUB);
      }
      case MANUAL -> {
        shotCalculator.setShootOnMoveEnabled(false);
        shotCalculator.setTargetingMode(ShotCalculator.TargetingMode.AUTO);
      }
    }
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
