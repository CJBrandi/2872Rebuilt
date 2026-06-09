package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
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
  private static final LoggedTunableNumber TRENCH_INTAKE_EXTRA_MARGIN_SECS =
      new LoggedTunableNumber("Superstructure/TrenchIntakeExtraMarginSecs", 0.5);
  private static final LoggedTunableNumber TRENCH_STATIC_ZONE_METERS =
      new LoggedTunableNumber("Superstructure/TrenchStaticZoneMeters", Units.inchesToMeters(6.0));
  private static final double TRENCH_CROSSING_EPSILON = 1e-9;
  private static final double TRENCH_INTAKE_EXTENSION_METERS = Units.inchesToMeters(12.0);
  private static final double AIM_PITCH_OFFSET_RAD = Units.degreesToRadians(0.5);

  @Getter private final Shooter shooter;
  @Getter private final Turret turret;
  @Getter private final Indexer indexer;
  private final Intake intake;
  private final ShotCalculator shotCalculator;
  private boolean trenchStowActive = false;
  private boolean trenchIntakeDeployActive = false;
  private RobotState.TurretShooterMode activeTurretShooterMode = RobotState.TurretShooterMode.SOTM;
  @Getter @Setter private int fuelSimInventoryCount = 0;

  public Superstructure(Shooter shooter, Turret turret, Indexer indexer, Intake intake) {
    this.shooter = shooter;
    this.turret = turret;
    this.indexer = indexer;
    this.intake = intake;
    this.shotCalculator = ShotCalculator.getInstance();
  }

  @Override
  public void periodic() {
    var robotState = RobotState.getInstance();
    activeTurretShooterMode = resolveActiveMode(robotState);
    robotState.setTurretShooterActiveMode(activeTurretShooterMode);
    configureShotCalculator(activeTurretShooterMode);

    trenchIntakeDeployActive = updateTrenchIntakeDeployState();
    intake.setTrenchAutoDeployEnabled(trenchIntakeDeployActive);

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
    Logger.recordOutput("Superstructure/TrenchIntakeDeployActive", trenchIntakeDeployActive);
    Logger.recordOutput("Superstructure/FuelSimInventoryCount", fuelSimInventoryCount);
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean isReadyToShoot() {
    return RobotState.getInstance().isAutoEmpty()
        && !trenchStowActive
        && shooter.isReady()
        && turret.isAtGoal();
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

  private boolean shouldDeployIntakeForTrench() {
    if (!intake.requiresTrenchAutoDeploy()) {
      return false;
    }

    var robotState = RobotState.getInstance();
    boolean pivotStowed = intake.isPivotStowedForTrench();
    if (!trenchIntakeDeployActive && !pivotStowed) {
      Logger.recordOutput(
          "Superstructure/TrenchIntakeRequiredLookaheadSecs",
          getRequiredIntakeDeployLookaheadSecs());
      Logger.recordOutput("Superstructure/TrenchIntakeEnvelopeActive", false);
      return false;
    }

    double lookaheadSecs = getRequiredIntakeDeployLookaheadSecs();
    boolean trenchEnvelopeActive =
        shouldDeployRearIntakeForTrench(
            robotState.getRobotPose().getTranslation(),
            robotState.getRobotPose().getRotation(),
            robotState.getRobotVelocity(),
            lookaheadSecs,
            Constants.RobotDimensions.length,
            TRENCH_INTAKE_EXTENSION_METERS,
            TRENCH_STATIC_ZONE_METERS.get());

    Logger.recordOutput("Superstructure/TrenchIntakeRequiredLookaheadSecs", lookaheadSecs);
    Logger.recordOutput("Superstructure/TrenchIntakeEnvelopeActive", trenchEnvelopeActive);

    return resolveTrenchIntakeDeployState(
        trenchIntakeDeployActive, true, pivotStowed, trenchEnvelopeActive);
  }

  private boolean updateTrenchIntakeDeployState() {
    boolean deployActive = shouldDeployIntakeForTrench();

    if (!intake.requiresTrenchAutoDeploy()) {
      Logger.recordOutput("Superstructure/TrenchIntakeRequiredLookaheadSecs", 0.0);
      Logger.recordOutput("Superstructure/TrenchIntakeEnvelopeActive", false);
    }

    return deployActive;
  }

  private double getRequiredIntakeDeployLookaheadSecs() {
    return intake.getWorstCaseDeployTimeSecs()
        + Math.max(0.0, TRENCH_INTAKE_EXTRA_MARGIN_SECS.get());
  }

  static boolean resolveTrenchIntakeDeployState(
      boolean currentDeployActive,
      boolean requiresAutoDeploy,
      boolean pivotStowed,
      boolean trenchEnvelopeActive) {
    if (!requiresAutoDeploy) {
      return false;
    }
    if (currentDeployActive) {
      return trenchEnvelopeActive;
    }
    return pivotStowed && trenchEnvelopeActive;
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

  static boolean shouldDeployRearIntakeForTrench(
      Translation2d currentTranslation,
      Rotation2d currentHeading,
      ChassisSpeeds robotRelativeSpeeds,
      double lookaheadSecs,
      double robotLengthMeters,
      double intakeExtensionMeters,
      double trenchSafetyZoneMeters) {
    Translation2d projectedTranslation =
        getProjectedTranslation(
            currentTranslation, robotRelativeSpeeds, currentHeading, lookaheadSecs);
    Rotation2d projectedHeading =
        getProjectedHeading(currentHeading, robotRelativeSpeeds, lookaheadSecs);

    return shouldDeployRearIntakeForTrench(
        currentTranslation,
        projectedTranslation,
        currentHeading,
        projectedHeading,
        robotLengthMeters,
        intakeExtensionMeters,
        trenchSafetyZoneMeters);
  }

  static boolean shouldDeployRearIntakeForTrench(
      Translation2d currentTranslation,
      Translation2d projectedTranslation,
      Rotation2d currentHeading,
      Rotation2d projectedHeading,
      double robotLengthMeters,
      double intakeExtensionMeters,
      double trenchSafetyZoneMeters) {
    Translation2d currentIntakeTip =
        getRearIntakeTipTranslation(
            currentTranslation, currentHeading, robotLengthMeters, intakeExtensionMeters);
    Translation2d projectedIntakeTip =
        getRearIntakeTipTranslation(
            projectedTranslation, projectedHeading, robotLengthMeters, intakeExtensionMeters);
    double deployZoneMeters = Math.max(0.0, trenchSafetyZoneMeters);

    return isWithinTrenchDeployEnvelope(
            currentIntakeTip,
            projectedIntakeTip,
            FieldConstants.LinesVertical.hubCenter,
            deployZoneMeters)
        || isWithinTrenchDeployEnvelope(
            currentIntakeTip,
            projectedIntakeTip,
            FieldConstants.LinesVertical.oppHubCenter,
            deployZoneMeters);
  }

  private static boolean isWithinTrenchDeployEnvelope(
      Translation2d currentIntakeTip,
      Translation2d projectedIntakeTip,
      double trenchX,
      double deployZoneMeters) {
    return isWithinTrenchStowZone(currentIntakeTip, trenchX, deployZoneMeters)
        || isWithinTrenchStowZone(projectedIntakeTip, trenchX, deployZoneMeters)
        || crossesTrenchPlane(currentIntakeTip, projectedIntakeTip, trenchX);
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

  private static Rotation2d getProjectedHeading(
      Rotation2d currentHeading, ChassisSpeeds robotRelativeSpeeds, double lookaheadSecs) {
    return currentHeading.plus(
        Rotation2d.fromRadians(
            robotRelativeSpeeds.omegaRadiansPerSecond * Math.max(lookaheadSecs, 0.0)));
  }

  static Translation2d getRearIntakeTipTranslation(
      Translation2d robotTranslation,
      Rotation2d robotHeading,
      double robotLengthMeters,
      double intakeExtensionMeters) {
    double rearIntakeDistanceMeters =
        robotLengthMeters / 2.0 + Math.max(0.0, intakeExtensionMeters);
    return robotTranslation.plus(
        new Translation2d(rearIntakeDistanceMeters, robotHeading.plus(Rotation2d.kPi)));
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

    // Use shot calculator outputs directly for simulation. This still bypasses
    // efficiency factor and hood angle convention issues while reflecting any
    // lob-specific command scaling.
    var params = shotCalculator.getParameters();

    FuelSim.getInstance()
        .launchFuel(
            MetersPerSecond.of(params.exitVelocity()), // Commanded exit velocity (no efficiency)
            Radians.of(params.pitchAngle()), // Commanded pitch from horizontal
            Radians.of(turret.getFieldRelativeAngle().getRadians()),
            Meters.of(Constants.launchHeight));
    fuelSimInventoryCount--;
  }
}
