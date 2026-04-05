package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@Getter
public class Shooter {
  private static final double HORIZONTAL_REFERENCE_RAD = Math.PI / 2.0;

  // Manual mode tunable numbers (shared with Turret via same NT key)
  private static final LoggedTunableNumber manualModeEnabled =
      new LoggedTunableNumber("Manual/Enabled", 0.0);
  private static final LoggedTunableNumber manualFlywheelRPM =
      new LoggedTunableNumber("Manual/FlywheelRPM", 0.0);
  private static final LoggedTunableNumber manualHoodAngleDeg =
      new LoggedTunableNumber("Manual/PitchDeg", 25.0);

  private final Flywheel flywheel;
  private final Hood hood;

  // Goal values set by Superstructure
  private double goalExitVelocityMps = 0.0;
  private double goalHoodAngleRad = Hood.getMinAngleRad();
  private double goalHoodVelocityRadPerSec = 0.0;
  private boolean forceHoodMinimum = false;

  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheel = new Flywheel(flywheelIO);
    this.hood = new Hood(hoodIO);
  }

  public void periodic() {
    // Apply goals to components BEFORE running periodic control loops
    // This ensures the correct mode/targets are set before control runs
    boolean manualMode = manualModeEnabled.get() > 0.5;
    if (manualMode) {
      // Manual mode: use tunable RPM and angle values with closed-loop control
      double manualVelocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(manualFlywheelRPM.get());
      flywheel.runWheelVelocity(manualVelocityRadPerSec);
    } else {
      // Normal mode: use goals from Superstructure
      flywheel.setTargetExitVelocity(goalExitVelocityMps);
    }

    if (forceHoodMinimum) {
      hood.setTargetAngle(Hood.getMinAngleRad(), 0.0);
    } else if (manualMode) {
      hood.setTargetAngle(Math.toRadians(manualHoodAngleDeg.get()), 0.0);
    } else {
      // Shot pitch is measured from horizontal, while the hood mechanism rotates opposite that.
      hood.setTargetAngle(
          pitchToHoodAngleRad(goalHoodAngleRad),
          pitchToHoodVelocityRadPerSec(goalHoodVelocityRadPerSec));
    }

    flywheel.periodic();
    hood.periodic();

    Logger.recordOutput("Shooter/ManualMode", manualMode);
    Logger.recordOutput("Shooter/ForceHoodMinimum", forceHoodMinimum);
    Logger.recordOutput("Shooter/Manual/FlywheelRPM", manualFlywheelRPM.get());
    Logger.recordOutput("Shooter/Manual/HoodAngleDeg", manualHoodAngleDeg.get());
    Logger.recordOutput("Shooter/Ready", isReady());
  }

  public void setGoals(double exitVelocityMps, double hoodAngleRad, double hoodVelocityRadPerSec) {
    this.goalExitVelocityMps = exitVelocityMps;
    this.goalHoodAngleRad = hoodAngleRad;
    this.goalHoodVelocityRadPerSec = hoodVelocityRadPerSec;
  }

  static double pitchToHoodAngleRad(double pitchAngleRad) {
    return HORIZONTAL_REFERENCE_RAD - pitchAngleRad;
  }

  static double pitchToHoodVelocityRadPerSec(double pitchVelocityRadPerSec) {
    return -pitchVelocityRadPerSec;
  }

  public void setGoals(double exitVelocityMps, double hoodAngleRad) {
    setGoals(exitVelocityMps, hoodAngleRad, 0.0);
  }

  @AutoLogOutput(key = "Shooter/Ready")
  public boolean isReady() {
    return flywheel.isAtSetpoint() && hood.isAtGoal();
  }

  public boolean isFlywheelReady() {
    return flywheel.isAtSetpoint();
  }

  public boolean isHoodReady() {
    return hood.isAtGoal();
  }

  public double getExitVelocityMps() {
    return flywheel.getExitVelocityMps();
  }

  public double getHoodAngleRad() {
    return hood.getAngle().getRadians();
  }

  public void runFlywheelVolts(double volts) {
    goalExitVelocityMps = 0.0;
    flywheel.runVolts(volts);
  }

  public void stop() {
    goalExitVelocityMps = 0.0;
    goalHoodAngleRad = Hood.getMinAngleRad();
    goalHoodVelocityRadPerSec = 0.0;
  }

  public void setForceHoodMinimum(boolean forceHoodMinimum) {
    this.forceHoodMinimum = forceHoodMinimum;
  }

  public Command hoodHomingCommand() {
    return hood.homingSequence();
  }

  public Command hoodStaticCharacterizationCommand(double currentRampRateAmpsPerSec) {
    return hood.staticCharacterization(currentRampRateAmpsPerSec);
  }

  public boolean isHoodHomed() {
    return Constants.currentMode == Constants.Mode.SIM || hood.isHomed();
  }
}
